import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Quaternion, Vector3
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from tm_robot_if.srv import PoseSrv, CaptureImage
import numpy as np
import cv2
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import time

class GraspEstimatorNode(Node):
    def __init__(self):
        super().__init__('grasp_estimator_node')
        self.bridge = CvBridge()
        self.z_normal_list = []
        self.latest_depth = None
        self.latest_mask = None
        self.sending = False
        self.pose_sent = False
        self.last_sent_time = 0.0

        self.K = np.array([[904.87290509, 0.0, 634.39373174],
                           [0.0, 903.32017544, 369.06447261],
                           [0.0, 0.0, 1.0]])

        self.dist_coeffs = np.array([
            0.05773164354848198,
            0.5821827164855585,
            0.004314151191910511,
            -0.001112447533308546,
            -2.461367058886307
        ])

        self.intrinsic = {
            'fx': 904.8729050868374,
            'fy': 903.3201754368574,
            'cx': 634.3937317400505,
            'cy': 369.0644726085734
        }

        self.create_subscription(Image, '/tm_robot/depth_image', self.depth_callback, 10)
        self.create_service(CaptureImage, 'grab_detect', self.handle_mask_service)
        self.pose_pub = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        self.br = TransformBroadcaster(self)
        self.pose_client = self.create_client(PoseSrv, 'thing_pose')

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.try_estimate()

    def handle_mask_service(self, request, response):
        try:
            self.latest_mask = self.bridge.imgmsg_to_cv2(request.mask, desired_encoding='mono8')
            self.get_logger().info("成功接收到 binary mask，開始估算抓取姿態")
            self.try_estimate()
            response.success = True
        except Exception as e:
            self.get_logger().error(f"mask 轉換失敗: {e}")
            response.success = False
        return response

    def try_estimate(self):
        if self.latest_depth is None or self.latest_mask is None:
            return

        raw_depth = self.latest_depth.astype(np.float32)
        depth_undistorted = cv2.undistort(raw_depth, self.K, self.dist_coeffs)
        mask_undistorted = cv2.undistort(self.latest_mask, self.K, self.dist_coeffs)

        fx, fy = self.intrinsic['fx'], self.intrinsic['fy']
        cx, cy = self.intrinsic['cx'], self.intrinsic['cy']
        depth = depth_undistorted * 0.001

        if mask_undistorted.shape != depth.shape:
            mask_undistorted = cv2.resize(mask_undistorted, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_NEAREST)

        mask_binary = mask_undistorted > 0
        xs, ys = np.meshgrid(np.arange(depth.shape[1]), np.arange(depth.shape[0]))
        xs, ys, zs = xs[mask_binary], ys[mask_binary], depth[mask_binary]

        valid = zs > 0
        if np.sum(valid) < 100:
            self.get_logger().warn("Too few valid depth points.")
            return

        xs, ys, zs = xs[valid], ys[valid], zs[valid]
        x3d = (xs - cx) * zs / fx
        y3d = (ys - cy) * zs / fy
        points = np.stack((x3d, y3d, zs), axis=-1)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # 嘗試不同的平面擬合參數，專門尋找水平平面
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, 
                                                ransac_n=3, 
                                                num_iterations=1000)
        
        if len(inliers) < 100:
            self.get_logger().warn("Too few inliers for plane fitting.")
            return
            
        # 計算inliers比例
        inlier_ratio = len(inliers) / len(points)
        self.get_logger().info(f"平面擬合結果: {len(inliers)}/{len(points)} 點 ({inlier_ratio:.2%})")
        
        if inlier_ratio < 0.5:  # 如果少於50%的點符合平面
            self.get_logger().warn(f"平面擬合品質較差 ({inlier_ratio:.1%})，可能不是平坦表面")

        center = np.mean(np.asarray(pcd.select_by_index(inliers).points), axis=0)
        normal = np.array(plane_model[:3])
        normal /= np.linalg.norm(normal)

        # IMPROVED: For top-down view, ensure normal points toward camera (negative Z direction)
        camera_direction = np.array([0.0, 0.0, -1.0])  # Camera looks down negative Z
        
        # 檢查法向量方向，確保指向相機
        dot_product = np.dot(normal, camera_direction)
        self.get_logger().info(f"法向量與相機方向點積: {dot_product:.3f}")
        
        if dot_product > 0:
            self.get_logger().info("法向量指向攝影機，方向正確")
        else:
            self.get_logger().warn("法向量遠離攝影機，自動反轉")
            normal = -normal
            
        # 額外檢查：法向量應該主要指向Z軸負方向（俯視配置）
        z_component = abs(normal[2])
        self.get_logger().info(f"法向量Z分量: {normal[2]:.3f}, 絕對值: {z_component:.3f}")
        
        if z_component < 0.5:  # Z分量太小，可能不是水平平面
            self.get_logger().warn(f"檢測到的可能不是水平平面，Z分量過小: {z_component:.3f}")
            self.get_logger().warn(f"完整法向量: [{normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f}]")

        self.z_normal_list.append(normal)
        if len(self.z_normal_list) > 10:
            self.z_normal_list.pop(0)

        if len(self.z_normal_list) < 5:  # Reduced from 10 for faster response
            self.get_logger().info(f"目前收集 {len(self.z_normal_list)} 筆法向量，等待穩定...")
            return

        normal_avg = np.mean(self.z_normal_list, axis=0)
        normal_avg /= np.linalg.norm(normal_avg)

        # IMPROVED: Calculate angle relative to camera's Z-axis (top-down view)
        dot_with_camera = np.dot(normal_avg, camera_direction)
        angle_to_camera = np.arccos(np.clip(abs(dot_with_camera), 0, 1)) * 180.0 / np.pi
        
        self.get_logger().info(f"[法向量分析]")
        self.get_logger().info(f"  平均法向量: [{normal_avg[0]:.3f}, {normal_avg[1]:.3f}, {normal_avg[2]:.3f}]")
        self.get_logger().info(f"  與相機Z軸點積: {dot_with_camera:.3f}")
        self.get_logger().info(f"  角度 (與水平面夾角): {angle_to_camera:.2f}°")

        # IMPROVED: 判斷表面類型並選擇適合的抓取策略
        if angle_to_camera < 30.0:  # 接近水平
            self.get_logger().info(f"✓ 檢測到水平表面 (傾斜 {angle_to_camera:.1f}°)，使用俯視抓取")
            surface_type = "horizontal"
            grasp_strategy = "top_down"
        elif angle_to_camera > 60.0:  # 接近垂直
            self.get_logger().info(f"✓ 檢測到垂直表面/側面 (角度 {angle_to_camera:.1f}°)，使用側面抓取")
            surface_type = "vertical"
            grasp_strategy = "side_grasp"
        else:  # 中等傾斜
            self.get_logger().info(f"✓ 檢測到傾斜表面 (角度 {angle_to_camera:.1f}°)，使用傾斜抓取")
            surface_type = "inclined"
            grasp_strategy = "angled_grasp"

        if time.time() - self.last_sent_time < 1.0:  # Reduced timeout
            self.get_logger().warn("已在處理或剛發送完成")
            return

        self.last_sent_time = time.time()

        # IMPROVED: 針對吸盤抓取生成適合的姿態
        # 吸盤需要垂直於表面接近，Z軸指向表面法向量方向
        if grasp_strategy == "top_down":
            # 俯視吸取：z軸沿法向量方向（向下）
            z_axis = normal_avg
            self.get_logger().info("🔧 生成俯視吸盤姿態 - Z軸向下垂直於水平面")
            
        elif grasp_strategy == "side_grasp":
            # 側面吸取：z軸沿法向量方向（垂直於側面，指向相機）
            # 對於吸盤，我們希望吸盤面垂直貼合側面
            z_axis = normal_avg  # 法向量指向相機方向
            self.get_logger().info("🔧 生成側面吸盤姿態 - Z軸垂直於側面指向相機")
            
        else:  # angled_grasp
            # 傾斜吸取：z軸沿法向量方向
            z_axis = normal_avg
            self.get_logger().info("🔧 生成傾斜吸盤姿態 - Z軸垂直於傾斜面")
        
        # 生成對應的坐標軸
        # 根據抓取策略選擇合適的參考軸
        if grasp_strategy == "side_grasp":
            # 側面抓取時，優先選擇水平方向作為參考
            x_temp = np.array([1.0, 0.0, 0.0])  # 水平方向
            if np.abs(np.dot(z_axis, x_temp)) > 0.95:
                x_temp = np.array([0.0, 1.0, 0.0])
        else:
            # 俯視和傾斜抓取時的標準處理
            x_temp = np.array([1.0, 0.0, 0.0])
            if np.abs(np.dot(z_axis, x_temp)) > 0.95:
                x_temp = np.array([0.0, 1.0, 0.0])

        x_axis = np.cross(x_temp, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        rot_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)
        
        # Ensure right-handed coordinate system
        if np.linalg.det(rot_matrix) < 0:
            self.get_logger().warn("檢測到左手系統，自動調整")
            y_axis = -y_axis
            rot_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)
            
        self.get_logger().info(f"📐 最終抓取軸向:")
        self.get_logger().info(f"  X軸: [{x_axis[0]:.3f}, {x_axis[1]:.3f}, {x_axis[2]:.3f}]")
        self.get_logger().info(f"  Y軸: [{y_axis[0]:.3f}, {y_axis[1]:.3f}, {y_axis[2]:.3f}]")
        self.get_logger().info(f"  Z軸: [{z_axis[0]:.3f}, {z_axis[1]:.3f}, {z_axis[2]:.3f}]")

        quat = R.from_matrix(rot_matrix).as_quat()

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'camera_depth_optical_frame'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position = Point(x=center[0], y=center[1], z=center[2])
        pose_msg.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        self.get_logger().info(f"🎯 抓取目標資訊:")
        self.get_logger().info(f"  策略: {grasp_strategy} ({surface_type})")
        self.get_logger().info(f"  位置 (相機座標): x={center[0]:.4f}, y={center[1]:.4f}, z={center[2]:.4f}")
        final_euler = R.from_matrix(rot_matrix).as_euler('xyz', degrees=True)
        self.get_logger().info(f"  姿態 (歐拉角): roll={final_euler[0]:.2f}°, pitch={final_euler[1]:.2f}°, yaw={final_euler[2]:.2f}°")
        
        # 根據策略給出吸盤操作提示
        if grasp_strategy == "top_down":
            self.get_logger().info("💡 建議: 吸盤垂直向下接近水平表面")
        elif grasp_strategy == "side_grasp":
            self.get_logger().info("💡 建議: 吸盤垂直接近側面（面向相機的表面）")
            self.get_logger().info("💡 注意: Z軸指向相機，吸盤面將貼合物體側面")
        else:
            self.get_logger().info("💡 建議: 吸盤垂直於傾斜面接近")
        
        self.pose_pub.publish(pose_msg)

        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = f'grasp_pose_{grasp_strategy}'
        tf_msg.transform.translation = Vector3(x=center[0], y=center[1], z=center[2])
        tf_msg.transform.rotation = pose_msg.pose.orientation
        self.br.sendTransform(tf_msg)

        if not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("thing_pose service not available.")
            return

        req = PoseSrv.Request()
        req.pose = pose_msg.pose
        future = self.pose_client.call_async(req)

        def handle_result(fut):
            try:
                result = fut.result()
                if result.success:
                    self.get_logger().info("Pose sent to service successfully.")
                else:
                    self.get_logger().error("PoseSrv returned failure.")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")

        future.add_done_callback(handle_result)

def main(args=None):
    rclpy.init(args=args)
    node = GraspEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()