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
        plane_model, inliers = pcd.segment_plane(0.005, 3, 1000)
        if len(inliers) < 100:
            self.get_logger().warn("Too few inliers for plane fitting.")
            return

        center = np.mean(np.asarray(pcd.select_by_index(inliers).points), axis=0)
        normal = np.array(plane_model[:3])
        normal /= np.linalg.norm(normal)

        self.z_normal_list.append(normal)
        if len(self.z_normal_list) > 10:
            self.z_normal_list.pop(0)

        if len(self.z_normal_list) < 10:
            self.get_logger().info(f"目前收集 {len(self.z_normal_list)} 筆法向量，等待穩定...")
            return

        normal_avg = np.mean(self.z_normal_list, axis=0)
        normal_avg /= np.linalg.norm(normal_avg)
        base_z = np.array([0.0, 0.0, -1.0])

        if np.dot(normal_avg, base_z) < 0:
            self.get_logger().warn("法向量方向相反，自動反轉")
            normal_avg = -normal_avg

        angle_to_base = np.arccos(np.clip(np.dot(normal_avg, base_z), -1, 1)) * 180.0 / np.pi
        self.get_logger().info(f"[法向量角度] mean: {angle_to_base:.2f}°")

        if time.time() - self.last_sent_time < 1.5:
            self.get_logger().warn("Already sending or sent.")
            return

        self.last_sent_time = time.time()

        # 使用 normal_avg 作為 z 軸
        z_axis = normal_avg
        x_temp = np.array([1.0, 0.0, 0.0])
        if np.abs(np.dot(z_axis, x_temp)) > 0.95:
            x_temp = np.array([0.0, 1.0, 0.0])

        x_axis = np.cross(x_temp, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        rot_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)
        if np.linalg.det(rot_matrix) < 0:
            self.get_logger().warn("檢測到左手系統，自動反轉 z 軸")
            z_axis = -z_axis
            rot_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)

        quat = R.from_matrix(rot_matrix).as_quat()

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'camera_depth_optical_frame'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position = Point(x=center[0], y=center[1], z=center[2])
        pose_msg.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        self.get_logger().info(f"Pose : {center[0], center[1], center[2]}")
        final_euler = R.from_matrix(rot_matrix).as_euler('xyz', degrees=True)
        self.get_logger().info(f"Euler : {final_euler}")
        self.pose_pub.publish(pose_msg)

        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = 'grasp_pose'
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