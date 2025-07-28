import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Pose as PoseMsg
import numpy as np
from scipy.spatial.transform import Rotation as R

from tm_robot_if.srv import PoseSrv
from tm_msgs.srv import SetPositions

from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class Camera2Base(Node):
    def __init__(self):
        super().__init__("camera2ee")
        client_cb_group = ReentrantCallbackGroup()
        timer_cb_group = client_cb_group
        
        self.thing_pose_srv = self.create_service(PoseSrv, 'thing_pose', self.pose_receive, callback_group=client_cb_group)
        self.base_pose_pub = self.create_publisher(Pose, '/cube_position', 10)
        
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.T_camera_to_ee = self.create_hand_eye_transform()
        
        # IMPROVED: Add configurable offsets for fine-tuning
        self.offset_x = 0.0  # Adjustable X offset
        self.offset_y = -0.02  # Adjustable Y offset 
        self.offset_z = 0.05  # Adjustable Z offset (approach distance)

        self.get_logger().info("Camera2Base node initialized")
        self.get_logger().info(f"Hand-eye transform:\n{self.T_camera_to_ee}")
        self.get_logger().info(f"Applied offsets: x={self.offset_x}, y={self.offset_y}, z={self.offset_z}")

    def create_hand_eye_transform(self):
        trans = np.array([0.03182497415436079, 0.033778346369806436, 0.06894045731602626])
        quat = [-0.0005731808229620196, 0.005829958484186053, -0.9999819369362807, 0.001344934563953267]
        T = np.eye(4)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = trans
        return T

    def lookup_ee_to_base(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame='base',
                source_frame='flange',
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            trans = tf.transform.translation
            rot = tf.transform.rotation
            T = np.eye(4)
            T[:3, :3] = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
            T[:3, 3] = [trans.x, trans.y, trans.z]
            
            # IMPROVED: Log current end-effector position for debugging
            self.get_logger().info(f"Current EE position: x={trans.x:.4f}, y={trans.y:.4f}, z={trans.z:.4f}")
            return T
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None

    def validate_pose(self, pose, frame_name):
        """Validate if pose values are reasonable"""
        pos = [pose.position.x, pose.position.y, pose.position.z]
        if any(abs(p) > 2.0 for p in pos):  # Check if position is within 2m range
            self.get_logger().warn(f"{frame_name} position seems unreasonable: {pos}")
            return False
        
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        quat_norm = np.linalg.norm(quat)
        if abs(quat_norm - 1.0) > 0.1:  # Check if quaternion is normalized
            self.get_logger().warn(f"{frame_name} quaternion not normalized: norm={quat_norm}")
            return False
            
        return True
    
    def pose_receive(self, request, response):
        cam_pose = request.pose

        # IMPROVED: Validate input pose
        if not self.validate_pose(cam_pose, "Camera"):
            self.get_logger().error("Invalid camera pose received")
            response.success = False
            return response

        # Convert camera pose to 4x4 matrix
        T_cam = np.eye(4)
        T_cam[:3, :3] = R.from_quat([
            cam_pose.orientation.x,
            cam_pose.orientation.y,
            cam_pose.orientation.z,
            cam_pose.orientation.w
        ]).as_matrix()
        T_cam[:3, 3] = [cam_pose.position.x, cam_pose.position.y, cam_pose.position.z]

        self.get_logger().info(
            f"[接收相機座標]\n"
            f"  位置: x={cam_pose.position.x:.4f}, y={cam_pose.position.y:.4f}, z={cam_pose.position.z:.4f}"
        )

        # Get current end-effector to base transform
        T_ee_to_base = self.lookup_ee_to_base()
        if T_ee_to_base is None:
            response.success = False
            return response

        # IMPROVED: Step-by-step transformation with logging
        self.get_logger().info("=== 座標轉換步驟 ===")
        
        # Step 1: Camera to end-effector
        T_ee = self.T_camera_to_ee @ T_cam
        pos_ee = T_ee[:3, 3]
        self.get_logger().info(f"1. 相機→末端座標: x={pos_ee[0]:.4f}, y={pos_ee[1]:.4f}, z={pos_ee[2]:.4f}")
        
        # Step 2: End-effector to base
        T_cam_to_ee_corrected = self.T_camera_to_ee @ T_cam
        T_base = T_ee_to_base @ T_cam_to_ee_corrected
        pos_base_raw = T_base[:3, 3]
        self.get_logger().info(f"2. 末端→基座標 (原始): x={pos_base_raw[0]:.4f}, y={pos_base_raw[1]:.4f}, z={pos_base_raw[2]:.4f}")
        
        # Step 3: Apply offsets
        pos_final = pos_base_raw.copy()
        pos_final[0] += self.offset_x
        pos_final[1] += self.offset_y  
        pos_final[2] += self.offset_z
        self.get_logger().info(f"3. 套用偏移量後: x={pos_final[0]:.4f}, y={pos_final[1]:.4f}, z={pos_final[2]:.4f}")

        quat = R.from_matrix(T_base[:3, :3]).as_quat()

        # Create final pose
        base_pose = Pose()
        base_pose.position.x = pos_final[0]
        base_pose.position.y = pos_final[1]
        base_pose.position.z = pos_final[2]
        base_pose.orientation.x = quat[0]
        base_pose.orientation.y = quat[1]
        base_pose.orientation.z = quat[2]
        base_pose.orientation.w = quat[3]

        # IMPROVED: Validate output pose
        if not self.validate_pose(base_pose, "Base"):
            self.get_logger().error("Invalid base pose calculated")
            response.success = False
            return response

        # IMPROVED: Log final results with more detail
        euler = R.from_quat(quat).as_euler('xyz', degrees=True)
        self.get_logger().info(
            f"=== 最終結果 (基座座標) ===\n"
            f"  位置: x={base_pose.position.x:.4f}, y={base_pose.position.y:.4f}, z={base_pose.position.z:.4f}\n"
            f"  姿態: roll={euler[0]:.2f}°, pitch={euler[1]:.2f}°, yaw={euler[2]:.2f}°"
        )

        self.base_pose_pub.publish(base_pose)
        self.get_logger().info("已發布轉換後的座標到 /cube_position")

        # Publish TF for visualization
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'base'
        tf_msg.child_frame_id = 'target_object'
        tf_msg.transform.translation.x = pos_final[0]
        tf_msg.transform.translation.y = pos_final[1]
        tf_msg.transform.translation.z = pos_final[2]
        tf_msg.transform.rotation = base_pose.orientation
        self.br.sendTransform(tf_msg)

        response.success = True
        return response

def main(args=None):
    rclpy.init()
    node = Camera2Base()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info('Camera2Base node started, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()