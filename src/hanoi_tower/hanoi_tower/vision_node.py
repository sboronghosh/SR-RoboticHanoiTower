import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R

try:
    ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    ARUCO_PARAMS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
except AttributeError:
    ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
    ARUCO_PARAMS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    ARUCO_DETECTOR = None

CUBES = {
    1: {'name': 'small',  'marker_size': 0.035, 'cube_depth': 0.04},
    2: {'name': 'medium', 'marker_size': 0.04,  'cube_depth': 0.05},
    3: {'name': 'large',  'marker_size': 0.05,  'cube_depth': 0.06},
}

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rgb_image = None
        self.depth_image = None
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.create_subscription(
            Image, '/camera/color/image_raw', 
            self._rgb_callback, sensor_qos)
        self.create_subscription(
            Image, '/camera/depth/depth/image_raw', 
            self._depth_callback, sensor_qos)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', 
            self._info_callback, sensor_qos)
        
        self.debug_pub = self.create_publisher(Image, '/hanoi/debug_image', 10)
        self.pose_pubs = {
            cfg['name']: self.create_publisher(
                PoseStamped, f"/hanoi/cube_{cfg['name']}_pose", 10)
            for cfg in CUBES.values()
        }
        
        self.create_timer(0.1, self._process)
        
        self.get_logger().info('Vision node started (robust mode)')

    def _info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d) if len(msg.d) > 0 else np.zeros(5)
            self.get_logger().info('Camera intrinsics received')

    def _rgb_callback(self, msg: Image):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'RGB error: {e}')

    def _depth_callback(self, msg: Image):
        try:
            if msg.encoding == '16UC1':
                raw = self.bridge.imgmsg_to_cv2(msg, '16UC1')
                self.depth_image = raw.astype(np.float32) / 1000.0  # mm -> m
            elif msg.encoding == '32FC1':
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f'Depth error: {e}')

    def _process(self):
        if self.rgb_image is None or self.camera_matrix is None:
            return
        
        gray = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2GRAY)
        
        if ARUCO_DETECTOR is not None:
            corners, ids, _ = ARUCO_DETECTOR.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
        
        debug_img = self.rgb_image.copy()
        
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)
            
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in CUBES:
                    continue
                
                cfg = CUBES[marker_id]
                cube_name = cfg['name']
                marker_corners = corners[i]
                
                center_px = marker_corners[0].mean(axis=0)
                
                depth_z = self._get_center_depth(center_px, marker_corners)
                
                if depth_z is None or depth_z < 0.1 or depth_z > 3.0:
                    continue
                
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]
                cx = self.camera_matrix[0, 2]
                cy = self.camera_matrix[1, 2]
                
                x = (center_px[0] - cx) * depth_z / fx
                y = (center_px[1] - cy) * depth_z / fy
                z = depth_z
                
                rot_matrix = self._get_marker_orientation(marker_corners, cfg)
                if rot_matrix is None:
                    continue
                
                offset = cfg['cube_depth'] / 2.0
                marker_z = rot_matrix[:, 2]
                
                tvec_center = np.array([x, y, z]) - marker_z * offset
                
                quat = R.from_matrix(rot_matrix).as_quat()
                
                self._broadcast_cube_tf(cube_name, tvec_center, quat)
                
                self._publish_pose(cube_name, tvec_center, quat)
                
                cv2.circle(debug_img, tuple(center_px.astype(int)), 5, (0, 255, 255), -1)
                cv2.putText(debug_img, f"{cube_name} z={z:.3f}m", 
                           (int(center_px[0]), int(center_px[1]) - 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
        except CvBridgeError:
            pass

    def _get_center_depth(self, center_px, corners):

        if self.depth_image is None:
            return None
        
        h, w = self.depth_image.shape[:2]
        cx, cy = int(center_px[0]), int(center_px[1])
        
        half_size = 2
        x1 = max(0, cx - half_size)
        x2 = min(w, cx + half_size + 1)
        y1 = max(0, cy - half_size)
        y2 = min(h, cy + half_size + 1)
        
        roi = self.depth_image[y1:y2, x1:x2]
        valid = roi[(roi > 0.1) & (roi < 3.0)]
        
        if len(valid) > 0:
            return float(np.median(valid))
        
        corner_depths = []
        for pt in corners[0]:
            px, py = int(pt[0]), int(pt[1])
            if 0 <= px < w and 0 <= py < h:
                d = self.depth_image[py, px]
                if 0.1 < d < 3.0:
                    corner_depths.append(d)
        
        if corner_depths:
            return float(np.median(corner_depths))
        
        return None

    def _get_marker_orientation(self, corners, cfg):
        marker_size = cfg['marker_size']
        half = marker_size / 2.0
        
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)
        
        img_points = corners.reshape(-1, 2).astype(np.float32)
        
        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        
        if not success:
            return None
        
        rot_matrix, _ = cv2.Rodrigues(rvec)
        
        if rot_matrix[2, 2] > 0:
            rot_matrix = rot_matrix @ np.diag([1.0, -1.0, -1.0])
        
        return rot_matrix


    def _broadcast_cube_tf(self, cube_name, tvec, quat):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id = f'cube_{cube_name}'
        
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        self.tf_broadcaster.sendTransform(t)

    def _publish_pose(self, cube_name, tvec, quat):
        pose_cam = PoseStamped()
        pose_cam.header.stamp = self.get_clock().now().to_msg()
        pose_cam.header.frame_id = 'camera_color_optical_frame'
        pose_cam.pose.position.x = float(tvec[0])
        pose_cam.pose.position.y = float(tvec[1])
        pose_cam.pose.position.z = float(tvec[2])
        pose_cam.pose.orientation.x = float(quat[0])
        pose_cam.pose.orientation.y = float(quat[1])
        pose_cam.pose.orientation.z = float(quat[2])
        pose_cam.pose.orientation.w = float(quat[3])
        
        try:
            if self.tf_buffer.can_transform('world', 'camera_color_optical_frame',
                                            rclpy.time.Time(), 
                                            timeout=Duration(seconds=1.0)):
                pose_world = self.tf_buffer.transform(pose_cam, 'world')
                self.pose_pubs[cube_name].publish(pose_world)
            else:
                self.pose_pubs[cube_name].publish(pose_cam)
        except Exception as e:
            self.get_logger().warn(f'TF error: {e}', throttle_duration_sec=5.0)
            self.pose_pubs[cube_name].publish(pose_cam)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
