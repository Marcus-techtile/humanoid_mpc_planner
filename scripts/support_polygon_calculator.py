#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scipy.spatial import ConvexHull
import threading
import tf2_ros

class SupportPolygonCalculator:
    def __init__(self):
        rospy.init_node('support_polygon_calculator', anonymous=True)
        
        # Robot parameters (from G1 URDF collision geometry)
        self.foot_length = rospy.get_param('~foot_length', 0.18)  # meters (URDF collision box)
        self.foot_width = rospy.get_param('~foot_width', 0.06)    # meters (URDF collision box)
        self.foot_height = rospy.get_param('~foot_height', 0.016) # meters (URDF collision box)
        
        # Foot offset from ankle_roll_link (from URDF collision origin)
        self.foot_offset = np.array([0.04, 0.0, -0.029])  # x, y, z offset
        
        # Publishing parameters
        self.publish_rate = rospy.get_param('~publish_rate', 50.0)  # Hz
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        
        # TF frame names for ankle frames (from URDF)
        self.left_ankle_frame = "left_ankle_roll_link"
        self.right_ankle_frame = "right_ankle_roll_link"
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Current foot positions
        self.left_foot_pose = PoseStamped()
        self.right_foot_pose = PoseStamped()
        
        # TF timeout
        self.tf_timeout = rospy.Duration(0.1)
        
        # Publishers
        self.support_polygon_pub = rospy.Publisher('/support_polygon', PolygonStamped, queue_size=1)
        self.foot_poses_pub = rospy.Publisher('/foot_poses', MarkerArray, queue_size=1)
        self.left_foot_pub = rospy.Publisher('/left_foot_pose', PoseStamped, queue_size=1)
        self.right_foot_pub = rospy.Publisher('/right_foot_pose', PoseStamped, queue_size=1)
        
        # Publisher timer
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_callback)
        
        rospy.loginfo("Support Polygon Calculator initialized with TF-based approach")
        rospy.loginfo(f"Publishing support polygon at {self.publish_rate} Hz")
        rospy.loginfo(f"G1 foot dimensions from URDF: {self.foot_length}m x {self.foot_width}m x {self.foot_height}m")
        rospy.loginfo(f"Using ankle frames: {self.left_ankle_frame}, {self.right_ankle_frame}")
    
    def get_ankle_transform(self, ankle_frame):
        """Get transform from ankle frame to base_link"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,  # target frame (base_link)
                ankle_frame,    # source frame (ankle)
                rospy.Time(),   # latest available
                self.tf_timeout
            )
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, f"Could not get transform from {ankle_frame} to {self.frame_id}: {e}")
            return None
    
    def transform_point(self, point, transform):
        """Transform a point using a TransformStamped"""
        # Extract translation and rotation from transform
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        
        # Apply rotation (convert quaternion to rotation matrix)
        # Simplified rotation matrix from quaternion
        rotated_point = self.rotate_point_by_quaternion(point, qx, qy, qz, qw)
        
        # Apply translation
        transformed_point = rotated_point + np.array([tx, ty, tz])
        
        return transformed_point
    
    def rotate_point_by_quaternion(self, point, qx, qy, qz, qw):
        """Rotate a point by a quaternion"""
        # Convert quaternion to rotation matrix
        # R = I + 2*Q_skew + 2*Q_skew^2 where Q_skew is skew matrix of (qx,qy,qz)
        # Simplified implementation
        x, y, z = point
        
        # Quaternion rotation formula: v' = v + 2*q_vec x (q_vec x v + q_w*v)
        q_vec = np.array([qx, qy, qz])
        v = np.array([x, y, z])
        
        cross1 = np.cross(q_vec, v) + qw * v
        cross2 = np.cross(q_vec, cross1)
        
        rotated = v + 2 * cross2
        return rotated
    
    def calculate_foot_positions(self):
        """Calculate current foot positions using TF transforms"""
        try:
            # Get transforms for both ankle frames
            left_transform = self.get_ankle_transform(self.left_ankle_frame)
            right_transform = self.get_ankle_transform(self.right_ankle_frame)
            
            if left_transform is None or right_transform is None:
                return False
            
            # Calculate foot center positions (ankle position + foot offset)
            current_time = rospy.Time.now()
            
            # Left foot
            left_foot_center = self.transform_point(self.foot_offset, left_transform)
            
            self.left_foot_pose.header.stamp = current_time
            self.left_foot_pose.header.frame_id = self.frame_id
            self.left_foot_pose.pose.position.x = left_foot_center[0]
            self.left_foot_pose.pose.position.y = left_foot_center[1]
            self.left_foot_pose.pose.position.z = left_foot_center[2]
            self.left_foot_pose.pose.orientation = left_transform.transform.rotation
            
            # Right foot
            right_foot_center = self.transform_point(self.foot_offset, right_transform)
            
            self.right_foot_pose.header.stamp = current_time
            self.right_foot_pose.header.frame_id = self.frame_id
            self.right_foot_pose.pose.position.x = right_foot_center[0]
            self.right_foot_pose.pose.position.y = right_foot_center[1]
            self.right_foot_pose.pose.position.z = right_foot_center[2]
            self.right_foot_pose.pose.orientation = right_transform.transform.rotation
            
            return True
                
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Error calculating foot positions: {e}")
            return False
    
    def get_foot_corners(self, foot_pose):
        """Get the four corner points of a foot given its pose"""
        corners = []
        
        # Foot corners in local frame (assuming foot center at origin)
        local_corners = [
            [-self.foot_length/2, -self.foot_width/2, 0],  # Back-left
            [ self.foot_length/2, -self.foot_width/2, 0],  # Front-left  
            [ self.foot_length/2,  self.foot_width/2, 0],  # Front-right
            [-self.foot_length/2,  self.foot_width/2, 0],  # Back-right
        ]
        
        # Transform corners to global frame
        from tf.transformations import euler_from_quaternion
        
        quat = [
            foot_pose.pose.orientation.x,
            foot_pose.pose.orientation.y, 
            foot_pose.pose.orientation.z,
            foot_pose.pose.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(quat)
        
        # Rotation matrix (simplified for z-rotation only)
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        for corner in local_corners:
            # Rotate and translate
            x_rot = corner[0] * cos_yaw - corner[1] * sin_yaw
            y_rot = corner[0] * sin_yaw + corner[1] * cos_yaw
            
            global_x = foot_pose.pose.position.x + x_rot
            global_y = foot_pose.pose.position.y + y_rot
            global_z = foot_pose.pose.position.z + corner[2]
            
            corners.append([global_x, global_y, global_z])
        
        return corners
    
    def calculate_support_polygon(self):
        """Calculate the convex hull of foot contact points"""
        try:
            # Get foot corner points
            left_corners = self.get_foot_corners(self.left_foot_pose)
            right_corners = self.get_foot_corners(self.right_foot_pose)
            
            # Combine all contact points
            all_points = left_corners + right_corners
            
            if len(all_points) < 3:
                rospy.logwarn_throttle(1.0, "Not enough points for support polygon")
                return None
            
            # Extract x,y coordinates for 2D convex hull
            points_2d = np.array([[p[0], p[1]] for p in all_points])
            
            try:
                # Calculate convex hull
                hull = ConvexHull(points_2d)
                
                # Create polygon message
                polygon = PolygonStamped()
                polygon.header.stamp = rospy.Time.now()
                polygon.header.frame_id = self.frame_id
                
                # Add hull vertices to polygon
                for vertex_idx in hull.vertices:
                    point = Point32()
                    point.x = points_2d[vertex_idx][0]
                    point.y = points_2d[vertex_idx][1]
                    point.z = 0.0  # Support polygon is on ground plane
                    polygon.polygon.points.append(point)
                
                return polygon
                
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"ConvexHull error: {e}")
                # Fallback: create simple rectangle
                return self.create_simple_polygon(all_points)
                
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Error calculating support polygon: {e}")
            return None
    
    def create_simple_polygon(self, points):
        """Create a simple rectangular polygon as fallback"""
        if not points:
            return None
            
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        polygon = PolygonStamped()
        polygon.header.stamp = rospy.Time.now()
        polygon.header.frame_id = self.frame_id
        
        # Create rectangle
        corners = [
            (min_x, min_y), (max_x, min_y),
            (max_x, max_y), (min_x, max_y)
        ]
        
        for corner in corners:
            point = Point32()
            point.x = corner[0]
            point.y = corner[1]
            point.z = 0.0
            polygon.polygon.points.append(point)
        
        return polygon
    
    def create_foot_markers(self):
        """Create visualization markers for foot positions"""
        marker_array = MarkerArray()
        
        # Left foot marker
        left_marker = Marker()
        left_marker.header = self.left_foot_pose.header
        left_marker.ns = "feet"
        left_marker.id = 0
        left_marker.type = Marker.CUBE
        left_marker.action = Marker.ADD
        left_marker.pose = self.left_foot_pose.pose
        left_marker.scale.x = self.foot_length
        left_marker.scale.y = self.foot_width
        left_marker.scale.z = self.foot_height
        left_marker.color = ColorRGBA(0.0, 0.8, 0.0, 0.7)  # Green
        marker_array.markers.append(left_marker)
        
        # Right foot marker
        right_marker = Marker()
        right_marker.header = self.right_foot_pose.header
        right_marker.ns = "feet"
        right_marker.id = 1
        right_marker.type = Marker.CUBE
        right_marker.action = Marker.ADD
        right_marker.pose = self.right_foot_pose.pose
        right_marker.scale.x = self.foot_length
        right_marker.scale.y = self.foot_width
        right_marker.scale.z = self.foot_height
        right_marker.color = ColorRGBA(0.0, 0.0, 0.8, 0.7)  # Blue
        marker_array.markers.append(right_marker)
        
        return marker_array
    
    def publish_callback(self, event):
        """Main publishing callback"""
        # Calculate current foot positions using TF
        if not self.calculate_foot_positions():
            return
        
        # Calculate support polygon
        support_polygon = self.calculate_support_polygon()
        if support_polygon is not None:
            self.support_polygon_pub.publish(support_polygon)
        
        # Publish individual foot poses
        self.left_foot_pub.publish(self.left_foot_pose)
        self.right_foot_pub.publish(self.right_foot_pose)
        
        # Publish foot visualization markers
        foot_markers = self.create_foot_markers()
        self.foot_poses_pub.publish(foot_markers)


def main():
    try:
        calculator = SupportPolygonCalculator()
        rospy.loginfo("Support Polygon Calculator started")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Support Polygon Calculator interrupted")
    except Exception as e:
        rospy.logerr(f"Exception in Support Polygon Calculator: {e}")

if __name__ == '__main__':
    main() 