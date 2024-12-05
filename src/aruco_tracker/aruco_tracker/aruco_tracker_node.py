import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import math

class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')

        # Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()

        # Initialize ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Video capture
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Unable to access the camera")

        # ROS 2 Timer to capture frames periodically
        self.timer = self.create_timer(0.1, self.capture_frame)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Calibration constant for distance estimation
        self.A = 425.73  # Replace with your calculated A value

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return

        # Detect ArUco markers in the frame
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, _ = detector.detectMarkers(frame)

        if ids is not None:
            ids = ids.flatten()
            for marker_corners, marker_id in zip(corners, ids):
                # Draw bounding box
                self.draw_marker_bounding_box(frame, marker_corners, marker_id)

                # Calculate error between marker center and frame center
                corners = marker_corners.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners
                center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                frame_center_x = int(frame.shape[1] / 2)
                error_x = center_x - frame_center_x

                # Calculate marker area
                area = self.calculate_marker_area(marker_corners)
                self.get_logger().info(f"Estimated area: {area:.2f} pixels")
                # Control the robot
                self.control_robot(error_x, area)
        else:
            # Stop the robot if no marker is detected
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)

    def draw_marker_bounding_box(self, frame, marker_corners, marker_id):
        corners = marker_corners.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners

        # Convert coordinates to integers
        top_left = (int(top_left[0]), int(top_left[1]))
        top_right = (int(top_right[0]), int(top_right[1]))
        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))

        # Draw the bounding box and center
        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

        # Center point
        center_x = int((top_left[0] + bottom_right[0]) / 2.0)
        center_y = int((top_left[1] + bottom_right[1]) / 2.0)
        cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)

        # Display marker ID
        cv2.putText(frame, str(marker_id), (top_left[0], top_left[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def calculate_marker_area(self, marker_corners):
        # Calculate the area of the marker in the image
        corners = marker_corners.reshape((4, 2))
        area = cv2.contourArea(corners)
        return area

    def control_robot(self, error_x, area):
        twist = Twist()
        gain = 0.0025  # Adjust the gain as necessary for smoother turns

        # Estimate the distance
        distance = self.estimate_distance(area)
        self.get_logger().info(f"Estimated Distance: {distance:.2f} meters")

        # Desired stopping distance (e.g., 0.5 meters)
        desired_distance = 0.05
        distance_error = distance - desired_distance

        # Control linear speed
        if distance_error > 0:
            # Proportional control for forward speed
            Kp_distance = 0.5  # Adjust gain as necessary
            twist.linear.x = min(Kp_distance * distance_error, 0.20)
            twist.linear.x = max(twist.linear.x, 0.0)
        else:
            # Stop the robot
            twist.linear.x = 0.0

        # Control angular speed
        deadband = 10  # pixels
        if abs(error_x) < deadband:
            twist.angular.z = 0.0
        else:
            twist.angular.z = -error_x * gain

        # Limit angular speed
        max_angular_speed = 0.5
        twist.angular.z = max(min(twist.angular.z, max_angular_speed), -max_angular_speed)

        # Publish the Twist message
        self.cmd_vel_publisher.publish(twist)

    def estimate_distance(self, area):
        # Use the calibrated relationship
        A = self.A  # Calibration constant
        distance = math.sqrt(A / area)
        return distance

    def destroy_node(self):
        cv2.destroyAllWindows()
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
