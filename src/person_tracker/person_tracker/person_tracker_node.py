from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import subprocess
import json
import os
import select
import threading


class PersonTracker(Node):
    """ROS 2 node for tracking and following a person using YOLO inference."""

    # Configuration constants
    SUBPROCESS_DIR = "/home/volk/Desktop/robotics/Mee5650_ros2_ws/inference"
    LINEAR_SPEED = 0.20
    ANGULAR_GAIN = 0.0025
    MAX_ANGULAR_SPEED = 0.5
    POSITION_DEADBAND = 10  # pixels

    def __init__(self):
        """Initialize the PersonTracker node with subprocess and publishers."""
        super().__init__('person_tracker')
        
        self.process = self._start_inference_subprocess()
        self._start_stderr_monitoring()
        
        # ROS 2 Publishers and Timers
        self.timer = self.create_timer(0.01, self.read_subprocess_output)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def _start_inference_subprocess(self):
        """
        Start the YOLO inference subprocess using Poetry.
        
        Returns:
            subprocess.Popen: The started subprocess
        """
        return subprocess.Popen(
            [
                'poetry', 'run', 'python', 
                os.path.join(self.SUBPROCESS_DIR, 'inference.py')
            ],
            cwd=self.SUBPROCESS_DIR,
            stdin=subprocess.PIPE, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        )

    def _start_stderr_monitoring(self):
        """Start a thread to continuously read and log subprocess stderr."""
        stderr_thread = threading.Thread(target=self._log_subprocess_stderr, daemon=True)
        stderr_thread.start()

    def _log_subprocess_stderr(self):
        """Continuously read and log stderr from the subprocess."""
        while True:
            try:
                stderr_line = self.process.stderr.readline()
                if not stderr_line:
                    break
                self.get_logger().error(f'Subprocess stderr: {stderr_line.strip()}')
            except Exception as e:
                self.get_logger().error(f'Error reading subprocess stderr: {e}')
                break

    def read_subprocess_output(self):
        """
        Read and process output from the inference subprocess.
        Handles person detection and robot control.
        """
        readable, _, _ = select.select([self.process.stdout], [], [], 1.0)

        if readable:
            try:
                message = self.process.stdout.readline().strip()
                if message:
                    detection_data = json.loads(message)
                    self._process_detection(detection_data)
            
            except json.JSONDecodeError:
                self.get_logger().warn(f"Invalid JSON received: {message}")
        
        self._check_subprocess_status()

    def _process_detection(self, detection_data):
        """
        Process person detection data and control the robot.
        
        Args:
            detection_data (dict): Detection information from the subprocess
        """
        self.get_logger().info(f"Received coords: {detection_data}")
        
        present = detection_data.get("present", False)
        x_center = detection_data.get("x") if present else 0
        
        self._control_robot(present, x_center)

    def _control_robot(self, person_present, error_x):
        """
        Generate and publish robot movement commands.
        
        Args:
            person_present (bool): Whether a person is detected
            error_x (float): Horizontal error from image center
        """
        twist = Twist()

        if person_present:
            # Control linear speed
            twist.linear.x = self.LINEAR_SPEED

            # Control angular speed
            if abs(error_x) >= self.POSITION_DEADBAND:
                twist.angular.z = -error_x * self.ANGULAR_GAIN
                twist.angular.z = max(
                    min(twist.angular.z, self.MAX_ANGULAR_SPEED), 
                    -self.MAX_ANGULAR_SPEED
                )
        
        else:
            twist.linear.x = 0
            twist.angular.z = 0

        # Publish movement command
        self.cmd_vel_publisher.publish(twist)

    def _check_subprocess_status(self):
        """Check if the subprocess is still running, exit if it has terminated."""
        if self.process.poll() is not None:
            self.get_logger().error("Subprocess has terminated unexpectedly.")
            super().destroy_node()
            exit(1)

    def destroy_node(self):
        """Ensure subprocess is terminated when the node is destroyed."""
        if self.process and self.process.poll() is None:
            self.process.terminate()
            self.process.wait()
        super().destroy_node()


def main(args=None):
    """Main entry point for the person tracking node."""
    rclpy.init(args=args)
    node = PersonTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()