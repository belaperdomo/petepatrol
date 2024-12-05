import rclpy
from rclpy.node import Node

from math import modf

from std_msgs.msg import Header
from custom_interfaces.msg import Controllermsg

import RPi.GPIO as GPIO
from time import sleep
from geometry_msgs.msg import Twist

import sys
sys.path.append('/home/volk/Desktop/robotics/Mee5650_ros2_ws/src/pete_patrol/pete_patrol')
import ps5controller as controller

STBY = 2 #H-bridge enabler (must be High)
AIn1 = 5
AIn2 = 6
BIn1 = 14
BIn2 = 15
PWMA = 12
PWMB = 13

GPIO.setmode(GPIO.BCM)

# front marked by sharpie, pins oriented facing robot front
STBY = 2 #H-bridge enabler (must be High) connected to both drivers

frontLeft_AIn1 = 3
frontLeft_AIn2 = 4
frontRight_BIn1 = 17 #direction might need changed
frontRight_BIn2 = 27

frontLeft_PWM = 13
frontRight_PWM = 12

backLeft_AIn1 = 22
backLeft_AIn2 = 10
backRight_BIn1 = 9
backRight_BIn2 = 11

backLeft_PWM = 19
backRight_PWM = 16

FL_encoder = 5
FR_encoder = 6
BL_encoder = 20
BR_encoder = 21

FL_tick = 0
FR_tick = 0
BL_tick = 0
BR_tick = 0

GPIO.setup(STBY,GPIO.OUT)
GPIO.setup(frontLeft_AIn1,GPIO.OUT)
GPIO.setup(frontLeft_AIn2,GPIO.OUT)
GPIO.setup(frontRight_BIn1,GPIO.OUT)
GPIO.setup(frontRight_BIn2,GPIO.OUT)
GPIO.setup(frontLeft_PWM,GPIO.OUT)
GPIO.setup(frontRight_PWM,GPIO.OUT)

GPIO.setup(backLeft_AIn1,GPIO.OUT)
GPIO.setup(backLeft_AIn2,GPIO.OUT)
GPIO.setup(backRight_BIn1,GPIO.OUT)
GPIO.setup(backRight_BIn2,GPIO.OUT)
GPIO.setup(backLeft_PWM,GPIO.OUT)
GPIO.setup(backRight_PWM,GPIO.OUT)

# GPIO.setup(FL_encoder,GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.setup(FR_encoder,GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.setup(BL_encoder,GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.setup(BR_encoder,GPIO.IN, pull_up_down=GPIO.PUD_UP)


GPIO.output(STBY,GPIO.HIGH)
GPIO.output(frontLeft_AIn1,GPIO.LOW)
GPIO.output(frontLeft_AIn2,GPIO.LOW)
GPIO.output(frontRight_BIn1,GPIO.LOW)
GPIO.output(frontRight_BIn2,GPIO.LOW)

GPIO.output(backLeft_AIn1,GPIO.LOW)
GPIO.output(backLeft_AIn2,GPIO.LOW)
GPIO.output(backRight_BIn1,GPIO.LOW)
GPIO.output(backRight_BIn2,GPIO.LOW)

FL_pwm=GPIO.PWM(frontLeft_PWM,1000)
FR_pwm=GPIO.PWM(frontRight_PWM,1000)

BL_pwm = GPIO.PWM(backLeft_PWM,1000) #1000Hz
BR_pwm = GPIO.PWM(backRight_PWM,1000)

class motorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription = self.create_subscription(Controllermsg, 'controller',self.drive_control, 10)
        #self.subscription
        FL_pwm.start(100) #100% duty cycle
        FR_pwm.start(100)
        BL_pwm.start(100)
        BR_pwm.start(100)

        self.cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.aruco_callback, 10)
        self.mode = 'manual' # start in manual mode
        self.prev_left_speed = 0.0
        self.prev_right_speed = 0.0

    def forward(self, speed=100):
        FL_pwm.start(speed)
        FR_pwm.start(speed)
        BL_pwm.start(speed)
        BR_pwm.start(speed)
        
        GPIO.output(frontLeft_AIn1,GPIO.LOW) 
        GPIO.output(frontLeft_AIn2,GPIO.HIGH) 

        GPIO.output(frontRight_BIn1,GPIO.HIGH) 
        GPIO.output(frontRight_BIn2,GPIO.LOW) 

        GPIO.output(backLeft_AIn1,GPIO.LOW) 
        GPIO.output(backLeft_AIn2,GPIO.HIGH) 

        GPIO.output(backRight_BIn1,GPIO.HIGH) 
        GPIO.output(backRight_BIn2,GPIO.LOW) 

    def backward(self, speed=100):
        FL_pwm.start(speed)
        FR_pwm.start(speed)
        BL_pwm.start(speed)
        BR_pwm.start(speed)
        
        GPIO.output(frontLeft_AIn1,GPIO.HIGH) 
        GPIO.output(frontLeft_AIn2,GPIO.LOW) 

        GPIO.output(frontRight_BIn1,GPIO.LOW) 
        GPIO.output(frontRight_BIn2,GPIO.HIGH) 

        GPIO.output(backLeft_AIn1,GPIO.HIGH) 
        GPIO.output(backLeft_AIn2,GPIO.LOW) 

        GPIO.output(backRight_BIn1,GPIO.LOW) 
        GPIO.output(backRight_BIn2,GPIO.HIGH) 

    def turnLeft(self, speed=100):
        FL_pwm.start(speed)
        FR_pwm.start(speed)
        BL_pwm.start(speed)
        BR_pwm.start(speed)

        GPIO.output(frontLeft_AIn1,GPIO.LOW) 
        GPIO.output(frontLeft_AIn2,GPIO.HIGH) 

        GPIO.output(frontRight_BIn1,GPIO.LOW) 
        GPIO.output(frontRight_BIn2,GPIO.HIGH) 

        GPIO.output(backLeft_AIn1,GPIO.LOW) 
        GPIO.output(backLeft_AIn2,GPIO.HIGH) 

        GPIO.output(backRight_BIn1,GPIO.LOW) 
        GPIO.output(backRight_BIn2,GPIO.HIGH)

    def turnRight(self, speed=100):
        FL_pwm.start(speed)
        FR_pwm.start(speed)
        BL_pwm.start(speed)
        BR_pwm.start(speed)

        GPIO.output(frontLeft_AIn1,GPIO.HIGH) 
        GPIO.output(frontLeft_AIn2,GPIO.LOW) 

        GPIO.output(frontRight_BIn1,GPIO.HIGH) 
        GPIO.output(frontRight_BIn2,GPIO.LOW) 

        GPIO.output(backLeft_AIn1,GPIO.HIGH) 
        GPIO.output(backLeft_AIn2,GPIO.LOW) 

        GPIO.output(backRight_BIn1,GPIO.HIGH) 
        GPIO.output(backRight_BIn2,GPIO.LOW)  
        

    def motorsOff(self):
        GPIO.output(frontLeft_AIn1,GPIO.LOW)
        GPIO.output(frontLeft_AIn2,GPIO.LOW)
        GPIO.output(frontRight_BIn1,GPIO.LOW)
        GPIO.output(frontRight_BIn2,GPIO.LOW)

        GPIO.output(backLeft_AIn1,GPIO.LOW)
        GPIO.output(backLeft_AIn2,GPIO.LOW)
        GPIO.output(backRight_BIn1,GPIO.LOW)
        GPIO.output(backRight_BIn2,GPIO.LOW)

    def drive_control(self, msg):
        if msg.button == 'triangle' and msg.button_direction == 'down':
            GPIO.cleanup()
            FL_pwm.stop() 
            FR_pwm.stop()
            BL_pwm.stop()
            BR_pwm.stop()
            print('GPIO cleared')
        elif msg.button == 'square':
            if self.mode == 'manual':
                self.mode = 'follow'
                self.get_logger().info('Switched to user-following mode')
        elif msg.button == 'x':
            if self.mode == 'follow':
                self.mode = 'manual'
                self.get_logger().info('Switched to manual mode')
            #return

        if self.mode == 'manual':
            if msg.left_joystick[0] > 0:
                self.forward()
            elif msg.left_joystick[0] < 0:
                self.backward()
            elif msg.l2 != 0:
                self.turnLeft()
            elif msg.r2 != 0:
                self.turnRight()
            else:
                self.motorsOff()
        else:
            pass
    
#    def apply_min_speed_threshold(self, speed, min_speed):
#        if abs(speed) > 0 and abs(speed) < min_speed:
#            return min_speed * (speed / abs(speed))  # Retain the sign
#        else:
#            return speed

    def smooth_speed(self, target_speed, prev_speed, max_change):
        if target_speed > prev_speed + max_change:
            return prev_speed + max_change
        elif target_speed < prev_speed - max_change:
            return prev_speed - max_change
        else:
            return target_speed


    def aruco_callback(self, twist_msg):
        if self.mode == 'follow':
            linear_x = twist_msg.linear.x
            angular_z = twist_msg.angular.z

            # Log received values
            #self.get_logger().info(f"Received linear_x: {linear_x}, angular_z: {angular_z}")

            # Map linear_x and angular_z to motor speeds
            base_speed = linear_x * 175  # Set base speed to 35%

            # Apply min_speed threshold to base_speed
            # min_speed = 30
            # if base_speed != 0:
            #    base_speed = self.apply_min_speed_threshold(base_speed, min_speed)

            turn_adjust = angular_z * 65  # Scale angular adjustment
            # Limit turn_adjust 
            max_turn_adjust = 30
            turn_adjust = max(-max_turn_adjust, min(turn_adjust, max_turn_adjust))

            # Compute preliminary speeds
            left_speed = base_speed + turn_adjust
            right_speed = base_speed - turn_adjust

            # Log calculated speeds before threshold and smoothing
            #self.get_logger().info(f"Calculated left_speed: {left_speed}, right_speed: {right_speed}")

            # Smooth speed transitions
            max_speed_change = 10.0  # Adjust as necessary
            left_speed = self.smooth_speed(left_speed, self.prev_left_speed, max_speed_change)
            right_speed = self.smooth_speed(right_speed, self.prev_right_speed, max_speed_change)

            # Update previous speeds
            self.prev_left_speed = left_speed
            self.prev_right_speed = right_speed

            # Log the calculated speeds
            #self.get_logger().info(f"Final Left Speed: {left_speed}, Right Speed: {right_speed}")

            # Set motor speeds accordingly
            if base_speed != 0 or turn_adjust != 0:
                self.set_motor_speeds(left_speed, right_speed)
            else:
                self.motorsOff()
        else:
            # Ignore ArUco commands in manual mode
            pass

    def set_motor_speeds(self, left_speed, right_speed):

        # Limit speeds to -100 to 100
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)

        # Determine direction and adjust speed values
        if left_speed >= 0:
            # Forward direction for left motors
            GPIO.output(frontLeft_AIn1, GPIO.LOW)
            GPIO.output(frontLeft_AIn2, GPIO.HIGH)
            GPIO.output(backLeft_AIn1, GPIO.LOW)
            GPIO.output(backLeft_AIn2, GPIO.HIGH)
        else:
            # Reverse direction for left motors
            GPIO.output(frontLeft_AIn1, GPIO.HIGH)
            GPIO.output(frontLeft_AIn2, GPIO.LOW)
            GPIO.output(backLeft_AIn1, GPIO.HIGH)
            GPIO.output(backLeft_AIn2, GPIO.LOW)
            left_speed = -left_speed  # Make speed positive for PWM

        if right_speed >= 0:
            # Forward direction for right motors
            GPIO.output(frontRight_BIn1, GPIO.HIGH)
            GPIO.output(frontRight_BIn2, GPIO.LOW)
            GPIO.output(backRight_BIn1, GPIO.HIGH)
            GPIO.output(backRight_BIn2, GPIO.LOW)
        else:
            # Reverse direction for right motors
            GPIO.output(frontRight_BIn1, GPIO.LOW)
            GPIO.output(frontRight_BIn2, GPIO.HIGH)
            GPIO.output(backRight_BIn1, GPIO.LOW)
            GPIO.output(backRight_BIn2, GPIO.HIGH)
            right_speed = -right_speed  # Make speed positive for PWM

        # Make speeds positive for PWM
        left_speed = abs(left_speed)
        right_speed = abs(right_speed)

        # Ensure speeds are within 0-100%
        left_speed = min(max(left_speed, 0), 100)
        right_speed = min(max(right_speed, 0), 100)

        # Set PWM duty cycles
        FL_pwm.ChangeDutyCycle(left_speed)
        BL_pwm.ChangeDutyCycle(left_speed)
        FR_pwm.ChangeDutyCycle(right_speed)
        BR_pwm.ChangeDutyCycle(right_speed)


def main(args=None):
    rclpy.init(args=args)

    motor_ros2 = motorNode()
    rclpy.spin(motor_ros2)

    # # Destroy the node explicitly
    motor_ros2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
