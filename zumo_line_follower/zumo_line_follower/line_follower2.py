import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import time

class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_left = self.create_subscription(
            Int32,
            'zumo/ir_sensor/left',
            self.listener_callback,
            10)
        self.subscription_right = self.create_subscription(
            Int32,
            'zumo/ir_sensor/right',
            self.listener_callback,
            10)
        self.detected_left = False
        self.detected_right = False

    def listener_callback(self, msg):
        # Here we update the detection status based on the sensor data
        if msg.data == 1:  # Assuming 1 means line detected
            if msg._topic == 'zumo/ir_sensor/left':
                self.detected_left = True
                self.detected_right = False
            elif msg._topic == 'zumo/ir_sensor/right':
                self.detected_right = True
                self.detected_left = False
        else:
            self.detected_left = False
            self.detected_right = False

        # Let's generate command messages
        self.command()

    def command(self):
        msg = Twist()
        if self.detected_left:
            # This means the robot is moving too far to the right, so we turn left
            msg.linear.x = 0.5  # Forward speed
            msg.angular.z = 0.5  # Turn speed (positive values go counter-clockwise)
        elif self.detected_right:
            # This means the robot is moving too far to the left, so we turn right
            msg.linear.x = 0.5  # Forward speed
            msg.angular.z = -0.5  # Turn speed (negative values go clockwise)
        else:
            # If no line is detected, the robot performs a 90 degree turn
            msg.linear.x = 0.0  # Stop moving forward
            msg.angular.z = 0.5  # Start turning
            self.publisher_.publish(msg)
            time.sleep(1.57)  # Wait for approximately 90 degree turn (adjust as needed)
            msg.angular.z = 0.0  # Stop turning
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    line_follower = LineFollower()

    rclpy.spin(line_follower)

    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
