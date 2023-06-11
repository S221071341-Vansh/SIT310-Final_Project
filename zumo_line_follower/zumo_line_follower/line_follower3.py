import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import serial
import time

class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.srv = self.create_service(SetBool, 'stop_line_following', self.stop_cb)
        self.stop = False

        # Setup serial connection
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        time.sleep(2)  # Wait for the serial connection to initialize

    def stop_cb(self, request, response):
        self.stop = request.data
        response.success = True
        response.message = 'Successfully updated stop status'
        return response

    def command(self):
        while not self.stop:
            # read line from IR sensors from Arduino
            line = self.ser.readline().decode('utf-8').strip()
            left, right = map(int, line.split())

            msg = Twist()
            if left:
                # This means the robot is moving too far to the right, so we turn left
                msg.linear.x = 0.5  # Forward speed
                msg.angular.z = 0.5  # Turn speed (positive values go counter-clockwise)
            elif right:
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
        self.ser.close()

def main(args=None):
    rclpy.init(args=args)

    line_follower = LineFollower()

    rclpy.spin(line_follower)

    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
