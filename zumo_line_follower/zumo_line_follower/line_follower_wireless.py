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
        self.ser = serial.Serial('/dev/98:D3:31:FB:3D:63', 9600)  # Using Bluetooth port here
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
                # If no line is detected, the robot performs a line recovery operation
                if not self.recover_line():
                    # if the line was not recovered, stop the robot
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    break
            self.publisher_.publish(msg)
        self.ser.close()

    def recover_line(self):
        recovery_time = 12  # max time for recovery in seconds
        start_time = time.time()  
        dist_covered = 0.1  # initial distance to cover forward after turn
        increment_dist = 0.1  # increment distance after every turn
        max_dist = 0.5  # maximum distance to go forward

        while time.time() - start_time < recovery_time:
            # read line from IR sensors from Arduino
            line = self.ser.readline().decode('utf-8').strip()
            left, right = map(int, line.split())
            if left or right:
                # Line found, return from function
                return True
            else:
                # Perform recovery operation
                # Start by turning 90 degrees
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.5
                self.publisher_.publish(msg)
                time.sleep(1)  # Adjust as needed for a 90 degree turn
                # Then go forward a certain distance
                msg.angular.z = 0.0
                msg.linear.x = 0.5
                self.publisher_.publish(msg)
                time.sleep(dist_covered)  # Sleep according to the distance to be covered
                # Increase the distance for the next cycle
                dist_covered = min(dist_covered + increment_dist, max_dist)
        # If we reach here, line was not recovered within the given time
        return False

def main(args=None):
    rclpy.init(args=args)

    line_follower = LineFollower()

    rclpy.spin(line_follower)

    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
