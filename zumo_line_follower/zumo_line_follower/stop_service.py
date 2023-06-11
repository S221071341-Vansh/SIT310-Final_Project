import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class StopService(Node):

    def __init__(self):
        super().__init__('stop_service')
        self.srv = self.create_service(SetBool, 'stop_zumo', self.stop_zumo_callback)

    def stop_zumo_callback(self, request, response):
        response.success = True
        if request.data: 
            # Stop the Zumo by publishing zero velocity
            pub = self.create_publisher(Twist, 'cmd_vel', 10)
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            pub.publish(msg)
            response.message = "Zumo has been stopped."
        else:
            response.message = "Zumo continues operation."

        return response


def main(args=None):
    rclpy.init(args=args)

    stop_service = StopService()

    rclpy.spin(stop_service)

    stop_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
