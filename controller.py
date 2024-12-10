import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/ball_position',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def listener_callback(self, msg):
        twist = Twist()
        x, y, distance = msg.data

        # Map normalized ball position to turtle movement
        twist.linear.x = distance / 700.0  # Scale linear velocity
        twist.angular.z = -x * 2.0  # Scale angular velocity
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
