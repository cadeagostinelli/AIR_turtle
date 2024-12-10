import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            msg_type=Twist,
            topic='/tracker_data',
            callback=self.listener_callback,
            qos_profile=10
        )

    def listener_callback(self, msg):
        x = msg.linear.x
        y = msg.linear.y

        # Adjust velocity based on x and y
        twist = Twist()
        twist.linear.x = y  # Forward/backward speed
        twist.angular.z = -x  # Turning speed

        self.publisher.publish(twist)

def main():
    rclpy.init()
    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
