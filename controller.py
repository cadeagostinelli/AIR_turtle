import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 65432))  #match with tracker socket
        self.timer = self.create_timer(0.1, self.listen_for_direction)

    def listen_for_direction(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            decoded_data = data.decode()

            # Assuming data format "DIRECTION x y"
            parts = decoded_data.split()
            if len(parts) >= 3:
                direction, x, y = parts[0], float(parts[1]), float(parts[2])
                self.get_logger().info(f"Received Direction: {direction}, X: {x}, Y: {y}")
            else:
                direction = decoded_data
                x = y = None
                self.get_logger().warn(f"Received Direction without coordinates: {direction}")

            #twist controls
            twist = Twist()

            if direction == "LEFT":
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            elif direction == "RIGHT":
                twist.linear.x = 0.0
                twist.angular.z = -0.5
            elif direction == "UP":
                twist.linear.x = 0.2  
                twist.angular.z = 0.0
            elif direction == "DOWN": 
                twist.linear.x = -0.2  
                twist.angular.z = 0.0
            elif direction == "CENTER":
                twist.linear.x = 0.0  
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0


            self.publisher.publish(twist)
        except socket.timeout:
            self.get_logger().warn("No direction data received. Stopping robot.")
            twist = Twist()  #stop
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
