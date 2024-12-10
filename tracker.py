import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

class BallTracker(Node):
    def __init__(self):
        super().__init__('ball_tracker')
        
        # Publisher for ball position
        self.ball_pub = self.create_publisher(Float32MultiArray, '/ball_position', 10)
        
        # Circle radii
        self.center_circle_radius = 100
        self.outer_circle_radius = 500

        # Video capture
        self.cam = cv2.VideoCapture(0)
        self.frame_width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.frame_center = (self.frame_width // 2, self.frame_height // 2)

        # Timer to process frames periodically
        self.timer = self.create_timer(0.1, self.track_ball)  # 10 Hz

    def track_ball(self):
        ret, frame = self.cam.read()
        frame = cv2.flip(frame, 1)
        if not ret:
            self.get_logger().error('Failed to capture frame.')
            return

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Ball color range
        lower_orange = np.array([10, 100, 80])
        upper_orange = np.array([25, 255, 255])

        # Create mask
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Get the largest contour
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > 10:
                # Calculate normalized position
                norm_x = (x - self.frame_center[0]) / self.outer_circle_radius
                norm_y = (y - self.frame_center[1]) / self.outer_circle_radius
                norm_x = max(-1, min(1, norm_x))
                norm_y = max(-1, min(1, norm_y))
            
                # Calculate distance
                distance = np.sqrt((x - self.frame_center[0])**2 + (y - self.frame_center[1])**2)

                #output to term
                if distance <= self.center_circle_radius:
                    output = (0, 0)
                elif distance > self.outer_circle_radius:
                    output = (1 if norm_x > 0 else -1, 1 if norm_y > 0 else -1)
                else:
                    output = (norm_x, norm_y)
                print(f"Output: {output}")
                
                # Publish position
                msg = Float32MultiArray()
                msg.data = [float(norm_x), float(norm_y), float(distance)]  # Ensure floats
                self.ball_pub.publish(msg)

        # Draw the circles on the frame
        cv2.circle(frame, self.frame_center, self.center_circle_radius, (0, 255, 0), 2)  # Center circle in green
        cv2.circle(frame, self.frame_center, self.outer_circle_radius, (0, 0, 255), 2)   # Outer circle in red

        # Display the frame
        cv2.imshow("Ball Tracking", frame)

        # Exit on 'q' key
        if cv2.waitKey(1) == ord('q'):
            self.cam.release()
            cv2.destroyAllWindows()



    def destroy_node(self):
        self.cam.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    tracker = BallTracker()
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
