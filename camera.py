import cv2

class Camera:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)  # Open the default camera (webcam)

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def release(self):
        self.cap.release()

    def __del__(self):
        self.release() 


def test_camera():
    # Create an instance of the Camera class
    cam = Camera()
    
    print("Press 'q' to quit the test.")

    while True:
        # Get a frame
        frame = cam.get_frame()
        
        # If no frame is returned, print an error and break
        if frame is None:
            print("Failed to capture frame.")
            break
        
        # Display the frame
        cv2.imshow("Webcam Test", frame)
        
        # Wait for 'q' key to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cam.release()
    cv2.destroyAllWindows()
    print("Camera test completed.")

if __name__ == "__main__":
    test_camera()
