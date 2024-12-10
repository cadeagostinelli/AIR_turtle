import cv2
import numpy as np
import socket


def track_ping_pong_ball():
    #socket for data to controller
    server_address = ('localhost', 65432)  #match with controller.py
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


    cam = cv2.VideoCapture(0)

    center_circle_radius = 100
    outer_circle_radius = 500

    
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_center = (frame_width // 2, frame_height // 2)

    while True:
        ret, frame = cam.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_orange = np.array([10, 100, 20])
        upper_orange = np.array([25, 255, 255])

        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > 10:
                
                distance = np.sqrt((x - frame_center[0]) ** 2 + (y - frame_center[1]) ** 2)

                if distance <= center_circle_radius:
                    direction = "CENTER"
                elif x < frame_center[0] - center_circle_radius:
                    direction = "LEFT"
                elif x > frame_center[0] + center_circle_radius:
                    direction = "RIGHT"
                elif y < frame_center[1] - center_circle_radius:
                    direction = "UP"
                elif y > frame_center[1] + center_circle_radius:
                    direction = "DOWN"
                else:
                    direction = "UNKNOWN"

                #directions sending to controller
                sock.sendto(direction.encode(), server_address)

        if cv2.waitKey(1) == ord('q'):
            break

    cam.release()
    cv2.destroyAllWindows()
    sock.close()

track_ping_pong_ball()

if __name__ == "__main__":
    track_ping_pong_ball()
