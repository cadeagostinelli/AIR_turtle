import cv2
import numpy as np

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

    cv2.circle(frame, frame_center, center_circle_radius, (0, 255, 0), 2)
    cv2.circle(frame, frame_center, outer_circle_radius, (255, 0, 0), 2)

    if contours:

        c = max(contours, key=cv2.contourArea)

        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 10:

            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

            cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)

            cv2.putText(frame, "({}, {})".format(int(x), int(y)), (int(x) + 10, int(y)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            norm_x = (x - frame_center[0]) / (outer_circle_radius)
            norm_y = (y - frame_center[1]) / (outer_circle_radius)

            norm_x = max(-1, min(1, norm_x))
            norm_y = max(-1, min(1, norm_y))

            distance = np.sqrt((x - frame_center[0]) ** 2 + (y - frame_center[1]) ** 2)

            if distance <= center_circle_radius:
                output = (0, 0)
            elif distance > outer_circle_radius:
                output = (1 if norm_x > 0 else -1, 1 if norm_y > 0 else -1)
            else:
                output = (norm_x, norm_y)

            print(f"Output: {output}")

    cv2.imshow('Tracker', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
