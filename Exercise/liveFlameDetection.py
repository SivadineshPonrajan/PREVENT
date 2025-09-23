import cv2
import numpy as np

def flamedetector():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    # Optional: make the image a bit sharper / consistent
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Parameters you can tune
    MIN_AREA_RATIO = 0.001   # 0.1% of frame area
    SAT_MIN = 140            # min saturation (reduce skin detections)
    VAL_MIN = 190            # min brightness (flames are bright)
    COOLDOWN_FRAMES = 8      # keep text on a few frames to avoid flicker

    cooldown = 0
    print("Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1) 
        if not ret:
            print("Error: Could not read frame.")
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # Hue ranges for red/orange/yellow flames
        # red wrap-around (0-10)
        mask_r1 = cv2.inRange(hsv, np.array([0,   SAT_MIN, VAL_MIN]),
                                   np.array([10,  255,     255]))
        # orange/yellow (15-45) – adjust upper bound if your flame looks more yellow
        mask_r2 = cv2.inRange(hsv, np.array([15,  SAT_MIN, VAL_MIN]),
                                   np.array([45,  255,     255]))

        mask = cv2.bitwise_or(mask_r1, mask_r2)

        # Clean up noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((3,3), np.uint8), iterations=2)

        # Find large enough regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_area = frame.shape[0] * frame.shape[1]
        min_area = max(200, int(MIN_AREA_RATIO * frame_area))  # never below 200 px

        detected = any(cv2.contourArea(c) > min_area for c in contours)

        # Debounce flicker
        if detected:
            cooldown = COOLDOWN_FRAMES
        elif cooldown > 0:
            cooldown -= 1

        if cooldown > 0:
            cv2.putText(frame, 'Flame Detected!', (12, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3, cv2.LINE_AA)

        cv2.imshow('Flame Detection (color)', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    flamedetector()
