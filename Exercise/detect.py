
import cv2
import numpy as np
import sys

def detect_arrow_and_show(image_path: str):
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not read {image_path}")
        return
    image_bgr = image.copy()
    hsv_image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    red_lower = np.array([0, 50, 50])
    red_upper = np.array([10, 255, 255])
    green_lower = np.array([35, 50, 50])
    green_upper = np.array([85, 255, 255])
    blue_lower = np.array([100, 50, 50])
    blue_upper = np.array([130, 255, 255])
    orange_lower = np.array([ 8,100,140])
    orange_upper = np.array([20,255,255])
    yellow_lower = np.array([20, 60,160])
    yellow_upper = np.array([45,255,255])

    red_mask = cv2.inRange(hsv_image, red_lower, red_upper)
    green_mask = cv2.inRange(hsv_image, green_lower, green_upper)
    blue_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
    orange_mask = cv2.inRange(hsv_image, orange_lower, orange_upper)
    yellow_mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)

    red_pixels = cv2.countNonZero(red_mask)
    green_pixels = cv2.countNonZero(green_mask)
    blue_pixels = cv2.countNonZero(blue_mask)
    flame_pixels = cv2.countNonZero(red_mask) + cv2.countNonZero(orange_mask) + cv2.countNonZero(yellow_mask)

    color_counts = {
        'Red': red_pixels,
        'Green': green_pixels,
        'Blue': blue_pixels,
        'Flame': flame_pixels
    }

    max_color = max(color_counts, key=color_counts.get)
    max_count = color_counts[max_color]
    detected_color = max_color if max_count > 100 else "Unknown"
    # return detected_color

    cv2.putText(image, f'Detected: {detected_color}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow(f'{image_path} - {detected_color}', image)
    print(f"{image_path}: {detected_color} detected")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python detect.py <filename>")
    else:
        detect_arrow_and_show(sys.argv[1])