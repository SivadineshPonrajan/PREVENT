import cv2
import numpy as np

# 1. Read the image
img = cv2.imread("./images/test3.png")

# 2. Select HSV range for red
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower_red = np.array([0, 50, 50])
upper_red = np.array([10, 255, 255])

# 3. Create a mask for the red color
mask = cv2.inRange(hsv, lower_red, upper_red)

# 4. Highlight red areas
highlighted = cv2.bitwise_and(img, img, mask=mask)

# Combine both images side-by-side
combined = np.hstack((img, highlighted))

cv2.imshow("Original (left)  |  Red Highlighted (right)", combined)
cv2.waitKey(0)
cv2.destroyAllWindows()