import cv2
 
# Load the image
image = cv2.imread("./images/test4.png")
 
# Display the image in a window
cv2.imshow("Show Image", image)
 
# Wait for a key press before closing the window
cv2.waitKey(0)
cv2.destroyAllWindows()