import cv2
import numpy as np
import os

# Get image path
pwd = os.path.dirname(__file__)
rel_path = "res/captured_image.png_20250505-115248.png"
image_path = os.path.join(pwd,rel_path)

#load image
image = cv2.imread(image_path)  
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define ROI (Region of Interest) around the handle
# You must manually tune these values to enclose the yellow handle
x, y, w, h = 640, 320, 60, 60  # Example values; adjust to your image

roi = image_hsv[y:y+h, x:x+w]

# Compute the HSV range in that region
h_min, s_min, v_min = np.min(roi.reshape(-1, 3), axis=0)
h_max, s_max, v_max = np.max(roi.reshape(-1, 3), axis=0)

print("Lower HSV bound:", (h_min, s_min, v_min))
print("Upper HSV bound:", (h_max, s_max, v_max))

# Optional: create a mask and display the region that matches the color
lower_bound = np.array([h_min, s_min, v_min])
upper_bound = np.array([h_max, s_max, v_max])

mask = cv2.inRange(image_hsv, lower_bound, upper_bound)
result = cv2.bitwise_and(image, image, mask=mask)

cv2.imshow("Original", image)
cv2.imshow("Mask", mask)
cv2.imshow("Detected Handle Region", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
