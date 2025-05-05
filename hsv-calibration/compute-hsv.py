import cv2
import numpy as np
import os
import glob

# Get image path
pwd = os.path.dirname(__file__)
image_folder = os.path.join(pwd,"preprocessed_data")
images_path = glob.glob(os.path.join(image_folder, "*.png"))

#List to hold hsv values
h_mins, s_mins, v_mins = [],[],[]
h_maxs, s_maxs, v_maxs = [],[],[]

for path in images_path:
    image = cv2.imread(path)
    if image is None:
        print(f"Failed to read image {path}")
        continue

    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Compute the HSV range in that region    
    h_min, s_min, v_min = np.min(image_hsv.reshape(-1, 3), axis=0)
    h_max, s_max, v_max = np.max(image_hsv.reshape(-1, 3), axis=0)

    h_mins.append(h_min)
    s_mins.append(s_min)
    v_mins.append(v_min)

    h_maxs.append(h_max)
    s_maxs.append(s_max)
    v_maxs.append(v_max)

    print(f"\nImage: {os.path.basename(path)}")
    print(f"   Lower HSV bound: ({h_min}, {s_min}, {v_min})")
    print(f"   Upper HSV bound: ({h_max}, {s_max}, {v_max})")



# Compute overall HSV range
overall_h_min = min(h_mins)
overall_s_min = min(s_mins)
overall_v_min = min(v_mins)

overall_h_max = max(h_maxs)
overall_s_max = max(s_maxs)
overall_v_max = max(v_maxs)


print("\n===============================")
print("📌 Final HSV Range to Detect All Handles")
print(f"🔻 Lower Bound: ({overall_h_min}, {overall_s_min}, {overall_v_min})")
print(f"🔺 Upper Bound: ({overall_h_max}, {overall_s_max}, {overall_v_max})")
print("===============================\n")
print("Lower HSV bound:", (h_min, s_min, v_min))
print("Upper HSV bound:", (h_max, s_max, v_max))

'''#create a mask and display the region that matches the color
lower_bound = np.array([h_min, s_min, v_min])
upper_bound = np.array([h_max, s_max, v_max])

mask = cv2.inRange(image_hsv, lower_bound, upper_bound)
result = cv2.bitwise_and(image, image, mask=mask)

cv2.imshow("Original", image)
cv2.imshow("Mask", mask)
cv2.imshow("Detected Handle Region", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''