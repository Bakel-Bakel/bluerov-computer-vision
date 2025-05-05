# 🔍 HSV Calibration for ROV Tool Detection

## 🧠 About the Project

During one of my practical sessions at the towing tank at Universidade da Coruña — I explored a unique challenge: enabling an ROV to detect and interact with a **yellow-colored tool** underwater (but the code can always be modified to suit any color).


While detecting yellow in HSV color space seems trivial in theory (usually around `[[25, 50, 70], [35, 255, 255]]`), it completely breaks down in low or variable light. The towing tank made that abundantly clear.

So I coded a **reverse-engineered HSV calibration tool** that automatically determines the best HSV range based on images captured in different lighting and viewing conditions.

---

## 📌 What the Code Does

This Python script:

1. Loads preprocessed images of the yellow tool in different positions.
2. Converts each image from BGR to HSV color space using OpenCV.
3. Extracts the **minimum and maximum HSV values** per image.
4. Computes the **overall lower and upper HSV bounds** that best generalize across all input frames.
5. (Optional) Can be extended to generate a real-time mask to test the computed HSV range visually.

---

## 🧪 Result

Here’s a summary of the computed HSV ranges across the dataset:

| Image | Lower HSV Bound | Upper HSV Bound |
|-------|------------------|------------------|
| Screenshot 12-25-01 | [18, 5, 82] | [70, 126, 143] |
| Screenshot 12-24-39 | [18, 18, 92] |[33, 117, 141] |
| Screenshot 12-24-48 | [0, 0, 63] |[178, 97, 159] |
| Screenshot 12-24-17 | [12, 22, 81] | [33, 102, 129] |
| Screenshot 12-25-09 | [4, 5, 79] | [168, 113, 158] |
| Screenshot 12-24-11 | [11, 36, 76] | [30, 127, 123] |
| Screenshot 12-24-30 | [7, 11, 70] | [64, 104, 157] |
| Screenshot 12-23-57 | [19, 30, 96] | [35, 115, 144] |
| Screenshot 12-26-51 | [14, 18, 79] | [39, 112, 136] |
| Screenshot 12-23-49 | [11, 21, 69] | [40, 127, 180] |
| Screenshot 12-26-58 | [12, 44, 71] | [34, 108, 129] |
| Screenshot 12-24-04 | [11, 8, 92] | [64, 117, 160] |

**✅ Widest HSV Range for Detection:**

- **Lower Bound:** `(0, 0, 63)`
- **Upper Bound:** `(178, 127, 180)`

Various ranges from the above were tested and ([11, 8, 92] [64, 117, 160]) yielded **robust detection accuracy** across various lighting conditions in the towing tank environment.

---

## 🧰 Requirements

- Python 3.x
- OpenCV (`cv2`)
- NumPy
- A folder of `.png` images placed inside `preprocessed_data/` relative to the script

Install dependencies:

```bash
pip install opencv-python numpy

python compute-hsv.py
