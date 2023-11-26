import numpy as np
import cv2

# Load calibration data
calibration_data = np.load('../calibration/calibration.npz')
mtx = calibration_data['mtx']
dist = calibration_data['dist']

# Load image
img = cv2.imread('../resource/lane.png')

# HSV threshold for yellow
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower_yellow = np.array([20, 65, 100])
upper_yellow = np.array([40, 255, 255])
mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
cv2.imshow('mask', mask)

# Find contours
contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
cv2.imshow('contours', img)
cv2.waitKey(5000)
cv2.destroyAllWindows()