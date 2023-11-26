import numpy as np
import cv2

# Load calibration data
calibration_data = np.load('../calibration/calibration.npz')
mtx = calibration_data['mtx']
dist = calibration_data['dist']

# Load image
img = cv2.imread('../resource/cone_x40cm.png')

# Bottom right corner of cone in image coordinates (col, row)
point_of_reference = (663,493)

fx = mtx[0,0]
fy = mtx[1,1]
x0 = mtx[0,2]
y0 = mtx[1,2]

Y_car = (point_of_reference[0] - x0)*0.4/fx
H_mount = (point_of_reference[1] - y0)*0.4/fy

print("Height of camera mount: ", H_mount)

new_point = (595, 414)

x_car_new = H_mount*(fy/(new_point[1] - y0))

print("Distance to cone: ", x_car_new)