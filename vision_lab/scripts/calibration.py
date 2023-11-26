import numpy as np
import cv2
import glob

square_side = 0.25 # meters
pattern_size = (8, 6) # corners

# Prepare object points
objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp = objp * square_side

# Arrays to store object points and image points from all the images
objpoints = [] # 3D points in real world space
imgpoints = [] # 2D points in image plane

# Make a list of calibration images
images = glob.glob('../calibration/*.png')
print("Found %d images" % len(images))

# Step through the list and search for chessboard corners
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print("Processing %s" % fname)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    # If found, add object points, image points
    if ret == True:
        print("Found corners")
        objpoints.append(objp)
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, pattern_size, corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibrate camera
print("Calibrating camera...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("Camera calibration complete")
print("Camera matrix:", mtx)
print("Distortion coefficients:", dist)

print("Saving calibration data...")
np.savez('../calibration/calibration.npz', mtx=mtx, dist=dist)