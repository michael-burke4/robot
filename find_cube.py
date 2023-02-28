import cv2
import numpy as np
import time

def filter_image(img, hsv_lower, hsv_upper):
    img_filt = cv2.medianBlur(img, 11)
    hsv = cv2.cvtColor(img_filt, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    return mask

def detect_blob(mask):
    img = cv2.medianBlur(mask, 5)
   # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 256;
    #filter by color (on binary)
    params.filterByColor = True
    params.blobColor = 255  # this looks at binary image 0 for looking for dark areas
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 50
    params.maxArea = 200000
    # Filter by Circularity
    params.filterByCircularity = False
    # Filter by Convexity
    params.filterByConvexity = False
    # Filter by Inertia
    params.filterByInertia = False
    detector = cv2.SimpleBlobDetector_create(params)
    # Detect blobs.
    keypoints = detector.detect(img)
    return keypoints

def find_cube(img, hsv_lower, hsv_upper):
    step1 = filter_image(img, hsv_lower, hsv_upper)
    mask = step1
    keypoints = detect_blob(mask)
    # print("num keypoints:", len(keypoints))
    if len(keypoints) == 0:
        return None

    big = -1
    big_ind = 0
    for i in range(len(keypoints)):
        if keypoints[i].size > big:
            big = keypoints[i].size
            big_ind = i

    return keypoints[big_ind]