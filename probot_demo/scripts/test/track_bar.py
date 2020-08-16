# -*- coding: utf-8 -*-

import cv2
import numpy as np
def nothing(x):
    pass
img = np.zeros((312,520,3),dtype=np.uint8)
cv2.namedWindow('image')
cv2.createTrackbar('R','image',0,255,nothing)
cv2.createTrackbar('G','image',0,255,nothing)
cv2.createTrackbar('B','image',0,255,nothing)
while True:
    cv2.imshow('image',img)
    r = cv2.getTrackbarPos('R','image')
    g = cv2.getTrackbarPos('G','image')
    b = cv2.getTrackbarPos('B','image')
    img[:] = [b,g,r]
    c = cv2.waitKey(1)
    if c == ord('q'):
        break
cv2.destroyAllWindows()
