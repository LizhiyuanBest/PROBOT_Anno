#!~/anaconda3/bin/env	python3

import python3_in_ros
import cv2 
import numpy as np

img = cv2.imread('./image_21.jpg')
print(img.shape)
cv2.imshow('img', img)
cv2.waitKey(0)
