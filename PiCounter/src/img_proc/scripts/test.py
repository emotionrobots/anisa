import cv2
import numpy as np

im = np.zeros((60,160,3), np.uint8)
im = cv2.ellipse(im,(40,40),(20,30),45,0,360,(255,255,255),-1)
cv2.imshow('ellipse',im)
cv2.waitKey(0)
