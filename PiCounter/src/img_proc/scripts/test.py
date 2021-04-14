import cv2
import numpy as np

im = np.zeros((60,160,3), np.uint8)
im = cv2.ellipse(im,(40,40),(20,30),45,0,360,(255,255,255),-1)
im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
contour, hierarchy = cv2.findContours(im, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

dimg = np.zeros((60,160,3), np.uint8)
dimg = cv2.cvtColor(dimg, cv2.COLOR_BGR2GRAY)
for cnt in contour:
  approx = cv2.approxPolyDP(cnt, .009 * cv2.arcLength(cnt, True), True)
  cv2.drawContours(dimg, [approx], 0, (0,0,255), 5)
  n = approx.ravel()
  i = 0
  
  for j in n:
    if i%2==0:
      x = n[i]
      y = n[i+1]
      
      if x > 60 or y > 160:
        pass
      else:
        print(dimg[x, y] == 0)
      cv2.circle(dimg, (x,y), radius=0, color=(255,255,255), thickness=-1)
    i += 1
    
cv2.imshow("cnts", dimg)
cv2.waitKey(0)
