#!/usr/bin/python3
#===========================================================================
#
#  img_proc_node.py
#
#  Copyright (C) 2020, E-Motion, Inc - All Rights Reserved
#  Unauthorized copying of this file, via any medium is
#  strictly prohibited
#
#  Proprietary and confidential
#  Written by Larry Li <larry@e-motion.ai>
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
#  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#  DEALINGS IN THE SOFTWARE.
#
#===========================================================================

import sys 
import sys 
import cv2
import math
import json 
from datetime import datetime
import queue
import numpy as np
import paho.mqtt.client as mqtt
from BlobTracker import BlobTracker

# Max and min depth cutoff  
max_depth = 0
min_depth = 0

# Morpho kernel
kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(8,8))

# Background subtraction algorithms
fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows = False)
    
# Background mask
fgmask = None
dimg1 = None

    
 # Background learning params
learningRateMax = .001
learningRateAlpha = .0001
    
frame1_blobs = []
    
# Blob params
minBlobArea = 10
minBlobPeri = 10
    
# For weighting the shape / location of blob tracking
alpha = .7
    
# Matched blobs (number of valid blobs)
matchedBlobs = []
trackedBlobs = []
    
# Enter / Exit paramaters
entering = 10
exiting = 150
error = 35

tracker = BlobTracker()

    
# total number of people who have entered
totalEntered = 0

#===================================================
# Scale image 
#===================================================
def scaleImage(img, factor):
  w = int(img.shape[1] * factor)
  h = int(img.shape[0] * factor)
  dim = (w, h)
  newImg = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
  return newImg

  #===================================================
  # Prepare array for display by scaling and 
  # normalizing, and filtering
  #===================================================
def prepare(img, scale):
  img = scaleImage(img, scale)
  img = cv2.normalize(img, None, 0, 65535 , cv2.NORM_MINMAX, cv2.CV_16U)
  img = cv2.medianBlur(img, 5)
  return img

  #===================================================
  # Get contour of blob
  #   return contour and hierarchy
  #===================================================
def update_contour(bimg):
  contour, hierarchy = cv2.findContours(bimg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
  bimg = cv2.cvtColor(bimg, cv2.COLOR_GRAY2RGB)
  cv2.drawContours(bimg, contour,0, (0, 255, 0), 1) 
  cv2.imshow('contour', prepare(bimg, 4))
  return contour

  #===================================================
  #
  # Clean up a binary image with morphological open 
  # returns a cleaned up binary image
  #
  #===================================================
def morph_clean(bimg):
  out = cv2.morphologyEx(bimg, cv2.MORPH_OPEN, kernel) 
  return out
 
  #===================================================
  #  Find foreground mask
  #===================================================
def get_fgnd(dimg):
  if fgmask is None:
    learningRate = 0.001
    dimg1 = dimg
  else:
    abs_diff = np.sum(cv2.absdiff(dimg1, dimg))/255.0
    learningRate = computeLearningRate(abs_diff)
    dimg1 = dimg
  fgmask = fgbg.apply(dimg, learningRate)
  fgmask = cv2.compare(fgmask, 0, cv2.CMP_GT)
  gmask = morph_clean(fgmask)
  return 
    
  #===================================================
  #  Compute learningRate based on movement
  #===================================================
def computeLearningRate(diff):
  lr = 0
  alpha = learningRateAlpha
  lrMax = learningRateMax
  if diff < 96:
    eta = diff / 9600.0
    lr = alpha / (eta + alpha/lrMax)
  return lr

 
  #===================================================
  #  Get current contour
  #===================================================
def getBlobs(depthFgnd):
  contourList = update_contour(depthFgnd)
  contours = []
  for cnt in contourList:
    area = cv2.contourArea(cnt)
    length = cv2.arcLength(cnt, True)
    if area > minBlobArea and length > minBlobPeri:
      contours.append(cnt)
      # print(area, length)
  return contours
  
  #===================================================
  #  Distance score based on normalized distanceScore
  #===================================================	
def distanceScore(ax, ay, bx, by):
  # dividing by the actual size of frame normalizes the value (between 0 and 1)
  dx = (ax - bx) / 160.0
  dy = (ay - by) / 60.0
  dist = math.sqrt(dx*dx + dy*dy)
  return dist
  
def addTrackedBlobs(blob_a, blob_b):
  global trackedBlobs
  matched = False
  for i in trackedBlobs:
    j = len(i)
    if findCenter(blob_b) == findCenter(i[j-1]) and not matched:
      i.append(blob_a.copy())
      print(findCenter(blob_b))
      print("added match")
      matched = True
  return
  
  #===================================================
  #  Finding x and y of centroid
  #===================================================  
def findCenter(blob):
  m = cv2.moments(blob)
  if m['m00'] != 0:
    bx = int(m['m10']/m['m00'])
    by = int(m['m01']/m['m00'])
  else:
    print("illegitimate blob")
    bx = 0
    by = 0
  return bx, by
    
  #===================================================
  #  Create list of all possible matches (relationships)
  #  along with score based on similarity of shape and
  #  distance between centers
  #===================================================
def findRelationships(frame1_blobs, frame2_blobs):
  relationship = []
  global alpha
  for i in range(len(frame1_blobs)):
    for j in range(len(frame2_blobs)):
      blob_a = frame1_blobs[i]
      blob_b = frame2_blobs[j]
      # hu moment score, how similar the shape is to the other
      # lower number is better
      hm_score = cv2.matchShapes(blob_a, blob_b, 1, 0.0)
      ax, ay = findCenter(blob_a)
      bx, by = findCenter(blob_b)
      dist_score = distanceScore(ax, ay, bx, by)
      score = alpha*hm_score + (1-alpha)*dist_score
      relationship.append((i,j,score))
  return relationship
    
  #===================================================
  #  Given a set of potential matches (relationship),
  #  find the best match
  #===================================================
def findBestMatch(relationship):
  blob_a = -1
  blob_b = -1
  minScore = 0
  for k in range(len(relationship)):
    (i, j, score) = relationship[k]
    if k == 0:
      minScore = score
      blob_a = i
      blob_b = j
    else:
      # checking if new score is lower than minScore,
      # meaning a more accurate match
      if score < minScore:
        minScore = score
        blob_a = i
        blob_b = j
  return blob_a, blob_b, minScore
    
  #===================================================
  #  Remove any relationships involving blob_a or blob_b
  #===================================================
def removeRelationships(blob_a, blob_b, relationship):
  new_relationship = []
  for k in range(len(relationship)):
    (i, j, score) = relationship[k]
    if i != blob_a and j != blob_b:
      new_relationship.append((i, j, score))
  return new_relationship
    
  #===================================================
  #  Find the matches between blobs in frame 1 and 2 by:
  #
  #    1. Find all possible matches and their scores
  # 
  #    2. Find the best match and remove the matched
  #       pair of blobs from the list for further consideration
  # 
  #    3. Repeat 2 until nothing left on the list to consider
  #
  #  Function returns all the matches and the remaining
  #  unmatched relationships
  #===================================================
def match(frame1_blobs, frame2_blobs):
  global trackedBlobs
  matches = []
  remains = findRelationships(frame1_blobs, frame2_blobs)
  while len(remains) != 0:
    (blob_a, blob_b, score) = findBestMatch(remains)
    if blob_a != -1:
      addTrackedBlobs(frame1_blobs[blob_a], frame2_blobs[blob_b])
      matches.append((blob_a, blob_b, score))
      remains = removeRelationships(blob_a, blob_b, remains)
  return matches, remains
    
  #===================================================
  #  Periodic call to publish data 
  #===================================================   
def enterOrExit(unmatched_tracked):
    peopleEntering = 0
    peopleExiting = 0
    
    for (i, blob) in unmatched_tracked:
     x, y = findCenter(blob)
     print('blob center', x, y)
     if abs(x - entering) <= error:
        peopleEntering += 1
     elif abs(x - exiting) <= error:
        peopleExiting += 1
    print('people entering', peopleEntering, 'people exiting', peopleExiting)
    return peopleEntering, peopleExiting
      
def track(dimg):
  display = dimg.copy()
  display = cv2.cvtColor(display, cv2.COLOR_GRAY2RGB)
  
  remains = []
  changeInPeople = False
  
  global tracker
  if dimg is not None:
    blobs = getBlobs(dimg)
    
    tracker.cartNormal = 3.0 
    tracker.alpha = 0  # dial go between shape vs dist matching   
    
    # Update tracker
    tracker.update(blobs)
   
    # Updated result is in tracker.tracked, which is an array of deque
    for i in range(len(tracker.tracked)):
      if len(tracker.tracked[i]) > 0:
        # Get the deque
        q = tracker.tracked[i]
        x = -1
        y = -1
        for j in q:
          cv2.circle(display, (findCenter(j)), 1, (0,0,255), -1)
          if x > -1 and y > -1:
            cv2.line(display, findCenter(j), (x, y), (0,0,255), 1)
          x, y = findCenter(j)
    
  if len(tracker.unmatched_tracked) > 0:
    print(len(tracker.unmatched_tracked))
    for j in  tracker.unmatched_tracked:
      if len(j) ==1:
        print(findCenter(j))
      if len(j) ==2:
        a, b = j
        print(findCenter(a))
        print(findCenter(b))
    peopleEntered, peopleExited = enterOrExit(tracker.unmatched_tracked)

  cv2.imshow("depth", prepare(display, 4))  
  cv2.waitKey(0)
    
      
# s for first test case
# ss for second test case   
# sss for third test case 
# test for fourth test case
# test1_ for fifth test case
for i in range(43):
  path = '/home/ubuntu/Pictures/test1_'+str(i)+'.jpg'
  im = cv2.imread(path)
  if i==42:
    im = np.zeros((60,160,3), np.uint8)
  im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
  thresh, binary = cv2.threshold(im, 127, 255, cv2.THRESH_BINARY)
  track(binary)

