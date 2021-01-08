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
import rospy
import cv2
import math
import json 
from datetime import datetime
import queue
import numpy as np
import sensor_msgs.point_cloud2 as pc2 
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2 
from std_msgs.msg import String 
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

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
minBlobArea = 50
minBlobPeri = 50
    
# For weighting the shape / location of blob tracking
alpha = .7
    
# Matched blobs (number of valid blobs)
matchedBlobs = []
trackedBlobs = []
    
# Enter / Exit paramaters
entering = 10
exiting = 150
error = 35

    
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
  cv2.drawContours(bimg, contour, -1, (0, 255, 0), 3) 
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
def enterOrExit(matches, frame1):
  peopleEntering = 0
  peopleExiting = 0
  for k in range(len(matches)):
    (a, b, score) = matches[k]
    blob = frame1[a]
    ax, ay = findCenter(blob)
    # left is entrance
    # right is exit
    if abs(ax - entering) <= error:
      peopleEntering += 1
    elif abs(ax - exiting) <= error:
      peopleExiting += 1
  print('people entering:', peopleEntering)
  print('people exiting:', peopleExiting)
  return peopleEntering, peopleExiting    
      
def track(dimg):
  display = dimg.copy()
  display = cv2.cvtColor(display, cv2.COLOR_GRAY2RGB)
  
  remains = []
  changeInPeople = False
  
  global frame1_blobs
  global matchedBlobs
  global totalEntered
  global trackedBlobs
   
  if dimg is not None:
    blobs = getBlobs(dimg)
    
    # the first frame
    if len(frame1_blobs) == 0:
      frame1_blobs = blobs
      # initialize list of trackedBlobs (list of lists)
      for k in range(len(blobs)):
        trackedBlobs.append([])
        trackedBlobs[k].append(blobs[k].copy())
        
    # if there are any blobs in the frame
    elif len(blobs) != 0:
      # match(current frame, previous frame)
      matchedBlobs, remains = match(blobs, frame1_blobs)
     
    # once there are fewer blobs currently than there were before, meaning
    # a blob has left the
      if len(blobs) < len(frame1_blobs):
        peopleEntered, peopleExited = enterOrExit(matchedBlobs, frame1_blobs)
        # update total number of people who have entered
        totalEntered += peopleEntered
        totalEntered -= peopleExited
        changeInPeople = True
        frame1_blobs = blobs
        
      frame1_blobs = blobs
      
    elif len(blobs) < len(frame1_blobs):
      peopleEntered, peopleExited = enterOrExit(matchedBlobs, frame1_blobs)
      # update total number of people who have entered
      totalEntered += peopleEntered
      totalEntered -= peopleExited
      changeInPeople = True
      frame1_blobs = blobs
      
    else:
      frame1_blobs = blobs
      matchedBlobs = []

    # display blobs now
    # draw last point
    for i in trackedBlobs:
      for j in range(len(i)):
        cv2.circle(display, (findCenter(i[j])), 2, (0,0,255), -1)
        if j>0:
          cv2.line(display, findCenter(i[j]), findCenter(i[j-1]), (0,0,255),1)
        
    cv2.imshow("depth", prepare(display, 4))  
    cv2.waitKey(0)
    
  # device, deviceid, longtitude, latitude, location, time, enter, exit, people in building
  if changeInPeople:
    now = datetime.now()
 
paths = []
paths.append('/home/ubuntu/Pictures/t00.jpg')
paths.append('/home/ubuntu/Pictures/t01.jpg')
paths.append('/home/ubuntu/Pictures/t02.jpg')
paths.append('/home/ubuntu/Pictures/t03.jpg')
paths.append('/home/ubuntu/Pictures/t04.jpg')
paths.append('/home/ubuntu/Pictures/t05.jpg')
paths.append('/home/ubuntu/Pictures/t06.jpg')
paths.append('/home/ubuntu/Pictures/t07.jpg')
paths.append('/home/ubuntu/Pictures/t08.jpg')

ims = [0] * 9
for i in range(len(paths)):
  ims[i] = cv2.imread(paths[i])
  ims[i] = cv2.cvtColor(ims[i], cv2.COLOR_BGR2GRAY)
  track(ims[i])




