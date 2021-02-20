#==============================================================================
#
#  BlobTracker.py
#
#  Author: E-Motion Inc
#
#  Confidential and proprietary software
#  Copyright (c) 2020-2021, E-Motion, Inc.  All Rights Researcved
# 
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#  THE SOFTWARE.
#
#==============================================================================
import math
import cv2
import numpy as np
from collections import deque
from Tracker import Tracker 


class BlobTracker(Tracker):

  #--------------------------------------------------------
  #  Constructor 
  #--------------------------------------------------------
  def __init__(self, width=160, height=60, max_obj=10, history=10):
    Tracker.__init__(self, max_obj, history)
    self.alpha = 0.5
    self.ximg = None
    self.yimg = None
    self.zimg = None
    self.width = width
    self.height = height 
    self.cartNormal = 2.0  # Set to dstance in meter  

  #--------------------------------------------------------
  #  Returns contour center as (x, y) tuple
  #--------------------------------------------------------
  def setPointCloud(self, ximg, yimg, zimg):
    self.ximg = ximg
    self.yimg = yimg
    self.zimg = zimg

  #--------------------------------------------------------
  #  Returns contour center as (x, y) tuple
  #--------------------------------------------------------
  def getCenter(self, a):
    M = cv2.moments(a)
    
    x = M['m10']/M['m00']
    y = M['m01']/M['m00']
    return (int(x), int(y))
    


  #--------------------------------------------------------
  #  Returns normalized screen distance between two contours 
  #
  #  Returns -1 if error 
  #--------------------------------------------------------
  def getScreenDist(self, a, b):

    point_a = self.getCenter(a)
    point_b = self.getCenter(b)

    dx = point_a[1] - point_b[1]
    dy = point_a[0] - point_b[0]
    dist = math.sqrt(dx*dx + dy*dy) / math.sqrt(self.width**2 + self.height**2) 
      
    return dist
 
  #--------------------------------------------------------
  #  Returns normalized cart distance between two contours 
  #
  #  Returns -1 if error 
  #--------------------------------------------------------
  def getCartDist(self, a, b):

    dist = -1

    if ((self.ximg is not None) and (self.yimg is not None) 
                               and (self.zimg is not None)):
 
      point_a = self.getCenter(a)
      point_b = self.getCenter(b)

      ax = self.ximg[point_a[1]][point_a[0]] 
      ay = self.yimg[point_a[1]][point_a[0]] 
      az = self.zimg[point_a[1]][point_a[0]] 

      bx = self.ximg[point_b[1]][point_b[0]] 
      by = self.yimg[point_b[1]][point_b[0]] 
      bz = self.zimg[point_b[1]][point_b[0]] 

      dx = ax - bx
      dy = ay - by
      dz = az - bz

      dist = math.sqrt(dx*dx + dy*dy + dz*dz) / self.cartNormal 

    return dist
    
  #--------------------------------------------------------
  #  Return cartesian coordinates given screen coordinate 
  #--------------------------------------------------------
  def getCartPos(self, a):

    if ((self.ximg is not None) and (self.yimg is not None) 
                               and (self.zimg is not None)):
 
      point_a = self.getCenter(a)

      ax = self.ximg[point_a[1]][point_a[0]] 
      ay = self.yimg[point_a[1]][point_a[0]] 
      az = self.zimg[point_a[1]][point_a[0]] 

    return [ax, ay, az]
    
    
  #---------------------------------------------------
  #  Given after blobTracker is updated, find
  #  velocities from the updated tracked collection
  #---------------------------------------------------
  def findVelocity(self, dt):
   
    velocity = []

    for i in range(len(self.tracked)):
      
      v = [0, 0, 0]
      
      # If the deque is NOT empty:
      if len(self.tracked[i]) > 0:
        # Get the deque
        q = self.tracked[i]
        
           
        if len(q) >= 2:
          # Get last contour
          t1_contour, t0_contour = self.last2Tracked(i)
           
          # find current position in Cartesian
          t0_pos = self.getCartPos(t0_contour)
           
          # find t-1 position
          t1_pos = self.getCartPos(t1_contour)
           
          # find change in position
          dpx = (t0_pos[0]) - (t1_pos[0])
          dpy = (t0_pos[1]) - (t1_pos[1])
          dpz = (t0_pos[2]) - (t1_pos[2])
          
          
          # Find velocity: 
          vx = dpx / dt
          vy = dpy / dt
          vz = dpz / dt
          
          v = [vx,vy,vz]
        
          
        velocity.append(v)
    
    return velocity
    
 
  #--------------------------------------------------------
  #  Scores the disparity between contours a and b 
  #--------------------------------------------------------
  def getLastObjects(self):
    objs = []
    for i in range(len(self.tracked)):
      if len(self.tracked[i]) > 0:
        objs.append(self.lastTracked(i))
    return objs

 
  #--------------------------------------------------------
  #  Scores the disparity between contours a and b 
  #--------------------------------------------------------
  def scoreFunc(self, ap, bp):
    (_, a) = ap
    (_, b) = bp
    shape = cv2.matchShapes(a, b, 3, 0)
    dist = self.distanceScore(a, b)
    #  dist = self.getCartDist(a, b)
    score = (1.0-self.alpha)*dist + self.alpha*shape
    return score
    
  def distanceScore(self, a, b):
    # dividing by the actual size of frame normalizes the value (between 0 and 1)
    ax, ay = self.findCenter(a)
    bx, by = self.findCenter(b)
    dx = (ax - bx) / 160.0
    dy = (ay - by) / 60.0
    dist = math.sqrt(dx*dx + dy*dy)
    return dist
  
  def findCenter(self, blob):
    m = cv2.moments(blob)
    if m['m00'] != 0:
      bx = int(m['m10']/m['m00'])
      by = int(m['m01']/m['m00'])
    else:
      print("illegitimate blob")
      bx = 0
      by = 0
    return bx, by
 
#--------------------------------------------------------
#  Main
#--------------------------------------------------------
def main():
  pass 

  # Below is pseudo code illustrating how to use the
  # BlobTracker() class
 
  tracker = BlobTracker()

  # In meters.  Set to what makes sense
  tracker.cartNormal = 3.0 

  # Dial that blends shape vs dist matching
  #  alpha = 0 --> dist matching only 
  #  alpha = 1 --> shape matching only 
  tracker.alpha = 0.5  # dial go between shape vs dist matching   

  # Update loop
  # ~ done = False
  # ~ while not done:

    # ~ # Get new dimg, ximg, yimg, zimg
    
 
    # ~ # Pass current ximg, yimg, zimg to tracker
    # ~ tracker.setPointCloud(ximg, yimg, zimg)

    # ~ # Call function to get new contours 
    # ~ contours = find_objects(dimg)

    # ~ # Update tracker
    # ~ tracker.update(contours)
   
    # ~ # Updated result is in tracker.tracked, which is an array of deque
    # ~ for i in range(len(tracker.tracked)):
      # ~ if len(tracker.tracked[i]) > 0:
        # ~ # Get the deque
        # ~ q = tracker.tracked[i]
  
        # ~ # Call getPVA to get position, velocity and acceleration
        # ~ p, v, a = getPVA(q)

        # ~ print("Track#", i, ":", "Position= ",p," Velocity= ",v," Accel= ", a)

      # ~ else:

        # ~ print("Track# ", i, ":" , "Unused")

 
if __name__=='__main__':
  main()
