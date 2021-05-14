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
from collections import deque
from Matcher import Matcher



class Tracker(Matcher):

  #--------------------------------------------------------
  #  Constructor 
  #--------------------------------------------------------
  def __init__(self, max_obj=10, history=10):
    Matcher.__init__(self)

    self.maxObjs = max_obj
    self.history = history
    self.tracked = [] 
    self.unmatched_tracked = []
    self.matchedPairs = []
    self.enterExit = []
    
    # enter/exit parameters
    self.entering = 25
    self.exiting = 135
    self.error = 5
    
    # exit/enter for current frame
    self.currentEnter = 0
    self.currentExit = 0
    self.change = False
    
    for i in range(self.maxObjs):
      self.tracked.append(deque([])) 
      self.enterExit.append(deque([]))

  #--------------------------------------------------------
  # Scoring function; small value means batter match
  #--------------------------------------------------------
  def scoreFunc(self, a, b):
    (_, (_, val_a)) = a
    (_, (_, val_b)) = b
    score = abs(val_a-val_b)
    return score
 
  #--------------------------------------------------------
  #  Push new object to tracked FIFO 
  #--------------------------------------------------------
  def pushTracked(self, i, object):
    q = self.tracked[i]
    if len(q) >= self.history:
      q.popleft()
    q.append(object)
   
  #--------------------------------------------------------
  #  Push enter or exit parameter to FIFO 
  #  0 is exit, 1 is enter
  #--------------------------------------------------------  
  def pushEnterExit(self, i, object):
    q = self.enterExit[i]
    tempEnter = 0
    tempExit = 0
    
    last = None
    # return last obj in queue
    if len(q) > 0:
      last = q.pop()
      q.append(last)
      
    if last == 1 and object == 0:
      tempExit += 1
      print("exit")
    elif last == 0 and object == 1:
      tempEnter += 1
      print("enter")
    
    if len(q) > 0:
      q.popleft()
      # print("popped")
    q.append(object)
    
    return tempEnter, tempExit
    
  def clearCurrent(self):
    self.currentEnter = 0
    self.currentExit = 0
     
  #--------------------------------------------------------
  #  Return last object added to tracked
  #--------------------------------------------------------
  def lastTracked(self, i):
    q = self.tracked[i]
    if len(q) == 0:
      return None
    else:
      obj = q.pop()
      q.append(obj)

    return obj
    
    
  #--------------------------------------------------------
  #  Return last two objects added to tracked
  #--------------------------------------------------------
  def last2Tracked(self, i):
    q = self.tracked[i]
    if len(q) == 0:
      return None
    else:
      try: 
        obj_t1 = q.pop()
        obj_t2 = q.pop()
        q.append(obj_t2)
        q.append(obj_t1)
      except IndexError:
        pass

    return obj_t2, obj_t1
    

  #--------------------------------------------------------
  #  Find first unused tracked 
  #--------------------------------------------------------
  def findUnused(self):
    for i in range(len(self.tracked)):
       if len(self.tracked[i]) == 0:
         return i
    return -1


  #--------------------------------------------------------
  #  Init tracked objects the first time 
  #--------------------------------------------------------
  def initTracked(self, objects):
    if len(objects) <= self.maxObjs:
      for i in range(len(objects)):
        self.tracked[i].append(objects[i]) 


  #--------------------------------------------------------
  #  Update tracked objects 
  #--------------------------------------------------------
  def update(self, objects):
    self.change = False
    
    # Prepare the list of current tracked objects
    tracked = []
    for i in range(len(self.tracked)):
      q = self.tracked[i]
      if len(q) > 0: 
        tracked.append( (i, self.lastTracked(i)) )

    # Prepare the list of incoming untracked objects
    untracked = []  
    for i in range(len(objects)):
      untracked.append( (i, objects[i]) ) 

    # Perform matching
    if(len(untracked) == 0):
      self.tracked[i] = deque([])
        
    # Do the match
    matchedPairs, unmatched_tracked, unmatched_untracked = self.match(tracked, untracked)
    self.unmatched_tracked = unmatched_tracked
    self.matchedPairs = matchedPairs

    tempEnter = 0
    tempExit = 0
    # Add matched untracked objects 
    for k in range(len(matchedPairs)): 
      ( (i, _), (_, b) ) = matchedPairs[k]
      self.pushTracked(i, b)
      x, y = self.getCenter(b)
      if abs(self.entering - x) <= self.error:
        tempEnter, tempExit = self.pushEnterExit(i, 1)
      elif abs(self.exiting - x) <= self.error:
        tempEnter, tempExit = self.pushEnterExit(i, 0)
        
    if tempEnter != self.currentEnter:
      self.change = True
      self.currentEnter = tempEnter
    if tempExit != self.currentExit:
      self.change = True
      self.currentExit = tempExit 
         
    # Zero out unmatched tracked object 
    for (i, object) in unmatched_tracked:
      self.tracked[i] = deque([])  
      self.enterExit[i] = deque([])

    # Find empty track to start tracking unmatched untracked objects
    for (_, object) in unmatched_untracked:
      i = self.findUnused()
      if i != -1:
        self.tracked[i].append(object)
  
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
    return (x, y)


  #--------------------------------------------------------
  #  Returns normalized screen distance between two contours 
  #
  #  Returns -1 if error 
  #--------------------------------------------------------
  def getScreenDist(self, a, b):

    dist = -1

    if ((self.ximg is not None) and (self.yimg is not None) 
                               and (self.zimg is not None)):
 
      point_a = self.getCenter(a)
      point_b = self.getCenter(b)

      ax = self.ximg[point_a[0]][point_a[1]] 
      ay = self.yimg[point_a[0]][point_a[1]] 

      bx = self.ximg[point_b[0]][point_b[1]] 
      by = self.yimg[point_b[0]][point_b[1]] 

      dx = ax-bx
      dy = ay-by
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

      ax = self.ximg[point_a[0]][point_a[1]] 
      ay = self.yimg[point_a[0]][point_a[1]] 
      az = self.zimg[point_a[0]][point_a[1]] 

      bx = self.ximg[point_b[0]][point_b[1]] 
      by = self.yimg[point_b[0]][point_b[1]] 
      bz = self.zimg[point_b[0]][point_b[1]] 

      dx = ax - bx
      dy = ay - by
      dz = az - bz

      dist = math.sqrt(dx*dx + dy*dy + dz*dz) / self.cartNormal 

    return dist


#--------------------------------------------------------
#  Main
#--------------------------------------------------------
def main():
  tracker = Tracker() 
  group_a = [("a1", 1), ("a2", 2), ("a3", 3), ("a4", 4), ("a5", 5), ("a6", 26)] 
  group_b = [("b1", 13), ("b2", 28), ("b3", 4)] 
  group_c = [("c1", 20), ("c2", 15), ("c3", 7), ("c4", 28), ("c5", 45)] 
  tracker.update(group_a)
  print("tracker a = ", tracker.tracked) 
  print(" ")
  tracker.update(group_b)
  print("tracker ab = ", tracker.tracked) 
  print(" ")
  tracker.update(group_c)
  print("tracker abc = ", tracker.tracked) 
  print(" ")



if __name__=='__main__':
  main()
