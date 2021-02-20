#==============================================================================
#
#  matcher.py
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
import lap as lap
import numpy as np
import math 
import time


class Matcher():

  def __init__(self):
    self._clear()

  def _clear(self):
    self.matchedPair = [] 
    self.unmatched_a = [] 
    self.unmatched_b = [] 
    self.cost = 0 
    self.assign_a = []
    self.assign_b = []

  def scoreFunc(self, a, b):
    pass
 
  def match(self, group_a, group_b, maxScore=float("inf")):
    self._clear()

    if (len(group_a) > 0) and (len(group_b) == 0):
      self.unmatched_a = group_a
    elif (len(group_a) == 0) and (len(group_b) > 0):
      self.unmatched_b = group_b
    elif (len(group_a) == 0) and (len(group_b) == 0):
      return self.matchedPair, self.unmatched_a, self.unmatched_b 
    else:
      # Setup the weight matrix
      costs = np.zeros( (len(group_a), len(group_b)) )
      for i in range(len(group_a)):
        for j in range(len(group_b)):
          costs[i, j] = self.scoreFunc(group_a[i], group_b[j])
 
      # Perform the match
      self.cost, self.assign_a, self.assign_b = lap.lapjv(costs, extend_cost=True)  
  
      # Extract matched pairs and unmatched a and b
      for i in range(len(self.assign_a)):
        if self.assign_a[i] == -1:
          self.unmatched_a.append(group_a[i])   
        else:
          a = group_a[i]
          b = group_b[self.assign_a[i]]
          if self.scoreFunc(a, b) < maxScore: 
            self.matchedPair.append( (group_a[i], group_b[self.assign_a[i]]) ) 
          else:
            self.unmatched_a.append(a)
            self.unmatched_b.append(b)

      if len(group_b) > len(group_a):
        for i in range(len(self.assign_b)):
          if self.assign_b[i] == -1:
            self.unmatched_b.append(group_b[i]) 

    return self.matchedPair, self.unmatched_a, self.unmatched_b 


#--------------------------------------------------------
#  Match scoring function
#--------------------------------------------------------
def scoring(a, b):
  dist = abs(a-b)
  return dist 
 
def scoring2d(a, b):
  dist = math.sqrt( (a[0]-b[0])**2 + (a[1]-b[1])**2 )
  return dist


