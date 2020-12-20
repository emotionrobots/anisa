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
from img_proc.cfg import img_procConfig
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import paho.mqtt.client as mqtt

node = None
client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    client.subscribe("topic2")


def on_message(client, userdata, msg):
    print("Message received-> " + msg.topic + " " + str(msg.payload))
    
def getDateTime(now):
    dt_string = now.strftime("%d/%m/%Y %H:%M%S")
    return dt_string

client.on_connect = on_connect
client.on_message = on_message

client.connect("52.43.181.166")

class Message:
  def __init__(self, device, deviceid, longitude, latitude, location, time, enter, exit, 			peopleinbuilding):
    self.device = device
    self.deviceid = deviceid
    self.longitude = longitude
    self.latitude = latitude
    self.location = location
    self.time = time
    self.enter = enter
    self.exit = exit
    self.peopleinbuilding = peopleinbuilding

  def dictStr(self):
    d = {}
    d["device"] = self.device
    d["deviceid"] = self.deviceid
    d["longitude"] = self.longitude
    d["latitude"] = self.latitude
    d["location"] = self.location
    d["time"] = self.time
    d["enter"] = self.enter
    d["exit"] = self.exit
    d["peopleinbuilding"] = self.peopleinbuilding
    return json.dumps(d)

#===========================================================================
#  dynamic_reconfigure callback 
#===========================================================================
def dr_callback(config, level):
  return config


#===========================================================================
# Process front-facing cam635
#===========================================================================
class ImgProcNode(object):
   
  #===================================================
  # Constructor 
  #===================================================
  def __init__(self):

    rospy.init_node('img_proc', anonymous=False)

    self.camera = {}
    self.camera['amp'] = None
    self.camera['depth'] = None
    self.camera['x'] = None
    self.camera['y'] = None
    self.camera['z'] = None
    self.camera['chip_id'] = None
    self.camera['wafer_id'] = None

    # Setup transform listener 
    self.xform_buf = tf2_ros.Buffer()
    self.xform_listener = tf2_ros.TransformListener(self.xform_buf)

    # Max and min depth cutoff  
    self.max_depth = 0
    self.min_depth = 0

    # Morpho kernel
    self.kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3,3))

    # Support ROS message to CV image conversion 
    self.br = CvBridge()
    
    # Background subtraction algorithms
    self.fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows = False)
    # self.fgbg = cv2.createBackgroundSubtractorKNN(detectShadows = False)
    
    # Background mask
    self.fgmask = None
    self.dimg1 = None
    
    self.maxDepth = 5000
    
    # Background learning params
    self.learningRateMax = .001
    self.learningRateAlpha = .0001
    
    # Blob params
    self.minBlobArea = 0
    self.minBlobPeri = 0

    # Subscribe to camera data 
    rospy.Subscriber('/espros_tof_cam635/camera/image_raw1', Image, self.amp_callback)
    rospy.Subscriber('/espros_tof_cam635/camera/image_raw2', Image, self.depth_callback)
    rospy.Subscriber('/espros_tof_cam635/camera/points', PointCloud2, self.pc_callback)

    return

  #===================================================
  # 
  #  Extract amplitude or depth array from incoming
  #  message
  #
  #===================================================
  def getArray(self, msg):
    a = self.br.imgmsg_to_cv2(msg, desired_encoding='mono16')
    a = cv2.flip(a, -1) 
    return a 

  #===================================================
  #
  #  Extract X, Y, Z arrays from incoming point cloud 
  #  message; adjust to common global reference frame 
  # 
  #  Returns x array, y array and z array
  #
  #===================================================
  def getXYZArrays(self, msg):
    gen = pc2.read_points(msg, field_names = ('x','y','z'), skip_nans=False)
    points = np.array([p for p in gen]) # p is (x,y,z) tuple
    points = points.reshape(-1, 160, 3) # row,col
    self.camera['x'] = cv2.flip(points[:,:,0], -1)
    self.camera['y'] = cv2.flip(points[:,:,1], -1)
    self.camera['z'] = cv2.flip(points[:,:,2], -1)
    return 

  #===================================================
  #
  # Scale image 
  #
  #===================================================
  def scaleImage(self, img, factor):
    w = int(img.shape[1] * factor)
    h = int(img.shape[0] * factor)
    dim = (w, h)
    newImg = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return newImg

  #===================================================
  #
  # Prepare array for display by scaling and 
  # normalizing, and filtering
  #
  #===================================================
  def prepare(self, img, scale):
    img = self.scaleImage(img, scale)
    img = cv2.normalize(img, None, 0, 65535 , cv2.NORM_MINMAX, cv2.CV_16U)
    img = cv2.medianBlur(img, 5)
    return img

  #===================================================
  #
  #  Process camera amp image
  #
  #===================================================
  def amp_callback(self, msg):
    self.camera['amp'] = self.getArray(msg)  
    return

  #===================================================
  #
  #  Process camera depth image
  #
  #===================================================
  def depth_callback(self, msg):
    self.camera['depth'] = self.getArray(msg)
    return

  #===================================================
  #
  #  Process camera point cloud 
  #
  #===================================================
  def pc_callback(self, msg):
    self.getXYZArrays(msg)
    return

  #===================================================
  #
  #  Get binary image 
  #
  #===================================================
  def getBinaryImage(self, dimg):
    if dimg is not None:
    # try something smaller here, maybe 1500
      dimg = cv2.compare(dimg, self.maxDepth, cv2.CMP_LT)
      self.get_fgnd(dimg)
    return self.fgmask

  #===================================================
  #
  # Get contour of blob
  #   return contour and hierarchy
  #
  #===================================================
  def update_contour(self, bimg):
    _,contour,_ = cv2.findContours(bimg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contour

  #===================================================
  #
  # Clean up a binary image with morphological open 
  # returns a cleaned up binary image
  #
  #===================================================
  def morph_clean(self, bimg):
    out = cv2.morphologyEx(bimg, cv2.MORPH_OPEN, self.kernel) 
    return out
 
  #===================================================
  #
  #===================================================
  def get_fgnd(self, dimg):
    if self.fgmask is None:
    	learningRate = 0.001
    	self.dimg1 = dimg
    else:
    	abs_diff = np.sum(cv2.absdiff(self.dimg1, dimg))/255.0
    	learningRate = self.computeLearningRate(abs_diff)
    	self.dimg1 = dimg
    self.fgmask = self.fgbg.apply(dimg, learningRate)
    self.fgmask = cv2.compare(self.fgmask, 0, cv2.CMP_GT)
    self.fgmask = self.morph_clean(self.fgmask)
    return 
    
  def computeLearningRate(self, diff):
    lr = 0
    alpha = self.learningRateAlpha
    lrMax = self.learningRateMax
    if diff < 96:
    	eta = diff / 9600.0
    	lr = alpha / (eta + alpha/lrMax)
    	print (lr)
    return lr

  #===================================================
  #  Return Laplacian edge
  #
  #===================================================
  def laplaceEdge(self, dimg, threshold):
    edge = None
    if dimg is not None:
      blur = cv2.GaussianBlur(dimg, (5,5), sigmaX=0)
      laplacian = np.abs(cv2.Laplacian(blur, cv2.CV_32F, ksize=3))
      edge = cv2.compare(laplacian, threshold, cv2.CMP_GT)
    return edge
    
  #===================================================
  #
  #  Get current contour
  #===================================================
  def getBlobs(self, depthFgndMask):
    contourList = self.update_contour(depthFgndMask)
    contour = []
    area = cv2.contourArea(cnt)
      # change blob params
    if area > self.minBlobArea and length > self.minBlobPeri:
      contours.append(cnt)
    return contours
  	

  #===================================================
  #
  #  Periodic call to publish data 
  #
  #===================================================
  def periodic(self):
    dimg = self.camera['depth']
    aimg = self.camera['amp']
    depthFgndMask = self.getBinaryImage(dimg)
    laplace = self.laplaceEdge(dimg, 200)
    depthFgnd = cv2.bitwise_and(dimg, dimg, mask = depthFgndMask)
    if dimg is not None:
      cv2.imshow("depth", self.prepare(dimg, 4))
    if aimg is not None:
      cv2.imshow("amplitude", self.prepare(aimg, 4))
    if depthFgndMask is not None:
       cv2.imshow("binary", self.prepare(depthFgndMask, 4))
    if dimg is not None:
      cv2.imshow("foreground", self.prepare(depthFgnd, 4))
    # MAKE SEPARATE METHOD FOR FORMATTING THE STRING IN THE RIGHT ORDER
    # device, deviceid, longtitude, latitude, location, time, enter, exit, people in building
    now = datetime.now()
    m1 = Message("rpi4", 36, 455, 566, "School, Gym", getDateTime(now), 23, 32, 24)
    client.publish("topic1", m1.dictStr())
    return

  #===================================================
  #
  #  Start processing 
  #
  #===================================================
  def start(self):
    while not rospy.is_shutdown():
      self.periodic()
      key = cv2.waitKey(33)
      if (key & 0xFF) == ord('q'):
        break 
    return


#===========================================================================
#  Main() 
#===========================================================================
def main(argv):
  global node
  node = ImgProcNode()
  srv = Server(img_procConfig, dr_callback)
  node.start()
  return

if __name__=='__main__':
  main(sys.argv[1:])    
  #
