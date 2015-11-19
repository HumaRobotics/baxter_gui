#!/usr/bin/env python


########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the Simplified BSD License on
# github: 
# HumaRobotics is a trademark of Generation Robots.
# www.humarobotics.com 

# Copyright (c) 2015, Generation Robots.
# All rights reserved.
# www.generationrobots.com
#   
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation 
#  and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
# THE POSSIBILITY OF SUCH DAMAGE. 
# 
# The views and conclusions contained in the software and documentation are 
# those of the authors and should not be interpreted as representing official 
# policies, either expressed or implied, of the FreeBSD Project.
#
#############################################################################
import rospy
import cv2
import cv_bridge

from sensor_msgs.msg import Image

from baxter_core_msgs.msg import (
    CameraSettings,
)
from baxter_core_msgs.srv import (
    CloseCamera,
    ListCameras,
    OpenCamera,
)

import baxter_interface
class CameraController(baxter_interface.camera.CameraController):
    """
    Modified init of the baxter_interface camera class 
    """

    # Valid resolutions
    MODES = [
             (1280, 800),
             (960, 600),
             (640, 400),
             (480, 300),
             (384, 240),
             (320, 200),
             ]

    # Used to represent when the camera is using automatic controls.
    # Valid for exposure, gain and white balance.
    CONTROL_AUTO = -1

    def __init__(self, name):
        """
        Constructor.

        @param name: camera identifier.  You can get a list of valid
                     identifiers by calling the ROS service /cameras/list.

                     Expected names are right_hand_camera, left_hand_camera
                     and head_camera.  However if the cameras are not
                     identified via the parameter server, they are simply
                     indexed starting at 0.
        """
        self._id = name

        self._open_svc = rospy.ServiceProxy('/cameras/open', OpenCamera)
        self._close_svc = rospy.ServiceProxy('/cameras/close', CloseCamera)

        self._settings = CameraSettings()
        self._settings.width = 320
        self._settings.height = 200
        self._settings.fps = 20
        self._open = False


class HRCamera():
    def __init__(self):
        self.cameras = {} 
        self.__getAllCameras()
            
    def openedCameras(self):
        try:
            srv = "/cameras/list"
            rospy.wait_for_service(srv, 5.0)
            camera_list_srv = rospy.ServiceProxy(srv, ListCameras)
            return camera_list_srv().cameras            
        except Exception, e:
            rospy.logerr("%s"%str(e))
            return None
        
    def __getAllCameras(self):
        """
            Gets all available cameras from the robot and create a controller for them
        """
        camera_list = self.openedCameras()
        for camera_name in ["left_hand_camera","right_hand_camera","head_camera"]:
            try:
                self.cameras[camera_name] = [CameraController(camera_name),0]
                if camera_name in camera_list:
                    self.cameras[camera_name][0]._open=True
            except Exception,e:
                rospy.logerr("could not get camera controller interface. %s",str(e))
                pass
        
        return self.cameras

    def open(self,camera_name,resolution= (960, 600)):
        """
            Opens a camera     
            :param camera_name: Name of the camera to be used
            :type camera_name: str
            :param resolution: Resolution of the camera stream
            :type resolution: (int,int) 
        """
        
        if not camera_name in self.cameras.keys():
            rospy.logerr("Camera %s does not exist!",camera_name)
            return -1
        
        if self.cameras[camera_name][0].resolution == resolution and self.cameras[camera_name][0]._open:
            rospy.loginfo("already opened with correct resolution")
            #state that camera is in use
            self.cameras[camera_name][1]+=1
            return 0
        else:
            if self.cameras[camera_name][1]>0:
                rospy.logerr("Camera is in use. It is not allowed to change the parameters. Close it first!")
                return -1
            used = 0
            opened = 0
            unused = ""
            if camera_name in self.openedCameras() and self.cameras[camera_name][1]==0:
                unused = camera_name
            else:
                for camera in self.openedCameras():
    #                 rospy.loginfo("%d workers on %s",self.cameras[camera][1],camera)    
                    if self.cameras[camera][1]>0:
                        used+=1
                    else:
                        unused = camera
                    if camera == camera_name:
                        continue                
                    if self.cameras[camera][0]._open:
                        opened+=1
                    
                if used >=2:
                    rospy.logerr("There are already 2 cameras used. Mark at least one camera that is not used in order to open the %s",camera_name)
                    return -1
    #             rospy.loginfo("unused %s",unused)
                
                if opened >= 2:
                    rospy.loginfo("closing %s",unused)
                    self.cameras[unused][0].close()
            self.cameras[camera_name][0].resolution = resolution
            try:
                rospy.loginfo("opening %s",camera_name)
                self.cameras[camera_name][0].open()
                self.cameras[camera_name][1]+=1
            except:
                rospy.logwarn("could not open camera %s",camera_name)
        return 0
                
    def close(self, camera_name):
        if not camera_name in self.cameras.keys():
            rospy.logerr("Camera name does not exist!")
            return
        self.cameras[camera_name][1]=max(0,self.cameras[camera_name][1]-1)
        
    def closeAll(self):
        #just marks that no1 is interessted in the camera
        for camera in self.cameras.keys():
            self.close(camera)
            
    
    def grabImage(self,camera_name,filename):
        """
            Grabs exactly one image from a camera
            
            :param camera_name: The image of the camera that should be saved
            :type camera_name: str
            :param filename: The full path of the filename where this image should be saved
            :type filename: str 
        """
        if self.open(camera_name) != 0:
            return
        msg=rospy.wait_for_message('/cameras/' + camera_name + "/image", Image)        
        img = cv_bridge.CvBridge().imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(filename,img)
        rospy.loginfo("Saved Image %s"%filename)
        self.close(camera_name)
  

if __name__ == "__main__":
    rospy.init_node("cameras_test")
    
    cams = HRCamera()
    print "opening left"
    cams.open("left_hand_camera")
    cams.grabImage("left_hand_camera","left_hand.jpg")
    rospy.sleep(2)
    print "2nd worker interested in left_hand_camera"
    cams.open("left_hand_camera")
    rospy.sleep(2)
    print "opening right"
    cams.open("right_hand_camera")
    cams.grabImage("right_hand_camera","right_hand.jpg")
    rospy.sleep(2)
    print "opening head. should fail"
    cams.open("head_camera")
    print "closing right hand camera"
    cams.close("right_hand_camera")
    print "opening head camera"
    cams.open("head_camera")
    cams.grabImage("head_camera","head.jpg")
    rospy.sleep(2)
    print "closing head"
    cams.close("head_camera")
    rospy.sleep(2)
    print "opening head again"
    cams.open("head_camera")
    rospy.sleep(2)
    print "opening head again, different resolution should fail"
    cams.open("head_camera",(320,200))
    cams.close("head_camera")
    print "trying again, closing it first"
    cams.open("head_camera",(320,200))
    rospy.sleep(2)
    print "closing head"
    cams.close("head_camera")
    print "open cameras:",cams.openedCameras()
    print "closing 1 worker on left camera"
    cams.close("left_hand_camera")
    print "open right hand camera"
    cams.open("right_hand_camera")
    print "open cameras:",cams.openedCameras()
    print "closing 1 worker on left camera"
    cams.close("left_hand_camera")
    print "closing right hand camera"
    cams.open("right_hand_camera")
    print "open cameras:",cams.openedCameras()
    print "closing 1 worker on left camera"
    cams.close("left_hand_camera")
    print "closing right hand camera"
    cams.open("right_hand_camera")
    print "open cameras:",cams.openedCameras()
    print "camera test done"
#     rospy.spin()