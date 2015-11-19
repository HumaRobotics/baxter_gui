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
from std_msgs.msg import UInt16
from sensor_msgs.msg import PointCloud
from threading import Lock
class Sonar:
    """
        Enables or disables the sonar sensors
    """
    def __init__(self):
        self.state = 0
        self.distances = [None for x in xrange(12)]
        self.mutex = Lock()
        self.__sonar_pub = rospy.Publisher("/robot/sonar/head_sonar/set_sonars_enabled",UInt16, queue_size=1)
        self.__sonar_sub = rospy.Subscriber("/robot/sonar/head_sonar/state",PointCloud,self.callback,queue_size=1)
        
    def callback(self,msg):
        
        with self.mutex:
            self.distances = [None for x in xrange(12)]
            for channel in msg.channels:
                if channel.name == "SensorId":
                    sensors  = list(channel.values)
                if channel.name == "Distance":
                    for i,sensor in enumerate(sensors):
                        self.distances[int(sensor)] = channel.values[i]
        rospy.sleep(0.2)
    
    def getRanges(self):
        with self.mutex:
            return self.distances
        
    def checkConnection(self):
        """
            Checks if baxter's sonars already subscribed to the publisher
        """
        while not rospy.is_shutdown() and self.__sonar_pub.get_num_connections() < 1:
           # rospy.logwarn("No subscriber for sonar state found yet")
            rospy.sleep(0.01)
        rospy.loginfo("Found a subscriber. Changing sonar state")
        
    def enable(self):
        """
            Enables all sonar sensors
        """
        rospy.loginfo("Enable sonar")
        self.checkConnection()
        self.state = 4095
        self.__sonar_pub.publish(4095)
        
    def disable(self):
        """
            Disables all sonar sensors
        """
        rospy.loginfo("Disable sonar")
        self.checkConnection()
        self.state = 0
        self.__sonar_pub.publish(0)


if __name__ == '__main__':        
    rospy.init_node("baxter_sonar")
