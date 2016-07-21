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
from baxter_gui.post_threading import Post
import baxter_interface
from std_msgs.msg import Float32

class HaloLed():
    def __init__(self):
        self.pub = {}
        self.pub["red"] = rospy.Publisher("/robot/sonar/head_sonar/lights/set_red_level",Float32,queue_size = 1)
        self.pub["green"] = rospy.Publisher("/robot/sonar/head_sonar/lights/set_green_level",Float32,queue_size = 1)
        self.post = Post(self) 

    def setLed(self,color, intensity):
        """
            Set an intensity value for the halo led
            
            :param color: Color of the led (red, green) 
            :type color: str
            :param intensity: Float value for the intensity between 0.0 and 100.0 
            :type intensity: float
        """
        try:
            self.pub[color].publish(Float32(intensity))
        except Exception,e:
            rospy.logwarn("%s",str(e))
            
    def setGreen(self,timeout = 0.0):
        self.setLed("green",100)
        self.setLed("red",0)
        if timeout > 0.0 :
            self.post.reset(timeout)
        
    def setRed(self,timeout = 0.0):
        self.setLed("red",100)
        self.setLed("green",0)
        if timeout > 0.0 :
            self.post.reset(timeout)
        
    def setOrange(self,timeout = 0.0):
        self.setLed("green",100)
        self.setLed("red",100)
        if timeout > 0.0 :
            self.post.reset(timeout)
        
    def reset(self,timeout = 0.0):
        rospy.sleep(timeout)
        self.setLed("green",0)
        self.setLed("red",0)
    
    def blink(self,color, timeout=0.0, frequency=2.0, blocking=True): #timeout <= 0 blinks endlessly
        """
            Blinks a led for a specific time and frequency (blocking)
            
            :param color: Color of the led (red, green, orange) 
            :type color: str
            :param timeout: Duration to let the led blinking. A value <= 0.0 means infinite time
            :type timeout: float
            :param frequency: Rate, how often a led blinks in one second
            :type frequency: float   
        """
        if blocking is False:
            self.post.blink(color,timeout,frequency,True)
        else :
            end = rospy.Time.now()+ rospy.Duration(timeout)
            self.led_state[name] = 1
            while not rospy.is_shutdown():
                start = rospy.Time.now()
                if (start > end and timeout > 0) or self.led_state[name] == 0:
                    self.setLed(color, 0)
                    break
                self.setLed(color, 100)
                rospy.Rate(frequency*2).sleep()
                self.setLed(color, 0)
                rospy.Rate(frequency*2).sleep()
        

class Led(): 
    def __init__(self):
        """
            Class to control all leds
            Available leds are:
            * 'left_outer_light',
            * 'left_inner_light',
            * 'right_outer_light',
            * 'right_inner_light',
            * 'torso_left_outer_light',
            * 'torso_left_inner_light',
            * 'torso_right_outer_light',
            * 'torso_right_inner_light'
        """
        self.post = Post(self) 
        self.led_names = ['left_outer_light','left_inner_light', 
                          'right_outer_light', 'right_inner_light',
                          'torso_left_outer_light','torso_left_inner_light',
                          'torso_right_outer_light','torso_right_inner_light']
        self.led_handle = {}
        self.led_state = {}
        try:
            for led in self.led_names:
                self.led_handle[led] = baxter_interface.DigitalIO(led)
                self.led_state[led] = 0
        except:
            rospy.logwarn("Leds not initialized.")
    
    def enable(self,name):
        """
            Enables a Led
            
            :param name: Name of the led 
            :type name: str
        """
        self.led_handle[name].set_output(True)
        self.led_state[name] = 1
    
    def disable(self,name):   
        """
            Disables a Led
            
            :param name: Name of the led
            :type name: str
        """ 
        self.led_handle[name].set_output(False)
        self.led_state[name] = 0
        pass
    
    def disableAll(self):
        """
            Disables all leds
        """
        for led in self.led_names:
            self.disable(led)
        
    
    def blink(self,name, timeout=0.0, frequency=2.0): #timeout <= 0 blinks endlessly
        """
            Blinks a led for a specific time and frequency (blocking)
            
            :param name: Name of the led
            :type name: str
            :param timeout: Duration to let the led blinking. A value <= 0.0 means infinite time
            :type timeout: float
            :param frequency: Rate, how often a led blinks in one second
            :type frequency: float   
        """
        end = rospy.Time.now()+ rospy.Duration(timeout)
        self.led_state[name] = 1
        try:
            while not rospy.is_shutdown():
                start = rospy.Time.now()
                if (start > end and timeout > 0) or self.led_state[name] == 0:
                    self.led_handle[name].set_output(False)
                    break
                self.led_handle[name].set_output(True)
                rospy.Rate(frequency*2).sleep()
                self.led_handle[name].set_output(False)
                rospy.Rate(frequency*2).sleep()
        except:
            pass
                                              
            
    def blinkAllOuter(self,timeout=0,frequency=2):
        """
            Blinks with all blue leds for a specific time and frequency (blocking)
            
            :param timeout: Duration to let the leds blinking. A value <= 0.0 means infinite time
            :type timeout: float
            :param frequency: Rate, how often all leds blink in one second
            :type frequency: float
        """
        for led in xrange(0,len(self.led_names),2):
            self.post.blink(self.led_names[led],timeout,frequency)
        
    def blinkAllInner(self,timeout=0,frequency=2):
        """
            Blinks with all white leds for a specific time and frequency (blocking)
            
            :param timeout: Duration to let the leds blinking. A value <= 0.0 means infinite time
            :type timeout: float
            :param frequency: Rate, how often all leds blink in one second  
            :type frequency: float 
        """
        for led in xrange(1,len(self.led_names),2):
            self.post.blink(self.led_names[led],timeout,frequency)   
            
if __name__=="__main__":    
    rospy.init_node("testLeds")   
    halo = HaloLed()
    rospy.sleep(2)
    halo.setRed()
    print "test red"
    rospy.sleep(2)
    halo.setGreen()
    print "test green"
    rospy.sleep(2)
    halo.reset()