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

import baxter_interface
class Navigator(baxter_interface.navigator.Navigator):
    """
    Modified Interface class for a Navigator on the Baxter robot, which gives more feedback on button change
    """
    def __init__(self, location):
        baxter_interface.navigator.Navigator.__init__(self,location)      

    def _on_state(self, msg):
        if not self._state:
            self._state = msg

        if self._state == msg:
            return

        old_state = self._state
        self._state = msg

        buttons = [self.button0_changed,
                   self.button1_changed,
                   self.button2_changed
                   ]
        for i, signal in enumerate(buttons):
            if old_state.buttons[i] != msg.buttons[i]:
                signal(msg.buttons[i],self._id,"button",i)

        if old_state.wheel != msg.wheel:
            diff = msg.wheel - old_state.wheel
            if abs(diff % 256) < 127:
                self.wheel_changed(diff % 256,self._id,"wheel",msg.wheel)
            else:
                self.wheel_changed(diff % (-256),self._id,"wheel",msg.wheel)
