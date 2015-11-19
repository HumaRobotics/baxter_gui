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
from baxter_core_msgs.msg import (
    DigitalIOState,
)
import baxter_interface

class DigitalIO(baxter_interface.digital_io.DigitalIO):
    """
    Modified Interface class for a simple Digital Input and/or Output on the
    Baxter robot, which gives more feedback on a changed state
    """
    
    def __init__(self, component_id):
        baxter_interface.digital_io.DigitalIO.__init__(self,component_id)

    def _on_io_state(self, msg):
        """
        Updates the internally stored state of the Digital Input/Output.
        """
        new_state = (msg.state == DigitalIOState.PRESSED)
        if self._state is None:
            self._is_output = not msg.isInputOnly
        old_state = self._state
        self._state = new_state

        # trigger signal if changed
        if old_state is not None and old_state != new_state:
            self.state_changed(new_state,self._id)
