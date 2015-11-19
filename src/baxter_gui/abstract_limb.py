#!/usr/bin/env python
########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the Simplified BSD License on
# github: git@www.humarobotics.com:baxter_tasker
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

DEFAULT_RATE=200
DEFAULT_SPEED=1
DEFAULT_POSITION_TOLERANCE=0.01
DEFAULT_ORIENTATION_TOLERANCE=0.01
DEFAULT_JOINT_TOLERANCE_PLAN=0.01
DEFAULT_JOINT_TOLERANCE=0.008726646
DEFAULT_SPEED_TOLERANCE=0.03
DEFAULT_TIMEOUT=15
DEFAULT_PATH_TOLERANCE=0.35 # Default path tolerance for speed 1

from sensor_msgs.msg import JointState
import collections
from copy import deepcopy,copy

"""

    IO and Conversions

"""
import pickle
import json
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,  
)
# from trajectory_msgs.msg import (
#     JointTrajectoryPoint,
# )

import os
# The datapath where all data, that is created by this package should be stored
datapath=str(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  + "/data/")
print datapath
def setDatapath(new_path):
    global datapath
    import rospkg
    datapath = rospkg.RosPack().get_path(new_path) + "/data/"
    if not os.path.exists(datapath):
        os.makedirs(datapath)
    print "Datapath set to:",datapath

def getAnglesFromDict(d):# NOTE Fails currently if dict is None
    """
        Converts a dictionary to a angles of a JointState()
        
        :param d (dict): The dictionary to be converted
        :return (sensor_msgs.msg.JointState): The angles
    """
#     print datapath
    if d is None:
        rospy.logerr("Could not get angles for non existing dictionary")
        return None
    js=JointState()
    for n,v in d.items():
        js.name.append(n.encode())
        js.position.append(v)
    return js        

def getDictFromAngles(angles):
    """
        Converts angles of a JointState() to a dictionary
        :param angles (sensor_msgs.msg.JointState): angles to be converted
        :return (dict): dictionary with angles 
    """
    
    d=dict(zip(angles.name,angles.position))
    return collections.OrderedDict(sorted( d.items() ))
    
def getDictFromPose(pose):
    """
        Converts a Pose to a dictionary
        :param pose (geometry_msgs.msg.Pose): Pose to be converted
        :return (dict): dictionary with position and orientation
    """
    if type(pose)==PoseStamped:        
        pose=pose.pose
    d={}
    d["position"]=(pose.position.x,pose.position.y,pose.position.z)
    d["orientation"]=(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
    return d

def getStrFromPose(pose):
    """
        Converts a pose of a Pose() or PoseStamped() to a printable string
        :param pose (geometry_msgs.msg.Pose): Pose to be converted
        :return (string): string of the pose
    """
    frame_id="base"
    if type(pose)==PoseStamped:        
        frame_id=pose.header.frame_id
        pose=pose.pose
    d = getDictFromPose(pose)
    return "%s %s"%(frame_id,str(d))

def getStrFromAngles(angles):
    """
        Converts all angles of a JointState() to a printable string
        :param angles (sensor_msgs.msg.JointState): JointState() angles to be converted
        :return (string): string of the angles
    """
    d=getDictFromAngles(angles)
    return str( dict(d.items()))
       
def getPoseFromDict(d):
    """
        Converts a dictionary to a Pose
        :param d (dict): The dictionary to be converted
        :return (geometry_msgs.msg.Pose): The pose from the dictionary
    """
    if d is None:
        rospy.logerr("Could not get pose for non existing dictionary")
        return None
    pose=Pose()
    (pose.position.x,pose.position.y,pose.position.z)=d["position"]
    (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)=d["orientation"]
    return pose

def createDir(path):
    """
        Created a directory if it does not exist
        :param path (string): Path for the directories to be created, or verified
    """ 
    if not os.path.exists(path):
        os.makedirs(path)

def loadPose(name):
    """
        Loads a pose from a file located in the data directory of the package
        :param name (string): Filename of the pose to load
        :return (geometry_msgs.msg.Pose): The pose as Pose() that is going to be saved 
    """
    path = datapath + "pose"+os.sep
    return getPoseFromDict(json.loads(open(path+name+".pose","r").read()))    
    
def savePose(name,pose):
    """
        Saves a pose to a file located in the data directory of the package
        :param name (string): Filename of the pose to save
        :param pose (geometry_msgs.msg.Pose): The pose that is going to be saved 
    """
    path = datapath + "pose"+os.sep
    createDir(path)
    d=getDictFromPose(pose)
    open(path+name+".pose","w").write(json.dumps(d))
    
def loadAngles(name):
    """
        Loads angles from a file located in the data directory of the package
        :param name (string): Filename of the angles to load
        :return (sensor_msgs.msg.JointState): The angles as a JointState() that are going to be saved 
    """
    path = datapath + "angles"+os.sep
    return getAnglesFromDict(json.loads(open(path+name+".angles","r").read()))

def loadAnglesDict(name):
    """
        Loads angles from a file located in the data directory of the package
        :param name (string): Filename of the angles to load
        :return (dict): The angles as a dictionary
    """
    path = datapath + "angles"+os.sep
    return json.loads(open(path+name+".angles","r").read())

def saveAngles(name,angles):
    """
        Saves angles to a file located in the data directory of the package
        :param name (string): Filename of the angles to save
        :param angles (sensor_msgs.msg.JointState): The angles that are going to be saved 
    """
    path = datapath + "angles"+os.sep
    createDir(path)
    open(path+name+".angles","w").write(json.dumps(getDictFromAngles(angles)))

def loadPlan(name):
    """
        Loads a plan from a file located in the data directory of the package
        :param name (string): Filename of the plan to be loaded
        :return (moveit_msgs.msg.RobotTrajectory): Returns the plan or None otherwise
    """
    path = datapath + "plans"+os.sep
    try:  
        with open(path+name+'.plan', 'rb') as pkl_file:
            plan = pickle.load(pkl_file)
        rospy.loginfo("old plan found, called %s"%name)
        return plan
    except:
        rospy.loginfo("plan to target does not exist yet")        
        return None  
    
def savePlan(name,plan):  
    """
        Saves a plan to a file located in the data directory of the package. The directory is created, if it does not exist.
        :param name (string): Filename of the plan to save
        :param plan (moveit_msgs.msg.RobotTrajectory): The plan that is going to be saved 
    """
    path = datapath + "plans"+os.sep
    createDir(path)
    rospy.loginfo("planname to save: %s.plan"%name)      
    with open(path+name+'.plan', 'wb') as pkl_file:
        pickle.dump(plan, pkl_file)


def getReversePlan(plan):
    """
        Returns the reverse version of a given plan
        :param plan (moveit_msgs.msg.RobotTrajectory): The original plan
        :return (moveit_msgs.msg.RobotTrajectory): The reversed plan
    """ 
    z=deepcopy(plan)
    maxtime=deepcopy(plan.joint_trajectory.points[-1].time_from_start)
    z.joint_trajectory.points.reverse()
    for p in z.joint_trajectory.points:
        p.time_from_start=maxtime-deepcopy(p.time_from_start)
    return z
    


"""

    Comparisons

"""
def getAnglesDiff(joint_angles_1,joint_angles_2):
    """
        Returns a dictionary of the difference between two joint state angles
        :param joint_angles_1 (sensor_msgs.msg.JointState): first angles of type sensor_msgs.msg.JointState()
        :param joint_angles_2 (sensor_msgs.msg.JointState): second angles of type sensor_msgs.msg.JointState() 
        :return (dict): Dictionary of angle differences
    """
    da=getDictFromAngles(joint_angles_1)
    db=getDictFromAngles(joint_angles_2)
    dd=dict()
    for name in da.keys():
        dd[name]=da[name]-db[name]
    return dd

def getPoseDiff(pose_1,pose_2):
    """
        Returns a dictionary of the difference between two Poses
        :param pose_1 (geometry_msgs.msg.Pose): first pose of type geometry_msgs.msg.Pose()
        :param pose_2 (geometry_msgs.msg.Pose): second angles of type geometry_msgs.msg.Pose() 
        :return (dict): Dictionary of pose difference
    """
    da=getDictFromPose(pose_1)
    db=getDictFromPose(pose_2)
    dd=dict()
    dd["position"]=[]
    dd["orientation"]=[]
    for va,vb in zip(da["position"],db["position"]):
        dd["position"].append(va-vb)
    for va,vb in zip(da["orientation"],db["orientation"]):
        dd["orientation"].append(va-vb)
    return dd

def getAnglesError(joint_angles_1, joint_angles_2):
    """
        Returns the accumulated error from the difference of two joint states angles
        :param joint_angles_1 (sensor_msgs.msg.JointState): first angles of type sensor_msgs.msg.JointState()
        :param joint_angles_2 (sensor_msgs.msg.JointState): second angles of type sensor_msgs.msg.JointState() 
        :return (float): Accumulated error
    """
    da=getDictFromAngles(joint_angles_1)
    db=getDictFromAngles(joint_angles_2)
    err=0.0
    for name in da.keys():
        err+=math.fabs(da[name]-db[name])    
    return err
    
 



"""

    Abstract Limb Interface

"""

class AbstractLimb():

    def getPose(self):
        """ Returns the current cartesian pose as geometry_msgs/Pose
            :return (geometry_msgs.msg.Pose): Current pose
        """
        return Pose()
    
    def getAngles(self):
        """ Returns the current joint state as  sensor_msgs/JointState
            :return (sensor_msgs.msg.JointState): Current angles
        """
        return JointState()

    def goToPose(self,pose,speed=DEFAULT_SPEED,position_tolerance=DEFAULT_POSITION_TOLERANCE,orientation_tolerance=DEFAULT_ORIENTATION_TOLERANCE,joint_tolerance=DEFAULT_JOINT_TOLERANCE,speed_tolerance=DEFAULT_SPEED_TOLERANCE,timeout=DEFAULT_TIMEOUT,cartesian=False,path_tolerance=0):
        """ Go to a cartesian pose provided as geometry_msgs/Pose for the defined eef , blocking , returns True if all went fine (trajectory found), false otherwise
            :param pose (geometry_msgs.msg.Pose):  target position as geometry_msgs/Pose 
            :param speed (float): a float multiplicator of the default baxter speed
            :param position_tolerance (float): position tolerance in meters for the target pose (used in planning pipeline)
            :param orientation_tolerance (float): orientation tolerance in radians for the target pose (used in planning pipeline)
            :param joint_tolerance (float): joint tolerance in radians for the target joint angles (used for execution)
            :param speed_tolerance (float): speed tolerance in rad/s-1 for the controller to stop (used for execution)
            :param timeout (float): timeout in seconds before giving on the control
            :param cartesian (bool): boolean to compute a cartesian path
            :param path_tolerance (float): maximum joint angular distance during execution to consider a failure during trajectory execution
            :return (bool): Success flag
        """
        rospy.logerr("ERROR: Mock Function")
        return True

    def goToAngles(self,angles,speed=DEFAULT_SPEED,joint_tolerance_plan=DEFAULT_JOINT_TOLERANCE_PLAN,joint_tolerance=DEFAULT_JOINT_TOLERANCE,speed_tolerance=DEFAULT_SPEED_TOLERANCE,timeout=DEFAULT_TIMEOUT,path_tolerance=0):
        """ Go to a angular joint state provided as sensor_msgs/JointState, blocking, returns True if all went fine (trajectory found), false otherwise
            :param angles (sensor_msgs.msg.JointState):  target joint position as sensor_msgs/JointState
            :param speed (float): a float multiplicator of the default baxter speed
            :param joint_tolerance_plan (float): joint angle tolerance in radians target joint angles (used in planning pipeline)
            :param joint_tolerance (float): joint angle tolerance in radians for the target joint angles (used for execution)
            :param speed_tolerance (float): speed tolerance in rad/s-1 for the controller to stop (used for execution)
            :param timeout (float): timeout in seconds before giving on the control
            :param path_tolerance (float): maximum joint angular distance during execution to consider a failure during trajectory execution                    
            :return (bool): Success flag
        """        
        rospy.logerr("ERROR: Mock Function")
        return True
    
    def stop(self):
        """ Stops all future and the current motions """
        rospy.logerr("ERROR: Mock Function")
        
    def resume(self):
        """ Allows future motions"""
        rospy.logerr("ERROR: Mock Function")
        
    def cancel(self):
        """ Cancel the current motion"""
        rospy.logerr("ERROR: Mock Function")
        pass

    def moving(self):
        """ Returns whether the arm is currently moving
            :return (bool): Currently moving flag
        """
        rospy.logerr("ERROR: Mock Function")
        pass
    
    def getAnglesFromPose(self,pose):
        """ Calls inverse kinematics to obtain joint angles from a cartesian pose, return None if failed
            :param pose (geometry_msgs.msg.Pose):  target position as geometry_msgs/Pose         
            :return (sensor_msgs.msg.JointState): Joint angles
        """
        rospy.logerr("ERROR: Mock Function")
        return JointState()

    def getPoseFromAngles(self,angles):
        """ Calls forward kinematics to obtain cartesian pose from joint angles, return None if failed
            :param angles (sensor_msgs.msg.JointState):  target joint position as sensor_msgs/JointState
            :return (geometry_msgs.msg.Pose): Cartesian pose
        """
        rospy.logerr("ERROR: Mock Function")
        return Pose()
    
    
    def waitForMoving(self,timeout=0):
        """ Wait for motion to start or timeout expires, blocking, return True if motion started, False otherwise
            :param timeout (float):  timeout in seconds
            :return (bool): True is the robot is moving
        """
        rospy.logdebug("Waiting %s to start motion"%self.side)
        start=rospy.get_time()
        while not self.moving() and not rospy.is_shutdown():
            if timeout>0 and (rospy.get_time()-start)>timeout:
                break           
            rospy.sleep(0.01)
        return self.moving()
    
    def waitForStopped(self,timeout=0):
        """ Wait for motion to stop or timeout expires, blocking, return True if motion stopped, False otherwise 
            :param timeout (float):  timeout in seconds
            :return (bool): True if the robot is stopped
        """
        rospy.logdebug("Waiting %s to finish motion"%self.side)
        start=rospy.get_time()
        while self.moving() and not rospy.is_shutdown():
            if timeout>0 and (rospy.get_time()-start)>timeout:
                break
            rospy.sleep(0.01)
        return not self.moving()
    
    
