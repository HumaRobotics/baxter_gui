#!/usr/bin/env python

import rospy
import sys
import os
from PySide import QtCore, QtGui
from baxter_gui.baxtergui import Ui_BaxterGUI 
import signal
from PySide.QtCore import QObject, Signal, Slot
from baxter_gui.post_threading import Post

from sensor_msgs.msg import Image
from baxter_gui.camera import HRCamera
import cv2
import cv_bridge
from PyQt4.Qt import QStringList
class CameraWrapper(QObject):
    updateCalled = Signal(QtGui.QImage)
    def __init__(self,datapath):
        QObject.__init__(self)
        self.sub=None
        self.save_image = False
        self.camera_name = None
        self.cam = HRCamera()
        self.datapath = datapath
        
    def open(self,camera_name):
        if self.camera_name == camera_name:
            return
        cur_cam = self.camera_name
        self.camera_name=camera_name
        self.cam.open(camera_name,(480,300))
        if not self.sub is None:
            self.sub.unregister()
        self.sub = rospy.Subscriber("/cameras/%s/image"%camera_name,Image,self.imageCallback,queue_size=1)
        if camera_name != cur_cam:
            self.close()   
        
    def close(self):
        print "trying to close ",self.camera_name
        if self.cam is None:
            return
        if self.camera_name is None:
            return
        self.cam.close(self.camera_name)
        #self.sub.unregister()
        
    def imageCallback(self,data):
        if self.save_image:
            filename = self.datapath + os.sep +"log" + self.camera_name + ".jpg"
            img = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite(filename,img)
            rospy.loginfo("Saved Image %s"%filename)
            self.save_image = False
        format = QtGui.QImage.Format_ARGB32
        try:
            self.updateCalled.emit(QtGui.QImage(data.data, data.width, data.height, format))
        except Exception,e:
            print e
 
class TabCamera():
    def __init__(self,gui,datapath):
        self.post = Post(self)
        self.gui = gui
        self.ui = gui.ui
        self.lbl = None
        self.initCameraImage()
        self.cam = CameraWrapper(datapath)
        self.cam.updateCalled.connect(self.updateImage)
        self.ui.cb_left_camera.clicked.connect(self.updateCamera)
        self.ui.cb_right_camera.clicked.connect(self.updateCamera)
        self.ui.cb_head_camera.clicked.connect(self.updateCamera)
        self.ui.btn_save_image.clicked.connect(self.saveImage)
        self.ui.btn_test_all_cameras.clicked.connect(self.testAllCameras)

    def saveImage(self):
        self.cam.save_image = True
        
    def testCam(self,name):
        self.cam.open(name)
        rospy.sleep(2)
        self.cam.save_image = True
        while self.cam.save_image:
            rospy.sleep(0.1)
            
    def testAllCameras(self):
        self.post._testAllCameras()

    def _testAllCameras(self):
        for name in ["left_hand_camera","right_hand_camera","head_camera"]:
            self.testCam(name)
        

    def updateCamera(self):
        if self.ui.cb_left_camera.isChecked():
            print "opening left cam"
            self.cam.open("left_hand_camera")
        elif self.ui.cb_right_camera.isChecked():
            print "opening right cam"
            self.cam.open("right_hand_camera")
        elif self.ui.cb_head_camera.isChecked():
            print "opening head cam"
            self.cam.open("head_camera")
            
    def initCameraImage(self):      
        hbox = QtGui.QHBoxLayout(self.ui.camera_image)
        self.lbl = QtGui.QLabel(self.gui)
        hbox.addWidget(self.lbl)
        self.gui.setLayout(hbox)
        
    @Slot(QtGui.QImage)
    def updateImage(self,image):
        if not self.lbl is None: 
            pixmap = QtGui.QPixmap.fromImage(image)
            self.lbl.setPixmap(pixmap)


from baxter_core_msgs.msg import *
from baxter_gui.digital_io import DigitalIO
from baxter_gui.navigator import Navigator
class TabButton():
    def __init__(self,gui):
        self.gui = gui
        self.ui = gui.ui
        #define user interface names
        self.button_names=["shoulder_button",
                           "lower_cuff",
                           "lower_button",
                           "upper_button",
                           "ok",
                           "wheel",
                           "back",
                           "rethink",
                           "torso_ok",
                           "torso_wheel",
                           "torso_back",
                           "torso_rethink",
                           ]
        #digital io buttons
        def digitalIOBtn(side):
            return [
                     "%s_shoulder_button"%side,
                     "%s_lower_cuff"%side,
                     "%s_lower_button"%side,
                     "%s_upper_button"%side,
                     ]
        #navigator buttons
        def navigatorBtn(side):
            return [
                     "%s"%side,
                     "torso_%s"%side,
                     ]      
            
        # Set header for table
        self.ui.tbl_buttons.setColumnCount(3)
        self.ui.tbl_buttons.setRowCount(12)
        self.ui.tbl_buttons.setHorizontalHeaderLabels(["Buttons","Left State","Right State"])
        self.ui.tbl_buttons.horizontalHeader().setVisible(True)
        
        # Fill table
        for i,item in enumerate(self.button_names):
            newItem = QtGui.QTableWidgetItem(item)
            self.ui.tbl_buttons.setItem(i, 0, newItem)
            newItem = QtGui.QTableWidgetItem("N/A")
            self.ui.tbl_buttons.setItem(i, 1, newItem)
            newItem = QtGui.QTableWidgetItem("N/A")
            self.ui.tbl_buttons.setItem(i, 2, newItem)
            
        
        
        # Create Object instances and list for buttons to push
        self.button_id=[]
        for side in ["left","right"]:
            self.side = side
            self.ui.list_buttons.addItems([side+"_"+item for item in self.button_names])
            if not self.gui.offline:
                for id in digitalIOBtn(side):
                    self.button_id.append(DigitalIO(id))
                for id in navigatorBtn(side):
                    self.button_id.append(Navigator(id))
        
        
        
        # Connect all buttons to a callback on button change
        if not self.gui.offline:
            for id in self.button_id:
                print id
                if type(id) == DigitalIO:
                    id.state_changed.connect(self.updateState)
                if type(id) == Navigator:
                    id.button0_changed.connect(self.updateState)
                    id.button1_changed.connect(self.updateState)
                    id.button2_changed.connect(self.updateState)
                    id.wheel_changed.connect(self.updateState)
            

#         self.updateState(True,"left_shoulder_button")
#         rospy.sleep(0.5)
#         self.updateState(False,"left_shoulder_button")
        
    def updateState(self,*args):
#         print args
        state = args[0]
        if len(args) > 2:
            button_name = args[1].split("_")
            if len(button_name) > 1:
                side = button_name[1]
                button_name = button_name[0]+"_"
            else:
                side = button_name[0]
                button_name = ""
                
            if args[2] == "button":
                id = args[3]
                if id == 0:
                    button_name+="ok"
                elif id==2:
                    button_name+="rethink"
                elif id==1:
                    button_name+="back"
            else:
                button_name+="wheel"
                state = args[3]
        else:
            button_name = args[1].split("_")
            side = button_name[0]
            button_name = "_".join(button_name[1:]) 

        newItem = QtGui.QTableWidgetItem(str(state))
        for i,button in enumerate(self.button_names):
            if button == button_name:
                if side == "left":
                    column = 1
                else:
                    column = 2
                old = self.ui.tbl_buttons.item(i,column).text()
#                 if not old == "N/A" and not str(state) == old:
                if old == "N/A":
                    self.removeButtonFromList(side, button)
                self.ui.tbl_buttons.setItem(i, column, newItem)
    
            
    def removeButtonFromList(self,side,button):
        try:
            print "side",side,"button",button
            list_item=self.ui.list_buttons.findItems(side+"_"+button, QtCore.Qt.MatchRegExp)
            for item in list_item:
                self.ui.list_buttons.takeItem(self.ui.list_buttons.row(item))
        except Exception,e:
            print e
    
from sensor_msgs.msg import Range        
class TabInfrared(QObject):
    updateCalled = Signal(float,str)
    def __init__(self,gui):
        QObject.__init__(self)
        self.gui = gui
        self.ui = gui.ui
        self.sub = {}
        self.buff_size = 9
        self.left_index = 0
        self.right_index = 0
        self.left_buffer = [0.0 for i in xrange(self.buff_size)]
        self.right_buffer = [0.0 for i in xrange(self.buff_size)]
                       
        for side in ["left","right"]:
            self.sub[side] = rospy.Subscriber("/robot/range/"+side+"_hand_range/state",Range,self.__sensorCallback,callback_args=side,queue_size=1)          
        self.updateCalled.connect(self.updateGUI)
               
        
    def __sensorCallback(self,msg,side):
        try:
            range = self.updateBuffer(min(msg.range*40/0.39,40), side)
            self.updateCalled.emit(range,side);
        except:
            pass
        
    @Slot(float,str)
    def updateGUI(self,range,side):
        if side == "left":
            self.ui.left_ir_range.setValue(range)
        else:
            self.ui.right_ir_range.setValue(range)
            
            
    def updateBuffer(self,new_value,side):
        if side == "left":
            self.left_buffer[self.left_index] = new_value
            self.left_index = (self.left_index+1)%self.buff_size
            return sorted(self.left_buffer)[self.buff_size/2+1]
        else:
            self.right_buffer[self.right_index] = new_value
            self.right_index = (self.right_index+1)%self.buff_size
            return sorted(self.right_buffer)[self.buff_size/2+1]
    
    
    
    

from baxter_gui.led import Led , HaloLed
class TabLed():
    def __init__(self,gui):
        self.ui = gui.ui
        self.led = Led()    
        self.halo = HaloLed()
        self.blinking = False
        self.ui.cb_led_halo_green.clicked.connect(self.updateHalo)
        self.ui.cb_led_halo_red.clicked.connect(self.updateHalo)
        self.ui.btn_blink_all.clicked.connect(self.blinkLeds)

        for name in self.led.led_names:
            func = getattr(self.ui, "cb_"+name)
            func.clicked.connect(lambda name=name, func=func: self.toggleLed(name,func))
        
    def blinkLeds(self):
        if self.blinking:
            self.led.disableAll()
            self.blinking = False
        else:
            self.blinking = True
            self.led.blinkAllInner()
            self.led.blinkAllOuter()
        
    def toggleLed(self,name,func):
        if func.isChecked():
            self.led.enable(name)
        else:
            self.led.disable(name)
        
        
    def updateHalo(self):
        self.halo.reset()
        if self.ui.cb_led_halo_green.isChecked() and self.ui.cb_led_halo_red.isChecked():
            self.halo.setOrange()
            return
        if self.ui.cb_led_halo_red.isChecked():
            self.halo.setRed()
            return
        if self.ui.cb_led_halo_green.isChecked():
            self.halo.setGreen()
            return

import math
import baxter_interface
class TabHead():
    def __init__(self,gui):
        self.ui = gui.ui
        self.post = Post(self)
        self.head = baxter_interface.Head()
        self.ui.btn_head_test.clicked.connect(self.__headTest)
    
    def __headTest(self):
        self.post.headTest()
        
    def headTest(self):
        self.head.set_pan(0.0, 1)
        self.head.set_pan(math.pi/2, 1)
        self.head.set_pan(-math.pi/2, 1)
        self.head.set_pan(0.0, 1)
        for i in range(3):
            self.head.command_nod()
        
from baxter_gui.simple_limb import SimpleLimb
class TabArms():
    def __init__(self,gui):
        self.ui = gui.ui
        self.post = Post(self)
        self.arm = {"right": SimpleLimb('right'),
                    "left":SimpleLimb('left')
                    }
        self.ui.btn_arms_test.clicked.connect(self.__armTest)
        
    def __armTest(self):
        self.post.armTest()
        
    def armTest(self):
        joint_moves = (
                [ 0.0, -0.55, 0.0, 0.75, 0.0, 1.26,  0.0],
                [ 0.5,  -0.8, 2.8, 0.15, 0.0,  1.9,  2.8],
                [-0.1,  -0.8,-1.0, 2.5,  0.0, -1.4, -2.8],
                [ 0.0, -0.55, 0.0, 0.75, 0.0, 1.26,  0.0],
                )
        
        
        for move in joint_moves:# todo try to use the basic interface
            side = "left"
            th_left = self.arm[side].post.move_to_joint_positions(dict(zip(self.arm[side].joint_names(),move)))
            side = "right"    
            self.arm[side].move_to_joint_positions(dict(zip(self.arm[side].joint_names(),move)))
            th_left.join()
            
   
   
import numpy as np
class TabDisplay():
    def __init__(self,gui):
        self.ui = gui.ui
        self.post = Post(self)
        self._display_pub= rospy.Publisher('/robot/xdisplay',Image, queue_size=1)
        self.ui.cb_display_white.clicked.connect(lambda: self.setImage("white"))
        self.ui.cb_display_black.clicked.connect(lambda: self.setImage("black"))
        self.ui.cb_display_blue.clicked.connect(lambda: self.setImage("blue"))
        self.ui.cb_display_green.clicked.connect(lambda: self.setImage("green"))
        self.ui.cb_display_red.clicked.connect(lambda: self.setImage("red"))
        self.ui.btn_display_show_all.clicked.connect(self.__showAll)
        
    def __showAll(self):
        self.post.showAll()
        
    def showAll(self):
        for color in ["white","black","blue","green","red"]:
            print "setting color",color
            self.setImage(color)
            rospy.sleep(10)
        print "done"
                    
    def setImage(self,color):
        blank_image = np.zeros((600,1024,3), np.uint8)
        if color == "black":
            pass
        elif color == "white":
            blank_image[:,:] = (255,255,255)
        elif color == "blue":
            blank_image[:,:] = (255,0,0)
        elif color == "green":
            blank_image[:,:] = (0,255,0)
        elif color == "red":
            blank_image[:,:] = (0,0,255)
        data = cv_bridge.CvBridge().cv2_to_imgmsg(blank_image,encoding="bgr8")
        self._display_pub.publish(data)
    
from baxter_interface import gripper
from baxter_core_msgs.msg import (    
    EndEffectorCommand,
    )
class TabGripper():
    def __init__(self,gui):
        self.gripper = {"left":gripper.Gripper("left"),
                         "right":gripper.Gripper("right")}

        self.ui = gui.ui
        self.ui.btn_gripper_left_calibrate.clicked.connect(lambda: self.gripper["left"].calibrate(False))
        self.ui.btn_gripper_right_calibrate.clicked.connect(lambda: self.gripper["right"].calibrate(False))
        self.ui.btn_gripper_left_open.clicked.connect(lambda: self.gripper["left"].open(False))
        self.ui.btn_gripper_left_close.clicked.connect(lambda: self.gripper["left"].close(False))
        self.ui.btn_gripper_right_open.clicked.connect(lambda: self.gripper["right"].open(False))
        self.ui.btn_gripper_right_close.clicked.connect(lambda: self.gripper["right"].close(False))
        self.ui.btn_gripper_left_test.clicked.connect(lambda: self.evaluateGripper("left"))
        self.ui.btn_gripper_right_test.clicked.connect(lambda: self.evaluateGripper("right"))
    
    def evaluateGripper(self,side):
        for i in range(5):
            self.gripper[side].open()
            rospy.sleep(1)
            self.gripper[side].close()
            rospy.sleep(1)
    
from baxter_interface.robot_enable import RobotEnable
from baxter_core_msgs.msg import AssemblyState
from baxter_gui.tuck_arms import Tucker
import subprocess
class TabGeneral(QObject):
    def __init__(self,gui):
        QObject.__init__(self)
        self.gui = gui
        self.ui = gui.ui
        
        if not self.gui.offline:
            self.robot_enable = RobotEnable()
            self.ui.btn_enable.clicked.connect(self.robot_enable.enable)
            self.ui.btn_disable.clicked.connect(self.robot_enable.disable)
            self.tucker = Tucker()
            if self.tucker.isTucked():
                self.ui.btn_enable.setEnabled(False)
            else:
                self.ui.btn_enable.setEnabled(True)
            self.ui.btn_untuck.clicked.connect(self.untuck)
            self.ui.btn_tuck.clicked.connect(self.tuck)
        self.ui.btn_open_log_dir.clicked.connect(self.openLogDir)
        
    def untuck(self):
        self.tucker.untuck()
        self.ui.btn_enable.setEnabled(True)
        
    def tuck(self):
        self.tucker.tuck()
        self.ui.btn_enable.setEnabled(False)
    
    def openLogDir(self):
        subprocess.call(["nautilus",self.getPath("baxter_gui")+os.sep+"log"])
        
        
    def getPath(self,package=None):
        import os
        if package is None:
            path=str(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        else:
            import rospkg
            path = rospkg.RosPack().get_path(package)
        if not os.path.exists(path):
            os.makedirs(path)
        return path
    

from baxter_gui.sonar import Sonar
class TabSonar(QObject):
    updateDistances = Signal(list)
    def __init__(self,gui):
        QObject.__init__(self)
        self.sonar = Sonar()
        self.gui = gui
        self.ui = gui.ui
        self.post = Post(self)
        self.ui.btn_sonar_enable.clicked.connect(self.sonar.enable)
        self.ui.btn_sonar_disable.clicked.connect(self.sonar.disable)
        self.updateDistances.connect(self.updateGui)
        self.ui.btn_sonar_reset.clicked.connect(self.reset)
        
        for i in range(12):
            newItem = QtGui.QTableWidgetItem("Sensor "+str(i))
            self.ui.tbl_sonar.setItem(i, 0, newItem)
        self.reset()
        
        self.post.callback()
            
    def callback(self):

        while not rospy.is_shutdown():
            self.updateDistances.emit(self.sonar.getRanges())
            rospy.sleep(0.1)
     
    def reset(self):
        for i in range(12):
            newItem = QtGui.QTableWidgetItem("None")
            self.ui.tbl_sonar.setItem(i, 1, newItem)
        
    def updateGui(self,distances):
        for i,distance in enumerate(distances):
            if distance is None:
                continue
            newItem = QtGui.QTableWidgetItem(str(distance))
            self.ui.tbl_sonar.setItem(i, 1, newItem)
        
class ControlMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.ui =  Ui_BaxterGUI()
        self.ui.setupUi(self)
        self.offline = True
        try:
            rospy.wait_for_message("/robot/state",AssemblyState,2.0)
            self.offline = False
        except rospy.exceptions.ROSException:
            self.offline = True
            
        self.general = TabGeneral(self)
        datapath = self.general.getPath("baxter_gui") + os.sep
        self.button = TabButton(self)
        if not self.offline:
            self.camera = TabCamera(self,datapath)
            self.infrared = TabInfrared(self)
            self.sonar = TabSonar(self)
            self.gripper = TabGripper(self)
            self.led = TabLed(self)
            self.display = TabDisplay(self)
            self.head = TabHead(self)
            self.arms = TabArms(self)
            
    def keyPressEvent(self, e):
        #closes the window on escape
        if e.key() == QtCore.Qt.Key_Escape:
            self.close()
            
        
        
      
class Application(QtGui.QApplication):
    """used to be able to terminate the qt window by ctrl+c"""
    def event(self, e):    
        return QtGui.QApplication.event(self, e)

        
    

    
if __name__ == "__main__":
    rospy.init_node("BaxterGUI")
    app = Application(sys.argv)
    signal.signal(signal.SIGINT, lambda *a: app.quit())
    app.startTimer(200)
    gui = ControlMainWindow()
    gui.show()
    sys.exit(app.exec_())
    