#!/usr/bin/env python

import rospy
import sys
from PySide import QtCore, QtGui
from baxter_gui.baxtergui import Ui_BaxterGUI 
import signal
from PySide.QtCore import QObject, Signal, Slot
from hr_helper.post_threading import Post


from sensor_msgs.msg import Image
from baxter_interface.camera import CameraController
import cv2
import cv_bridge
from PyQt4.Qt import QStringList
class CameraWrapper(QObject):
    updateCalled = Signal(QtGui.QImage)
    def __init__(self):
        QObject.__init__(self)
        self.cam=None
        self.sub=None
        self.save_image = False
        self.camera_name = None
        
    def open(self,camera_name):
        if not self.cam is None:
            self.cam.close()
        self.camera_name=camera_name
        self.cam = CameraController(camera_name)
        self.cam.resolution = (480,300)
        self.cam.open()
        self.sub = rospy.Subscriber("/cameras/%s/image"%camera_name,Image,self.imageCallback,queue_size=1)
        
    def close(self):
        if self.cam is None:
            return
        self.cam.close()
        self.sub.unregister()
        
    def imageCallback(self,data):
        if self.save_image:
            filename = self.camera_name+".jpg"
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
    def __init__(self,gui):
        self.post = Post(self)
        self.gui = gui
        self.ui = gui.ui
        self.lbl = None
        self.initCameraImage()
        self.cam = CameraWrapper()
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
from baxter_hr_interface.io.digital_io import DigitalIO
from baxter_hr_interface.io.navigator import Navigator
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
            
        
        self.ui.tbl_buttons.setColumnCount(3)
        self.ui.tbl_buttons.setRowCount(12)
        self.ui.tbl_buttons.setHorizontalHeaderLabels(["Buttons","Left State","Right State"])
        self.ui.tbl_buttons.horizontalHeader().setVisible(True)
        
        for i,item in enumerate(self.button_names):
            newItem = QtGui.QTableWidgetItem(item)
            self.ui.tbl_buttons.setItem(i, 0, newItem)
            newItem = QtGui.QTableWidgetItem("N/A")
            self.ui.tbl_buttons.setItem(i, 1, newItem)
            newItem = QtGui.QTableWidgetItem("N/A")
            self.ui.tbl_buttons.setItem(i, 2, newItem)
            
        
        self.button_id=[]
        for side in ["left","right"]:
            self.side = side
            self.ui.list_buttons.addItems([side+"_"+item for item in self.button_names])
            for id in digitalIOBtn(side):
                self.button_id.append(DigitalIO(id))
            for id in navigatorBtn(side):
                self.button_id.append(Navigator(id))
        
        
        for id in self.button_id:
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
        print args
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
                old = self.ui.tbl_buttons.item(i,1).text()
                if not old == "N/A" and not str(state) == old:
                    self.removeButtonFromList(side, button)
                self.ui.tbl_buttons.setItem(i, column, newItem)
    
            
    def removeButtonFromList(self,side,button):
        list_item=self.ui.list_buttons.findItems(side+"_"+button, QtCore.Qt.MatchRegExp)
        for item in list_item:
            self.ui.list_buttons.takeItem(self.ui.list_buttons.row(item))
    
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
    
from baxter_interface.robot_enable import RobotEnable
import subprocess
class TabGeneral(QObject):
    def __init__(self,gui):
        QObject.__init__(self)
        self.gui = gui
        self.ui = gui.ui
        self.robot_enable = RobotEnable()
        self.ui.btn_enable.clicked.connect(self.robot_enable.enable)
        self.ui.btn_disable.clicked.connect(self.robot_enable.disable)
        self.ui.btn_open_log_dir.clicked.connect(self.openLogDir)
        
    def openLogDir(self):
        subprocess.call(["nautilus",self.getPath("baxter_gui")])
        
        
    def getPath(self,package=None):
        import os
        if package is None:
            path=str(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))+"/log"
        else:
            import rospkg
            path = rospkg.RosPack().get_path(package) +"/log"
        if not os.path.exists(path):
            os.makedirs(path)
        return path

from baxter_hr_interface.sensors.sonar import Sonar
class TabSonar(QObject):

    def __init__(self,gui):
        QObject.__init__(self)
        self.gui = gui
        self.ui = gui.ui
        self.sonar = Sonar()
        self.ui.btn_sonar_enable.clicked.connect(self.sonar.enable)
        self.ui.btn_sonar_disable.clicked.connect(self.sonar.disable)
        
        for i in range(12):
            newItem = QtGui.QTableWidgetItem("Sensor "+str(i))
            self.ui.tbl_sonar.setItem(i, 0, newItem)
        
class ControlMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.ui =  Ui_BaxterGUI()
        self.ui.setupUi(self)
        self.camera = TabCamera(self)
        self.button = TabButton(self)
        self.infrared = TabInfrared(self)
        self.general = TabGeneral(self)
        self.sonar = TabSonar(self)

      
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
    