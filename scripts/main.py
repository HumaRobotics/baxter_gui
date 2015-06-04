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
class TabButton():
    def __init__(self,gui):
        self.post = Post(self)
        self.gui = gui
        self.ui = gui.ui
        self.rootName = "/robot/"
        self.buttons = {}
        side = "left"
        self.button_names=["shoulder",
                           "cuff",
                           "white oval",
                           "gray circle",
                           "ok",
                           "wheel",
                           "back",
                           "rethink",
                           "torso ok",
                           "torso wheel",
                           "torso back",
                           "torso rethink",
                           ]
        self.button_ids=[
                         "digital_io/%s_shoulder_button/state"%side,
                         "digital_io/%s_lower_cuff/state"%side,
                         "digital_io/%s_lower_button/state"%side,
                         "digital_io/%s_upper_button/state"%side,
                         "itb/%s_itb/state/button[0]"%side,
                         "itb/%s_itb/state/wheel"%side,
                         "itb/%s_itb/state/button[1]"%side,
                         "itb/%s_itb/state/button[2]"%side,
                         "itb/torso_%s_itb/state/button[0]"%side,
                         "itb/torso_%s_itb/state/wheel"%side,
                         "itb/torso_%s_itb/state/button[1]"%side,
                         "itb/torso_%s_itb/state/button[2]"%side,
                         ]
        self.button_states=[False for x in xrange(12)]
        
        
        self.ui.list_buttons.addItems([""])
        self.ui.tbl_buttons.setColumnCount(3)
        self.ui.tbl_buttons.setRowCount(12)
        self.ui.tbl_buttons.setHorizontalHeaderLabels(["Buttons","Left State","Right State"])
        self.ui.tbl_buttons.horizontalHeader().setVisible(True)
        i=0
        for item in self.button_names:
            newItem = QtGui.QTableWidgetItem(item)
            self.ui.tbl_buttons.setItem(i, 0, newItem)
            i+=1
            
        i=0     
        for id in self.button_ids:
            vars()[self.btnInfo[i][0]] = rospy.Subscriber(self.rootName + id,self.btnInfo[i][1],self.buttonCallback,[i,self.btnInfo[i][1]] )    
        
            
        
class ControlMainWindow(QtGui.QMainWindow):
#     cb_left_camera = Signal(int)
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.ui =  Ui_BaxterGUI()
        self.ui.setupUi(self)
#         self.camera = TabCamera(self)
        self.button = TabButton(self)
        
    def update(self):
        print "update"
   
 
      
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
    