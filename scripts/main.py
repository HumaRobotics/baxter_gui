#!/usr/bin/env python

import rospy
import sys
from PySide import QtCore, QtGui
from baxter_gui.baxtergui import Ui_BaxterGUI 
import signal
from PySide.QtCore import QObject, Signal, Slot



from sensor_msgs.msg import Image
from baxter_interface.camera import CameraController

    
class CameraWrapper(QObject):
    updateCalled = Signal(QtGui.QImage)
    def __init__(self):
        QObject.__init__(self)
        self.cam=None
        self.sub=None
        
    def open(self,camera_name):
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
        format = QtGui.QImage.Format_ARGB32
        try:
            self.updateCalled.emit(QtGui.QImage(data.data, data.width, data.height, format))
        except Exception,e:
            print e
 

class ControlMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.ui =  Ui_BaxterGUI()
        self.ui.setupUi(self)
        self.lbl = None
        self.initCameraImage()

    @Slot(QtGui.QImage)
    def updateImage(self,image):
        if not self.lbl is None: 
            pixmap = QtGui.QPixmap.fromImage(image)
            self.lbl.setPixmap(pixmap)
    
    def initCameraImage(self):      
        hbox = QtGui.QHBoxLayout(self.ui.camera_image)
        self.lbl = QtGui.QLabel(self)
        hbox.addWidget(self.lbl)
        self.setLayout(hbox)
      
class Application(QtGui.QApplication):
    """used to be able to terminate the qt window by ctrl+c"""
    def event(self, e):
        return QtGui.QApplication.event(self, e)

        
    
if __name__ == "__main__":
    rospy.init_node("BaxterGUI")
    cam = CameraWrapper()
    cam.open("left_hand_camera")
    app = Application(sys.argv)
    signal.signal(signal.SIGINT, lambda *a: app.quit())
    app.startTimer(200)
    gui = ControlMainWindow()
    cam.updateCalled.connect(gui.updateImage)
    gui.show()
#     cam.close()
#     rospy.sleep(2)
#     cam.open("right_hand_camera")
    sys.exit(app.exec_())
    