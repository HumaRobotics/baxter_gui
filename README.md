# baxter_gui

Install

* Install 3rd party dependencies:
```
sudo apt-get install pyside-tools
```
* Clone the repo into your source of your ROS workspace:
```
git clone https://github.com/HumaRobotics/baxter_evalutation_gui.git
```
* Do a catkin make in the root of your workspace
```
catkin_make
```
* Create the baxtergui.py. Go inside the source 
```
cd src/baxter_gui
./create_ui.sh
```
* Run the main script (preferably, when you are connected to a baxter)
```
rosrun baxter_gui main.py
```
