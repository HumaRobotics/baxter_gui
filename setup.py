#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['baxter_gui']
d['package_dir'] = {'': 'src'}

#import os
#os.system("pyside-uic ./src/ui_qt/baxtergui.ui -o ./src/ui_qt/baxtergui.py")
setup(**d)
#pyside-uic BaxterGUI.ui -o BaxterGUI.py
