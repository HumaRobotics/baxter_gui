#!/usr/bin/env python

from PySide.QtCore import QObject, Signal, Slot

@Slot(int)
def say_punched(i):
    ''' Give evidence that a bag was punched. '''
    print('Bag was punched %d'%i)

class PunchingBag(QObject):
    
    ''' Represents a punching bag; when you punch it, it
        emits a signal that indicates that it was punched. '''
 
    punched = Signal(int)
    def __init__(self):
        # Initialize the PunchingBag as a QObject
        QObject.__init__(self)
 
    def punch(self,i):
        ''' Punch the bag '''
        self.punched.emit(i)
        
bag = PunchingBag()
# Connect the bag's punched signal to the say_punched slot
bag.punched.connect(say_punched)

# Punch the bag 10 times
for i in range(10):
    bag.punch(i)
exit()