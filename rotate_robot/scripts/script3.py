#!/usr/bin/env python
import rospy
import matlab.engine
import time
eng = matlab.engine.start_matlab()
eng.simplescript2(nargout=0)
green = eng.workspace['p']
yellow = eng.workspace['q']
red = eng.workspace['r']
time.sleep(50)
print green
print yellow
print red
eng.quit()

