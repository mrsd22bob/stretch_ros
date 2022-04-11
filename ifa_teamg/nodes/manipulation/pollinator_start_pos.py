#!/usr/bin/python
from __future__ import print_function
from stretch_body.dynamixel_XL430 import *
import vfa_pollinator as pollinator
import time
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

g=pollinator.VFAPollinator()
if not g.startup():
    exit()
g.home()
print("moving to start position")
g.movetostartpos()
time.sleep(0.5)

#start vibration

g.movetopos2()
time.sleep(0.5)
g.movetopos3()
time.sleep(0.5)
g.movetozero()
#stop vibration
time.sleep(3.0)
#g.stop()
