#!/usr/bin/python
from __future__ import print_function
import vfa_cutter as cutter
import vfa_gripper as gripper
import time
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

c=cutter.VFACutter()
g=gripper.VFAGripper()
if not g.startup():
    exit()
if not c.startup():
    exit()
print("Homing gripper")
g.home()
print("Homing cutter")
c.home()
time.sleep(0.5)
g.open()
c.open()
time.sleep(0.5)
g.stop()
c.stop()
