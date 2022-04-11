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
print("Gripping")
g.close()
print("Cutting")
c.close()
time.sleep(2.0)
print("Re-cutting")
c.open_slight()
time.sleep(2.0)
c.close()
time.sleep(2.0)
print("Releasing cutter")
c.open()
time.sleep(2.0)
g.stop()
c.stop()
