#!/usr/bin/python
from __future__ import print_function
import vfa_gripper as gripper
import time
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

g=gripper.VFAGripper()
if not g.startup():
    exit()
g.open()
time.sleep(3.0)
g.stop()
