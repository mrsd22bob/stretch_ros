#!/usr/bin/python
from __future__ import print_function
import poll as pollinator
import time
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

g=pollinator.VFAPollinator()
if not g.startup():
    exit()
g.home()
time.sleep(3.0)
g.stop()
