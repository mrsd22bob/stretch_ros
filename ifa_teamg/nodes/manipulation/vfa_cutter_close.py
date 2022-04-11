#!/usr/bin/python
from __future__ import print_function
import vfa_cutter as cutter
import time
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

g=cutter.VFACutter()
if not g.startup():
    exit()
g.close()
time.sleep(3.0)
g.stop()
