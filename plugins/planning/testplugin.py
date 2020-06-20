#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/planning')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    torsoplanning_3D = RaveCreateModule(env,'torsoplanning_3D')
    print torsoplanning_3D.SendCommand('help')
finally:
    RaveDestroy()
