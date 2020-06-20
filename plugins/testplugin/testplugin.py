#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/testplugin')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')

finally:
    RaveDestroy()
