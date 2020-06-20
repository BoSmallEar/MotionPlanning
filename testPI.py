#!/usr/bin/env python
import openravepy
import time
import IPython as Ip
from environment_handler_2 import *  


if  not __openravepy_build_doc__:
    from openravepy import *
if __name__ == "__main__":
    RaveInitialize()
    RaveLoadPlugin('plugins/planning/build/planning')
    try:
        env=Environment()
        env.SetViewer('qtcoin') 
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)
        env.Reset()
        env.Load('scenes/border.env.xml')
        time.sleep(0.1)
        robot = env.GetRobots()[0] 
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.RotationQuat) 
        robot.SetActiveDOFValues([-0.9,0,1,1,0,0,0]) 
        handler = environment_handler(env) 
    	handler.update_environment(escher = None ,surface_source='large stair')  
        prob = RaveCreateModule(env,'torsoplanning_3D')
        robot.SetVisible(0)
        env.AddModule(prob,args='')  
        with env:
            cmdout = prob.SendCommand('findpath 0 2.7 0 1 1 0 0 0')
            print cmdout
            
    finally:
        RaveDestroy()