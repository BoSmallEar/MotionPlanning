# Python interface for the C++ Escher motion planning module
import openravepy as rave

import time
import sys
import numpy as np
import IPython as ip
import scipy
import random
import csv

def load_robot(env, urdf_path=None, srdf_path=None):
    if(not urdf_path):
        urdf_path = urdf

    if(not srdf_path):
        srdf_path = srdf


    rave.RaveLoadPlugin('../or_urdf/build/devel/lib/openrave-0.9/or_urdf_plugin')
    module = rave.RaveCreateModule(env, 'urdf')
 
    robot_name = module.SendCommand('load {} {}'.format(urdf_path, srdf_path))
    robot = env.GetRobot(robot_name)

    robot.GetManipulator('l_arm').SetLocalToolDirection(np.array([1, 0, 0]))
    robot.GetManipulator('l_arm').SetLocalToolTransform(np.array([
        [0,  1, 0, 0.086],
        [ -1, 0, 0, -0.03],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1]])
    )

    robot.GetManipulator('r_arm').SetLocalToolDirection(np.array([1, 0, 0]))
    robot.GetManipulator('r_arm').SetLocalToolTransform(np.array([
        [ 0,  -1, 0, 0.086],
        [ 1,  0, 0, 0.03],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1]])
    )

    robot.GetManipulator('l_leg').SetLocalToolDirection(np.array([0, 0, -1]))
    robot.GetManipulator('r_leg').SetLocalToolDirection(np.array([0, 0, -1]))

    return robot
 

def main():
    env = rave.Environment()
    env.SetViewer('qtcoin')
    env.Reset()

    #tune the torso size
    # env.Load('scenes/border.env.xml')
    # time.sleep(0.1)
    # robot = env.GetRobot('torso')
    # robot.SetActiveDOFs([],rave.DOFAffine.X|rave.DOFAffine.Y|rave.DOFAffine.Z|rave.DOFAffine.RotationQuat) 
    # robot.SetActiveDOFValues([0,0,1.3,1,0,0,0]) 

    ## load the Escher robot 
    urdf = 'file://escher_model/escher_cpp.urdf'
    srdf = 'file://escher_model/escher_cpp.srdf'

    robot = load_robot(env, urdf_path=urdf, srdf_path=srdf)
 

    #see all available manipulators

    manips = robot.GetManipulators()

    for manip in manips:
        print manip.GetName()
        
    [lower_limit,upper_limit] = robot.GetDOFLimits() 

    file = open('l_arm.txt','w')
    l_arm = robot.GetManipulator('l_arm')
    l_arm_indices = l_arm.GetArmIndices()
    robot.SetActiveDOFs(l_arm_indices)
    #sample 10000 points 
    for k in range(1000):
        l_arm_sample = []
        for i in l_arm_indices:
            l_arm_sample.append(random.uniform(lower_limit[i],upper_limit[i])) 
        robot.SetActiveDOFValues(l_arm_sample)
        sample_tranform = l_arm.GetTransform() 
        sample_tranform[2][3] = sample_tranform[2][3]-1.3
        trans = np.array([sample_tranform[0][3],sample_tranform[1][3],sample_tranform[2][3]])
        normal = np.array([sample_tranform[0][0],sample_tranform[1][0],sample_tranform[2][0]])
        prod = np.dot(trans,normal)
        if prod>0:
            sample_mat =  rave.poseFromMatrix(sample_tranform)  
            for i in range(7):
                file.write(str(sample_mat[i])+' ')
            file.write('\n')
    file.close()


    file = open('r_arm.txt','w')
    r_arm = robot.GetManipulator('r_arm') 
    r_arm_indices = r_arm.GetArmIndices()
    robot.SetActiveDOFs(r_arm_indices) 
    r_arm_samples = []
    for k in range(1000):
        r_arm_sample = []
        for i in r_arm_indices:
            r_arm_sample.append(random.uniform(lower_limit[i],upper_limit[i])) 
        robot.SetActiveDOFValues(r_arm_sample)
        sample_tranform = r_arm.GetTransform()
        sample_tranform[2][3] = sample_tranform[2][3]-1.3
        trans = np.array([sample_tranform[0][3],sample_tranform[1][3],sample_tranform[2][3]])
        normal = np.array([sample_tranform[0][0],sample_tranform[1][0],sample_tranform[2][0]])
        prod = np.dot(trans,normal)
        if prod>0:
            sample_mat =  rave.poseFromMatrix(sample_tranform)  
            for i in range(7):
                file.write(str(sample_mat[i])+' ')
            file.write('\n')
    file.close()

    file = open('l_leg.txt','w')
    l_leg = robot.GetManipulator('l_leg')
    l_leg_indices = l_leg.GetArmIndices()
    robot.SetActiveDOFs(l_leg_indices)
    #sample 10000 points
    l_leg_samples = []
    for k in range(1000):
        l_leg_sample = []
        for i in l_leg_indices:
            l_leg_sample.append(random.uniform(lower_limit[i],upper_limit[i])) 
        robot.SetActiveDOFValues(l_leg_sample) 
        sample_tranform = l_leg.GetTransform()
        sample_tranform[2][3] = sample_tranform[2][3]-1.3
        trans = np.array([sample_tranform[0][3],sample_tranform[1][3],sample_tranform[2][3]])
        normal = np.array([-sample_tranform[0][2],-sample_tranform[1][2],-sample_tranform[2][2]])
        prod = np.dot(trans,normal)
        if prod>0:
            sample_mat =  rave.poseFromMatrix(sample_tranform)  
            for i in range(7):
                file.write(str(sample_mat[i])+' ')
            file.write('\n')
    file.close()


    file = open('r_leg.txt','w')
    r_leg = robot.GetManipulator('r_leg')
    r_leg_indices = r_leg.GetArmIndices()
    robot.SetActiveDOFs(r_leg_indices) 
    r_leg_samples = []
    for k in range(1000):
        r_leg_sample = []
        for i in r_leg_indices:
            r_leg_sample.append(random.uniform(lower_limit[i],upper_limit[i])) 
        robot.SetActiveDOFValues(r_leg_sample)
        sample_tranform = r_leg.GetTransform()
        sample_mat =  rave.poseFromMatrix(sample_tranform) 
        sample_mat[6] = sample_mat[6]-1.3
        for i in range(7):
            file.write(str(sample_mat[i])+' ')
        file.write('\n')
    file.close()
 
    return



if __name__ == "__main__":
    main()

