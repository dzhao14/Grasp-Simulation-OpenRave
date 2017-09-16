"""This script runs openrave with a simple environment containing the pr2 robot and a mug on a table. The goal of this script is to have the data of the pointcloud that shines on the table be sent to Andre Te Pas's gpd library. Afterwards hopefully the robot will try to move its arms in to the grasp position given as output from gpd."""

"""In order to '~run...' this you must launch the gpd ROS node that waits for a pointcould. This should be done with (after sourcing the gpd library) roslaunch gpd tutorial1.launch"""

#!/usr/bin/env python

from openravepy import *
import numpy as np
import time
import rospy
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
from gpd.msg import GraspConfigList #source the catkin_ws

grasps = []

def callback(msg):
    global grasps
    grasps = msg.grasps

if __name__ == "__main__":
    env = Environment()
    env.Load('env.xml')
    time.sleep(1)
    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    robot = env.GetRobots()[0]

    #move the robot arms so they don't get block the flashlight
    basemanip = interfaces.BaseManipulation(robot)
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        basemanip.MoveActiveJoints(goal=[1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996])
    time.sleep(5)

    manip = robot.SetActiveManipulator('rightarm_torso')
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()

    sensors = env.GetSensors()
    flashlidar = sensors[0] 
    flashlidar.Configure(Sensor.ConfigureCommand.PowerOn)
    flashlidar.Configure(Sensor.ConfigureCommand.RenderDataOn)
    olddata = flashlidar.GetSensorData(Sensor.Type.Laser)

    #-- ros publisher stuff
    pub = rospy.Publisher('/detect_grasps', pc2.PointCloud2) 
    rospy.init_node('I_give_PC_info', anonymous = True)
    sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)
    

    while True:
        time.sleep(2)

        with env:
            for grasp in grasps:
                print 'attempting to grasp: '
                print repr(grasp)
                Tgoal = numpy.array([[grasp.approach.x, grasp.binormal.x, grasp.axis.x, grasp.bottom.x], [grasp.approach.y, grasp.binormal.y, grasp.axis.y, grasp.bottom.y], [grasp.approach.z, grasp.binormal.z, grasp.axis.z, grasp.bottom.z], [0,0,0,1]])
                sol = manip.FindIKSolution(Tgoal, IkFilterOptions.IgnoreJointLimits)
                with robot:
                    robot.SetDOFValues(sol, manip.GetArmIndices())
                    env.UpdatePublishedBodies()
                    time.sleep(5)

        #publish point cloud
        while True:
            data = flashlidar.GetSensorData(Sensor.Type.Laser)
            if data.stamp != olddata.stamp:
                break
            time.sleep(0.1)
        msg = pc2.create_cloud_xyz32(std_msgs.msg.Header(), data.ranges + data.positions[0])
        if not rospy.is_shutdown():
            #rospy.loginfo(msg)
            print 'Publishing pointcloud'
            pub.publish(msg)
        else:
            print 'NOT PUBLISHED'
        time.sleep(5)
