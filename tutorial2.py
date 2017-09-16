from openravepy import *
import numpy as np

env = Environment()
env.SetViewer('qtcoin')
env.Load('data/lab1.env.xml')
robot = env.GetRobots()[0]

Tz = matrixFromAxisAngle([0,0,np.pi/4])

with env:
    #raveLogInfo("Robot "+robot.GetName()+" has "+repr(robot.GetDOF())+" joints with values:\n"+repr(robot.GetDOFValues()))
    #robot.SetDOFValues([0.5],[0])
    #T = robot.GetLinks()[1].GetTransform()
    #raveLogInfo("The transformation of link 1 is:\n"+repr(T))
    for body in env.GetBodies():
        body.SetTransform(np.dot(Tz,body.GetTransform()))

while True:
    pass
