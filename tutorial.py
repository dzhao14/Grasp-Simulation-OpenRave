import openravepy

env = openravepy.Environment()
help(env.CloneSelf)
help(openravepy.KinBody.GetChain)
help(openravepy.Robot.Manipulator.FindIKSolution)

try:
    env.Load('robots/barrettwam.robot.xml')
    env.GetRobots()[0].SetDOFValues([])
except openravepy.openrave_exception, e:
    print 'exception thrown'
    print e
