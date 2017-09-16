""" So I just copy pasta'd the three segments of code from Andres's tutorial 2 (gpd) and I'm not sure if they're supposed to go together in one file..."""

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

cloud = [] # global variable to store the point cloud

def cloudCallback(msg):
    global cloud
    if len(cloud) == 0:
        for p in point_cloud2.read_points(msg):
            cloud.append([p[0], p[1], p[2]])


"""So I don't need these next ~4 lines of code and the callback above because these 4 lines, I believe, convert the given cloud_pcd to a pointcloud. My input is already a point cloud so I won't be needing this..."""
# Create a ROS node.
rospy.init_node('select_grasp')

# Subscribe to the ROS topic that contains the grasps.
cloud_sub = rospy.Subscriber('/cloud_pcd', PointCloud2, cloudCallback)

# Wait for point cloud to arrive.
while len(cloud) == 0:
    rospy.sleep(0.01)

"""This part of the code fits a least squares plane to the pointcloud. The one constraint is that the plane must be in the form 0 = ax + by + c. (Thus the plane is parallel to the xy-plane or orthogonal to the z-axis) After the fitting, the error (z-distance squared) is calculated and all points in the point cloud that are too close to the fitted plane will be filtered out. The matrix representation of the plane is Ax = b where b is the z column of the point cloud, A = [x's, y's, 1's] of the pointcloud. The system is clearly inconsistent which is why a least squares solution is used.
"""
# Extract the nonplanar indices. Uses a least squares fit AX = b. Plane equation: z = ax + by + c.
import numpy as np
from scipy.linalg import lstsq

cloud = np.asarray(cloud)
B = cloud
A = np.c_[B[:,0], B[:,1], np.ones(B.shape[0])]
X, _, _, _ = lstsq(A, B[:,2])
a, b, c, d = X[0], X[1], -1., X[2] # coefficients of the form: a*x + b*y + c*z + d = 0.
dist = ((a*B[:,0] + b*B[:,1] + d) - B[:,2])**2
err = dist.sum()
idx = np.where(dist > 0.01)

# Publish point cloud and nonplanar indices.
from gpd.msg import CloudIndexed
from std_msgs.msg import Header, Int64
from geometry_msgs.msg import Point

pub = rospy.Publisher('cloud_indexed', CloudIndexed, queue_size=1)

msg = CloudIndexed()
header = Header()
header.frame_id = "/base_link"
header.stamp = rospy.Time.now()
msg.cloud_sources.cloud = point_cloud2.create_cloud_xyz32(header, cloud.tolist())
msg.cloud_sources.view_points.append(Point(0,0,0))
for i in xrange(cloud.shape[0]):
    msg.cloud_sources.camera_source.append(Int64(0))
for i in idx[0]:
    msg.indices.append(Int64(i))    
s = raw_input('Hit [ENTER] to publish')
pub.publish(msg)
rospy.sleep(2)
print 'Published cloud with', len(msg.indices), 'indices'
