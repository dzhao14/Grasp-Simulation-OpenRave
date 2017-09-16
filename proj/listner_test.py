import rospy
import sensor_msgs.point_cloud2 as pc2

def callback(data):
    print data.data

if __name__ == "__main__":
    rospy.init_node('AreYouTHERE', anonymous = True)
    rospy.Subscriber("/detect_grasps", pc2.PointCloud2, callback)
    rospy.spin()
