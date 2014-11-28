#!/usr/bin/env python
# license removed for brevity
import rospy
from ardrone_autonomy.msg import *

def callback(data):
    if(data.longitude > 0.0 or data.latitude > 0.0):
        rospy.loginfo("lon:%s"+ str(data.longitude) + ",lat:" + str(data.latitude))
        rospy.loginfo("Elevation:"+str(data.elevation))
    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/ardrone/navdata_gps", navdata_gps, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
