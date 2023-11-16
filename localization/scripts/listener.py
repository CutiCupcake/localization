#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# rate = rospy.Rate(200) # 10hz
pub_hz_debug = rospy.Publisher('hz_test', String, queue_size=1)
pub_hz_debug = rospy.Publisher('odom_hz_test', Odometry, queue_size=1)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pub_hz_debug.publish(data)
    # rate.sleep()
    
def odom_callback(odom):
    pub_hz_debug.publish(odom)
    # rate.sleep()
    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('listener', anonymous=True)
    rate = rospy.Rate(200) # 10hz  
    rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("Odometry_imu", Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
