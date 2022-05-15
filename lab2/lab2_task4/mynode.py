#!/usr/bin/env python3
# Software License Agreement (BSD License)

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    
def interrupter():
    rospy.Subscriber('chatter', String, callback)
    pub = rospy.Publisher('newchat', String, queue_size=10)
    rospy.init_node('interrupter', anonymous=True)
    rate = rospy.Rate(15) # 15hz
    while not rospy.is_shutdown():
        my_str = "my new message %s" % rospy.get_time()
        rospy.loginfo(my_str)
        pub.publish(my_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        interrupter()
    except rospy.ROSInterruptException:
        pass