#!/usr/bin/env python3
# Software License Agreement (BSD License)

import rospy
from std_msgs.msg import String

def speaker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('speaker', anonymous=True)
    rate = rospy.Rate(15) # 15hz
    while not rospy.is_shutdown():
        my_str = "my message %s" % rospy.get_time()
        rospy.loginfo(my_str)
        pub.publish(my_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        speaker()
    except rospy.ROSInterruptException:
        pass
