#!/usr/bin/env python3
# Software License Agreement (BSD License)

import rospy
from std_msgs.msg import String


def callback(data):
    pub = rospy.Publisher('newchat', String, queue_size=10)
    i = 0
    rate = rospy.Rate(15) # 15hz
    
    while not rospy.is_shutdown():
        my_str = data.data + " smth new " + str(i)
        rospy.loginfo(my_str)
        pub.publish(my_str)
        rate.sleep()
        i += 1
        
def interrupter():
    rospy.init_node('interrupter', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        interrupter()
    except rospy.ROSInterruptException:
        pass