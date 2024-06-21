#!/usr/bin/env python

import rospy
import std_msgs.msg


def talker():
    pub = rospy.Publisher('/alpha_topic', std_msgs.msg.Float32, queue_size=10)
    rospy.init_node('alpha_pub', anonymous=True)
    rate = rospy.Rate(1/10) 
    alpha = std_msgs.msg.Float32()
    t = rospy.get_rostime()
    d = rospy.Duration.from_sec(10)

    while not rospy.is_shutdown():
        t_now = rospy.Time.now()
        time_interval = t_now - t
        if time_interval < d:
            alpha.data = 0.25
            print("-------")
            rospy.loginfo(time_interval.to_sec())
            rospy.loginfo(alpha)
            pub.publish(alpha)
            rate.sleep()
        elif time_interval > d and time_interval < 2*d:
            print("-------")
            alpha.data = 0.5
            rospy.loginfo(time_interval.to_sec())
            rospy.loginfo(alpha)
            pub.publish(alpha)      
            rate.sleep()
        elif time_interval > 2*d and time_interval < 3*d:
            print("-------")
            alpha.data = 0.75
            rospy.loginfo(time_interval.to_sec())
            rospy.loginfo(alpha)
            pub.publish(alpha) 
            rate.sleep()
        elif time_interval > 3*d:
            t = rospy.Time.now()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass