#!/usr/bin/env python

import rospy
import std_msgs.msg


def talker():
    pub = rospy.Publisher('/alpha', std_msgs.msg.Float32, queue_size=10)
    rospy.init_node('alpha_publisher', anonymous=True)
    rate = rospy.Rate(1/15) 
    alpha = std_msgs.msg.Float32()
    t = rospy.get_rostime()
    d = rospy.Duration.from_sec(15)

    while not rospy.is_shutdown():
        t_now = rospy.Time.now()
        time_interval = t_now - t

        if alpha.data > 0.9 and alpha.data < 1:
            print("")
            print("alpha publication from 0.1 to 0.9 with step 0.1 completed!")
            break

        if time_interval < d:
            alpha.data = alpha.data + 0.1
            print("-------")
            rospy.loginfo(time_interval.to_sec())
            rospy.loginfo(alpha)
            pub.publish(alpha)
            rate.sleep()
        elif time_interval > d:
            t = rospy.Time.now()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass