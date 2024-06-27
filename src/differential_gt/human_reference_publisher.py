#!/usr/bin/env python

import rospy
import std_msgs.msg
import geometry_msgs.msg


def talker():
    pub = rospy.Publisher('/human_ref', geometry_msgs.msg.PoseStamped, queue_size=10)
    rospy.init_node('human_ref_publisher', anonymous=True)
    rate = rospy.Rate(1/10) 
    human_reference = geometry_msgs.msg.PoseStamped()
    t = rospy.get_rostime()
    d = rospy.Duration.from_sec(10)

    while not rospy.is_shutdown():
        t_now = rospy.Time.now()
        time_interval = t_now - t

        human_reference.pose.position.x = 0.3072
        human_reference.pose.position.y = 1.6e-05
        human_reference.pose.position.z = 0.482591
        human_reference.pose.orientation.x = 0.99998
        human_reference.pose.orientation.y = 0.0062
        human_reference.pose.orientation.z = -3e-05
        human_reference.pose.orientation.w = -0.00155
        print("-------")
        rospy.loginfo(time_interval.to_sec())
        rospy.loginfo(human_reference)
        pub.publish(human_reference)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass