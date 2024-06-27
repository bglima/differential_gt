#!/usr/bin/env python

import rospy
import std_msgs.msg
import geometry_msgs.msg


def talker():
    pub = rospy.Publisher('/robot_ref', geometry_msgs.msg.PoseStamped, queue_size=10)
    rospy.init_node('robot_ref_publisher', anonymous=True)
    rate = rospy.Rate(1/10) 
    robot_reference = geometry_msgs.msg.PoseStamped()
    t = rospy.get_rostime()
    d = rospy.Duration.from_sec(10)

    while not rospy.is_shutdown():
        t_now = rospy.Time.now()
        time_interval = t_now - t

        robot_reference.pose.position.x = 0
        robot_reference.pose.position.y = 0
        robot_reference.pose.position.z = 0
        robot_reference.pose.orientation.x = 1
        robot_reference.pose.orientation.y = 0
        robot_reference.pose.orientation.z = 0
        robot_reference.pose.orientation.w = 0
        print("-------")
        rospy.loginfo(time_interval.to_sec())
        rospy.loginfo(robot_reference)
        pub.publish(robot_reference)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass