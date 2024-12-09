#!/usr/bin/env python

import rospy
import geometry_msgs.msg

def publish_robot_reference():

    # robot_ref_publisher definition
    pub = rospy.Publisher('/robot_ref', geometry_msgs.msg.PoseStamped, queue_size=30)
    rospy.init_node('robot_ref_publisher', anonymous=True)

    # robot_reference message type definition
    robot_reference = geometry_msgs.msg.PoseStamped()

    # Publishing rate and duration definition
    rate = 30
    t = rospy.Time(0)
    d = rospy.Duration.from_sec(1.0/rate)
    Rate = rospy.Rate(rate)

    # Waiting for connecting the publisher node previously generated 
    while pub.get_num_connections() < 1:
        rospy.loginfo("Waiting for connection of at least another node to the /robot_ref topic ...")
        rospy.sleep(5)

    # Publishing the simulated and static robot reference iteratively
    while not rospy.is_shutdown():

        # Definition of the time stamped of the PoseStamped message
        robot_reference.header.stamp = t
        
        # Definition of the simulated and static robot reference
        robot_reference.pose.position.x = 0.3072 
        robot_reference.pose.position.y = 1.6e-05
        robot_reference.pose.position.z = 0.482591
        robot_reference.pose.orientation.x = 0.999998
        robot_reference.pose.orientation.y = 0.000117441
        robot_reference.pose.orientation.z = 0.00039217
        robot_reference.pose.orientation.w = -3.94717e-05

        # Print the simulated and static robot reference
        # print("-------")
        # rospy.loginfo(t.to_sec())
        # rospy.loginfo(robot_reference)
        pub.publish(robot_reference)
        t += d

        # Keeping the publishing rate more or less fixed
        Rate.sleep()

if __name__ == '__main__':
    try:
        publish_robot_reference()
    except rospy.ROSInterruptException:
        pass