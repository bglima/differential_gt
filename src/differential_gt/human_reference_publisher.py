#!/usr/bin/env python

import rospy
import geometry_msgs.msg

def publish_human_reference():

    # human_ref_publisher definition
    pub = rospy.Publisher('/human_ref', geometry_msgs.msg.PoseStamped, queue_size=30)
    rospy.init_node('human_ref_publisher', anonymous=True)

    # human_reference message type definition
    human_reference = geometry_msgs.msg.PoseStamped()

    # Publishing rate and duration definition
    rate = 30
    t = rospy.Time(0)
    d = rospy.Duration.from_sec(1.0/rate)
    Rate = rospy.Rate(rate)

    # Waiting for connecting the publisher node previously generated 
    while pub.get_num_connections() < 1:
        rospy.loginfo("Waiting for connection of at least another node to the /human_ref topic ...")
        rospy.sleep(5)
    
    # Publishing the simulated and static human reference iteratively
    while not rospy.is_shutdown():

        # Definition of the time stamped of the PoseStamped message
        human_reference.header.stamp = t

        # Definition of the simulated and static human reference
        human_reference.pose.position.x = 0.4572
        human_reference.pose.position.y = 0.15
        human_reference.pose.position.z = 0.632591
        human_reference.pose.orientation.x = 0.999998
        human_reference.pose.orientation.y = 0.000117441
        human_reference.pose.orientation.z = 0.00039217
        human_reference.pose.orientation.w = -3.94717e-05

        # Print the simulated and static human reference 
        # print("-------")
        # rospy.loginfo(t.to_sec())
        # rospy.loginfo(human_reference)
        pub.publish(human_reference)
        t += d

        # Keeping the publishing rate more or less fixed
        Rate.sleep()

if __name__ == '__main__':
    try:
        publish_human_reference()
    except rospy.ROSInterruptException:
        pass