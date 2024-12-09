#!/usr/bin/env python

import rospy
import std_msgs.msg

def publish_alpha():

    # Publisher node definition
    pub = rospy.Publisher('/alpha', std_msgs.msg.Float32, queue_size=30)
    rospy.init_node('alpha_publisher', anonymous=True)

    # Alpha definition
    alpha = std_msgs.msg.Float32()
    alpha.data = 0
    alpha_step = 0.001

    # Publishing rate and duration definition
    rate = 30
    t = rospy.Time(0)
    d = rospy.Duration.from_sec(1.0/rate)
    Rate = rospy.Rate(rate)

    # Waiting for connecting the publisher node previously generated
    while pub.get_num_connections() < 1:
        rospy.loginfo("Waiting for connection of at least another node to the /alpha topic ...")
        rospy.sleep(5)

    # Publishing alpha iteratively with an increasing step of 0.001
    while not rospy.is_shutdown():
        
        if alpha.data >= 0.999:
            alpha.data = 0.999
        else:
            alpha.data += alpha_step
            alpha.data = round(alpha.data,3)
        
        # Print the current value of alpha
        # print("-------")
        # rospy.loginfo(t.to_sec())
        # rospy.loginfo(alpha)
        pub.publish(alpha)
        t += d

        # Keeping the publishing rate more or less fixed
        Rate.sleep()

if __name__ == '__main__':
    try:
        publish_alpha()
    except rospy.ROSInterruptException:
        pass