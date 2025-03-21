#!/usr/bin/env python

import rospy
from differential_gt.msg import alpha_with_header

def publish_alpha():

    # Publisher node definition
    pub = rospy.Publisher('/alpha', alpha_with_header, queue_size=30)
    rospy.init_node('alpha_publisher', anonymous=True)

    # alpha definition
    alpha_with_header_msg = alpha_with_header()
    alpha_with_header_msg.alpha.data = 0
    alpha_step = 0.001

    # Waiting for connecting the publisher node previously generated
    while pub.get_num_connections() < 1:
        rospy.loginfo("Waiting for connection of at least another node to the /alpha topic ...")
        rospy.sleep(20)

    # Publishing rate and duration definition
    rate = 30
    t = 0
    while not t:
        t = rospy.Time.now()

    d = rospy.Duration.from_sec(1.0/rate)
    Rate = rospy.Rate(rate)

    # Publishing alpha iteratively with an increasing step of 0.001
    while not rospy.is_shutdown():

        # Update the Header of the alpha_with_header message
        alpha_with_header_msg.header.stamp = t
        
        if alpha_with_header_msg.alpha.data >= 0.999:
            alpha_with_header_msg.alpha.data = 0.999
        else:
            alpha_with_header_msg.alpha.data += alpha_step
            alpha_with_header_msg.alpha.data = round(alpha_with_header_msg.alpha.data,3)
        
        # Print the current value of alpha
        # print("-------")
        # rospy.loginfo(t.to_sec())
        # rospy.loginfo(alpha)
        pub.publish(alpha_with_header_msg)
        t += d

        # Keeping the publishing rate more or less fixed
        Rate.sleep()

if __name__ == '__main__':
    try:
        publish_alpha()
    except rospy.ROSInterruptException:
        pass