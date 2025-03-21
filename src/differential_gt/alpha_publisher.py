#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

# Get the alpha parameter from the .yaml file
alpha = rospy.get_param("alpha")

def publish_alpha():

    # Publisher node definition
    pub = rospy.Publisher('/alpha', Float32, queue_size=30)
    rospy.init_node('alpha_publisher', anonymous=True)

    # alpha definition
    alpha_msg = Float32()
    alpha_msg.data = round(alpha,3)

    # Waiting for connecting the publisher node previously generated
    # while pub.get_num_connections() < 1:
    #     rospy.loginfo("Waiting for connection of at least another node to the /alpha topic ...")
    #     rospy.sleep(1)

    # Publishing rate and duration definition
    rate = 1.0/10
    Rate = rospy.Rate(rate)

    # Publishing alpha iteratively with an increasing step of 0.001
    while not rospy.is_shutdown():
        
        # Print the current value of alpha
        print("-------")
        rospy.loginfo(alpha)
        pub.publish(alpha_msg)

        # Keeping the publishing rate more or less fixed
        Rate.sleep()

if __name__ == '__main__':
    try:
        publish_alpha()
    except rospy.ROSInterruptException:
        pass