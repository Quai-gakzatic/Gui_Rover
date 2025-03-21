#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

def publish_stats():
    pub = rospy.Publisher('/rover_stats', Float32MultiArray, queue_size=10)
    rospy.init_node('stats_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        msg = Float32MultiArray(data=[80.5, 0.75, 12.3])  # Sample data
        pub.publish(msg)
        print("Published stats:", msg.data)  # Confirmation print
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_stats()
    except rospy.ROSInterruptException:
        pass
