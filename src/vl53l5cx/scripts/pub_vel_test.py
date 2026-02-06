#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time

def talker():
    rospy.init_node('talker_node', anonymous=True)
    pub = rospy.Publisher('/final_cmd_vel_mux/output', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    direction = 1          # 1 = quay thuận, -1 = quay ngược
    last_switch_time = time.time()

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 0.1 * direction  # quay thuận hoặc ngược
        pub.publish(msg)

        rospy.loginfo(f"Gửi Twist: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")

        # Sau 5 giây thì đổi hướng quay
        if time.time() - last_switch_time >= 2.0:
            direction *= -1
            last_switch_time = time.time()
            rospy.loginfo("→ Đổi hướng quay!")

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
