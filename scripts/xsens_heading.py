#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class HeadingPublisher:
    def __init__(self):
        rospy.init_node('xsens_heading')

        self.pub = rospy.Publisher('/aristos/imu/heading', Float64, queue_size=10)
        rospy.Subscriber('/aristos/imu/data', Imu, self.imu_callback)
        rospy.loginfo("[xsens_heading] Subscription to /aristos/imu/data, publishing heading to /aristos/imu/heading.")

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        # yaw calcultion from quaternion
        yaw = math.atan2(
            2 * (qw * qz + qx * qy),
            1 - 2 * (qy**2 + qz**2)
        )
        yaw = math.degrees(yaw)

        heading_msg = Float64()
        heading_msg.data = yaw
        self.pub.publish(heading_msg)

if __name__ == '__main__':
    try:
        HeadingPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
