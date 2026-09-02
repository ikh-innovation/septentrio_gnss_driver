#!/usr/bin/env python3
"""
This ROS node subscribes to the /rtcm topic and forwards the received
RTCM correction data to the Septentrio GNSS receiver via a TCP socket.

The node automatically attempts to reconnect to the Septentrio receiver if the TCP connection is lost.
"""

import socket
import time
import rospy
from mavros_msgs.msg import RTCM

SEPTENTRIO_IP = "10.42.0.127"
PORT = 28784

class RTCMBridge:
    def __init__(self):
        rospy.init_node('rtcm_bridge')
        
        self.sock = None
        self.connect_tcp()
        
        # Subscription to /rtcm topic
        rospy.Subscriber('rtcm', RTCM, self.rtcm_callback)
        rospy.loginfo("[rtcm_bridge] Subscription to /rtcm, forwarding to Septentrio TCP socket.")
        
        rospy.spin()

    def connect_tcp(self):
        while not rospy.is_shutdown():
            try:
                rospy.loginfo(f"[rtcm_bridge] Connecting to Septentrio ({SEPTENTRIO_IP}:{PORT})...")
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((SEPTENTRIO_IP, PORT))
                rospy.loginfo("[rtcm_bridge] Connection succeeded!")
                break
            except Exception as e:
                rospy.logwarn(f"[rtcm_bridge] Connection failed: {e}. Retrying in 3 secs...")
                time.sleep(3)

    def rtcm_callback(self, msg):
        if self.sock:
            try:
                # Convert bytes of ROS message in port 28784
                self.sock.sendall(bytes(msg.data))
            except Exception as e:
                rospy.logerr(f"[rtcm_bridge] RTCM sending error: {e}")
                self.connect_tcp()

if __name__ == '__main__':
    try:
        RTCMBridge()
    except rospy.ROSInterruptException:
        pass
