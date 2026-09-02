#!/usr/bin/env python
"""
Subscribes to Septentrio NavSatFix (lat/lon/alt) and AttEuler (heading),
calls the /aristos/fromLL service (robot_localization::FromLL)
to convert the GNSS antenna position to local map-frame XYZ,
then offsets that point using the static tf between the antenna and the robot's body frame
so the published marker is centered on the robot body (e.g. base_footprint) instead of on the antenna.

This is useful in Foxglove Costmap (Aristos Real tab) for comparing the robot's pose from the existing
localization pipeline with an independent pose estimate derived directly from the Septentrio GNSS data.
"""

import math
import rospy
import tf2_ros
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped
from robot_localization.srv import FromLL, FromLLRequest
from visualization_msgs.msg import Marker
from septentrio_gnss_driver.msg import AttEuler


class AristosBaseFromSeptentrio(object):
    def __init__(self):
        rospy.init_node("aristos_pose_from_septentrio")

        # tf Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Parameters
        self.input_topic   = rospy.get_param("~input_topic", "/septentrio_gnss/navsatfix")
        self.output_topic  = rospy.get_param("~output_topic", "/septentrio_gnss/xyz")
        self.heading_topic = rospy.get_param("~heading_topic", "/septentrio_gnss/atteuler")
        self.marker_topic  = rospy.get_param("~marker_topic", "/septentrio_gnss/marker")

        self.output_frame  = rospy.get_param("~output_frame", "map")
        self.body_frame    = rospy.get_param("~body_frame", "aristos_base_footprint")
        self.antenna_frame = rospy.get_param("~antenna_frame", "aristos_gnss_main_antenna_link")

        self.tf_timeout = rospy.Duration(0.2)
 
        self.yaw_rad = 0.0 # heading (rad) wrt aristos_gnss_main_antenna_link

        # Service call
        rospy.wait_for_service("/aristos/fromLL", timeout=10.0)
        self.from_ll_srv = rospy.ServiceProxy("/aristos/fromLL", FromLL)
        rospy.loginfo("[aristos_pose_from_septentrio] Service %s is up.", "/aristos/fromLL")

        # Publishers / Subscribers
        self.xyz_pub    = rospy.Publisher(self.output_topic, PointStamped, queue_size=10)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=10)

        self.lla_sub     = rospy.Subscriber(self.input_topic, NavSatFix, self.navsatfix_cb, queue_size=10)
        self.heading_sub = rospy.Subscriber(self.heading_topic, AttEuler, self.heading_cb, queue_size=10)

        rospy.loginfo(
            "[aristos_pose_from_septentrio] Listening on '%s' & '%s', publishing on '%s' & '%s.",
            self.input_topic,
            self.heading_topic,
            self.output_topic,
            self.marker_topic,
        )

    def heading_cb(self, msg):
        # Heading: deg to rad
        self.yaw_rad = math.radians(msg.heading)

    def navsatfix_cb(self, msg):
        # Ignore fixes with no valid status
        if msg.status.status < 0:
            rospy.logwarn_throttle(5.0, "[aristos_pose_from_septentrio] No NavSat fix yet, skipping conversion.")
            return

        # 1. Transformation from aristos_gnss_main_antenna_link to aristos_base_footprint
        try:
            trans = self.tf_buffer.lookup_transform(
                self.body_frame,    # to
                self.antenna_frame, # from
                rospy.Time(0),
                self.tf_timeout,
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(
                5.0,
                "[aristos_pose_from_septentrio] Transform %s -> %s unavailable: %s",
                self.body_frame,
                self.antenna_frame,
                e
            )
            return
        
        offset = trans.transform.translation

        # 2. Convert the GNSS lat/lon/alt of MAIN antenna to XYZ (wrt map frame)
        req = FromLLRequest()
        req.ll_point = GeoPoint()
        req.ll_point.latitude = msg.latitude
        req.ll_point.longitude = msg.longitude
        req.ll_point.altitude = msg.altitude

        try:
            resp = self.from_ll_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr_throttle(
                5.0,
                "[aristos_pose_from_septentrio] fromLL service call failed: %s",
                e
            )
            return

        stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()

        # 3. Publish PointStamped (raw antenna position in map frame)
        out = PointStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.output_frame
        out.point = resp.map_point
        self.xyz_pub.publish(out)

        # 4. The antenna's offset from aristos_base_footprint is fixed to the robot's
        #    body, so it rotates with the robot. Rotate that offset by the
        #    current heading, then subtract it from the antenna's map position
        #    to get aristos_base_footprint's map position.
        cos_yaw = math.cos(self.yaw_rad)
        sin_yaw = math.sin(self.yaw_rad)

        dx_map = cos_yaw * offset.x - sin_yaw * offset.y
        dy_map = sin_yaw * offset.x + cos_yaw * offset.y
        dz_map = offset.z

        x = resp.map_point.x - dx_map
        y = resp.map_point.y - dy_map
        z = resp.map_point.z - dz_map

        # 5. Publish 3D Box Marker, wrt aristos_base_footprint frame
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.output_frame
        marker.ns = "blue_box"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Marker (blue_box) dimensions (m) 
        marker.scale.x = 1.63
        marker.scale.y = 1.15
        marker.scale.z = 0.4

        # Marker (blue_box) color
        marker.color.r = 0.0
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.7

        # Marker (blue_box) position wrt map frame
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        # Orientation (quaternion) from heading
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(self.yaw_rad / 2.0)
        marker.pose.orientation.w = math.cos(self.yaw_rad / 2.0)

        self.marker_pub.publish(marker)


if __name__ == "__main__":
    try:
        AristosBaseFromSeptentrio()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass