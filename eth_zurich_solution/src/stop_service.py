#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool, SetBoolRequest


class EmergencyClient:
    def __init__(self, subscriber_queue_size=10):
        self._subscriber_queue_size = subscriber_queue_size
        self._service_name = "/startService"
        self._scan_topic = "/scan"
        self._max_distance_update = False
        self._stop_called = False
        self._prior_collision = False
        self._max_distance_to_pillar = 0.0
        self._collision_threshold = 0.0
        self._prev_imu_x = 0.0
        if not self._read_parameters():
            rospy.signal_shutdown("Could not read parameters")
        rospy.sleep(1)
        self._stop_client = rospy.ServiceProxy(self._service_name, SetBool)
        if self._prior_collision:
            self._stop_subscriber = rospy.Subscriber(
                "/scan",
                LaserScan,
                self._scan_callback,
                queue_size=self._subscriber_queue_size,
            )
        else:
            self._stop_subscriber = rospy.Subscriber(
                "/imu0",
                Imu,
                self._imu_callback,
                queue_size=self._subscriber_queue_size,
            )

    def on_shutdown(self):
        self._stop_subscriber.unregister()

    def _read_parameters(self):
        # Access with private parameters name
        success = True
        success &= rospy.has_param("~max_distance_to_pillar")
        self._max_distance_to_pillar = rospy.get_param(
            "~max_distance_to_pillar", self._max_distance_to_pillar
        )
        success &= rospy.has_param("~service_name")
        self._service_name = rospy.get_param("~service_name", self._service_name)
        success &= rospy.has_param("~collision_threshold")
        self._collision_threshold = rospy.get_param(
            "~collision_threshold", self._collision_threshold
        )
        success &= rospy.has_param("~prior_collision")
        self._prior_collision = rospy.get_param(
            "~prior_collision", self._prior_collision
        )
        return success

    def _request_stop(self):
        start_srv = SetBoolRequest()
        start_srv.data = False
        if not self._stop_client.call(start_srv):
            rospy.logerr("Failed to request stop")
            rospy.signal_shutdown()

    def _imu_callback(self, msg: Imu):
        imu_x = msg.linear_acceleration.x
        abs_change_x = abs(imu_x - self._prev_imu_x)
        if abs_change_x > self._collision_threshold:
            rospy.logwarn(f"Change in imu x: {abs_change_x}")
            self._request_stop()
        self._prev_imu_x = imu_x

    def _scan_callback(self, msg: LaserScan):
        min_distance = min(msg.ranges)

        if (
            self._max_distance_to_pillar < msg.range_min
            and not self._max_distance_update
        ):
            self._max_distance_to_pillar = msg.range_min
            self._max_distance_update = True

        if min_distance < self._max_distance_to_pillar:
            self._request_stop()


def main():
    rospy.init_node("stop_condition")
    emergency_client = EmergencyClient()
    rospy.on_shutdown(emergency_client.on_shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()