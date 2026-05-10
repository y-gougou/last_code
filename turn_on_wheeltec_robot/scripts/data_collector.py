#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Collect synchronized robot fault-diagnosis rows into a fixed CSV schema.

The node keeps the existing low-risk strategy: /odom triggers one CSV row, and
all other topics contribute their latest cached value.  Each row records source
timestamps and ages so bad synchronization can be found before training.
"""

from __future__ import print_function

import csv
import os
from datetime import datetime

import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray


class DataCollector(object):
    FAULT_NAMES = {
        0: "normal",
        1: "drive_fault",
        2: "wheel_slip",
        3: "shaft_eccentric",
        4: "encoder_fault",
    }

    CSV_FIELDS = [
        "timestamp",
        "run_id",
        "sample_id",
        "fault_label",
        "fault_name",
        "fault_mode",
        "motion_mode",
        "cmd_vx",
        "cmd_vy",
        "cmd_wz",
        "odom_vx",
        "odom_vy",
        "odom_wz",
        "wheel_speed0",
        "wheel_speed1",
        "wheel_speed2",
        "imu_ax",
        "imu_ay",
        "imu_az",
        "imu_gx",
        "imu_gy",
        "imu_gz",
        "voltage",
        "current_seq",
        "current0",
        "current1",
        "current2",
        "odom_time",
        "imu_time",
        "voltage_time",
        "current_time",
        "cmd_time",
        "odom_age",
        "imu_age",
        "voltage_age",
        "current_age",
        "cmd_age",
        "record_rate",
        "current_valid",
    ]

    def __init__(self):
        self.output_dir = rospy.get_param(
            "~output_dir", "/home/wheeltec/R550PLUS_data_collect/log"
        )
        self.run_id = rospy.get_param("~run_id", "")
        self.fault_label = int(rospy.get_param("~fault_label", 0))
        fault_name_param = rospy.get_param("~fault_name", "")
        self.fault_name = fault_name_param or self.FAULT_NAMES.get(
            self.fault_label, "unknown"
        )
        self.fault_mode = int(rospy.get_param("~fault_mode", 0))
        self.motion_mode = rospy.get_param("~motion_mode", "straight_0.5ms")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu")
        self.voltage_topic = rospy.get_param("~voltage_topic", "/PowerVoltage")
        self.current_topic = rospy.get_param("~current_topic", "/current_data")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.wheel_speed_topic = rospy.get_param("~wheel_speed_topic", "")

        if not self.run_id:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.run_id = "%s_%s" % (self.fault_name, stamp)

        self.sample_id = 0
        self.current_seq = -1
        self.last_record_wall_time = None
        self.csv_file = None
        self.csv_writer = None

        self.latest = {
            "odom": None,
            "imu": None,
            "voltage": None,
            "current": None,
            "cmd": None,
            "wheel_speed": None,
            "odom_time": None,
            "imu_time": None,
            "voltage_time": None,
            "current_time": None,
            "cmd_time": None,
            "wheel_speed_time": None,
        }

        self._open_csv_file()
        self._setup_subscribers()

        rospy.loginfo(
            "data_collector started: run_id=%s label=%d/%s mode=%s",
            self.run_id,
            self.fault_label,
            self.fault_name,
            self.motion_mode,
        )

    def _setup_subscribers(self):
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=50)
        rospy.Subscriber(self.imu_topic, Imu, self._imu_callback, queue_size=50)
        rospy.Subscriber(self.voltage_topic, Float32, self._voltage_callback, queue_size=20)
        rospy.Subscriber(
            self.current_topic, Float32MultiArray, self._current_callback, queue_size=50
        )
        rospy.Subscriber(self.cmd_topic, Twist, self._cmd_callback, queue_size=50)
        if self.wheel_speed_topic:
            rospy.Subscriber(
                self.wheel_speed_topic,
                Float32MultiArray,
                self._wheel_speed_callback,
                queue_size=50,
            )

    def _open_csv_file(self):
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        filename = "%s.csv" % self.run_id
        filepath = os.path.join(self.output_dir, filename)
        self.csv_file = open(filepath, "w")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.CSV_FIELDS)
        self.csv_writer.writeheader()
        rospy.loginfo("CSV output: %s", filepath)

    @staticmethod
    def _stamp_to_sec(stamp):
        value = stamp.to_sec()
        if value > 0:
            return value
        return rospy.Time.now().to_sec()

    @staticmethod
    def _fmt(value, precision=6):
        return ("%%.%df" % precision) % float(value)

    def _age(self, now, topic_time):
        if topic_time is None:
            return ""
        return self._fmt(max(0.0, now - topic_time))

    def _odom_callback(self, msg):
        self.latest["odom"] = msg
        self.latest["odom_time"] = self._stamp_to_sec(msg.header.stamp)
        self._record_row()

    def _imu_callback(self, msg):
        self.latest["imu"] = msg
        self.latest["imu_time"] = self._stamp_to_sec(msg.header.stamp)

    def _voltage_callback(self, msg):
        self.latest["voltage"] = msg
        self.latest["voltage_time"] = rospy.Time.now().to_sec()

    def _current_callback(self, msg):
        self.latest["current"] = msg
        self.latest["current_time"] = rospy.Time.now().to_sec()
        self.current_seq += 1

    def _cmd_callback(self, msg):
        self.latest["cmd"] = msg
        self.latest["cmd_time"] = rospy.Time.now().to_sec()

    def _wheel_speed_callback(self, msg):
        self.latest["wheel_speed"] = msg
        self.latest["wheel_speed_time"] = rospy.Time.now().to_sec()

    def _record_row(self):
        odom = self.latest["odom"]
        if odom is None:
            return

        timestamp = self.latest["odom_time"]
        wall_now = rospy.Time.now().to_sec()
        if self.last_record_wall_time is None:
            record_rate = 0.0
        else:
            dt = max(1e-6, wall_now - self.last_record_wall_time)
            record_rate = 1.0 / dt
        self.last_record_wall_time = wall_now

        imu = self.latest["imu"]
        voltage = self.latest["voltage"]
        current = self.latest["current"]
        cmd = self.latest["cmd"]
        wheel_speed = self.latest["wheel_speed"]

        current_values = list(current.data) if current is not None else []
        wheel_values = list(wheel_speed.data) if wheel_speed is not None else []

        row = {
            "timestamp": self._fmt(timestamp),
            "run_id": self.run_id,
            "sample_id": self.sample_id,
            "fault_label": self.fault_label,
            "fault_name": self.fault_name,
            "fault_mode": self.fault_mode,
            "motion_mode": self.motion_mode,
            "cmd_vx": self._fmt(cmd.linear.x if cmd else 0.0),
            "cmd_vy": self._fmt(cmd.linear.y if cmd else 0.0),
            "cmd_wz": self._fmt(cmd.angular.z if cmd else 0.0),
            "odom_vx": self._fmt(odom.twist.twist.linear.x),
            "odom_vy": self._fmt(odom.twist.twist.linear.y),
            "odom_wz": self._fmt(odom.twist.twist.angular.z),
            "wheel_speed0": self._fmt(wheel_values[0] if len(wheel_values) > 0 else 0.0),
            "wheel_speed1": self._fmt(wheel_values[1] if len(wheel_values) > 1 else 0.0),
            "wheel_speed2": self._fmt(wheel_values[2] if len(wheel_values) > 2 else 0.0),
            "imu_ax": self._fmt(imu.linear_acceleration.x if imu else 0.0),
            "imu_ay": self._fmt(imu.linear_acceleration.y if imu else 0.0),
            "imu_az": self._fmt(imu.linear_acceleration.z if imu else 0.0),
            "imu_gx": self._fmt(imu.angular_velocity.x if imu else 0.0),
            "imu_gy": self._fmt(imu.angular_velocity.y if imu else 0.0),
            "imu_gz": self._fmt(imu.angular_velocity.z if imu else 0.0),
            "voltage": self._fmt(voltage.data if voltage else 0.0, 4),
            "current_seq": self.current_seq,
            "current0": self._fmt(current_values[0] if len(current_values) > 0 else 0.0, 4),
            "current1": self._fmt(current_values[1] if len(current_values) > 1 else 0.0, 4),
            "current2": self._fmt(current_values[2] if len(current_values) > 2 else 0.0, 4),
            "odom_time": self._fmt(self.latest["odom_time"] or timestamp),
            "imu_time": self._fmt(self.latest["imu_time"] or timestamp),
            "voltage_time": self._fmt(self.latest["voltage_time"] or timestamp),
            "current_time": self._fmt(self.latest["current_time"] or timestamp),
            "cmd_time": self._fmt(self.latest["cmd_time"] or timestamp),
            "odom_age": self._age(wall_now, self.latest["odom_time"]),
            "imu_age": self._age(wall_now, self.latest["imu_time"]),
            "voltage_age": self._age(wall_now, self.latest["voltage_time"]),
            "current_age": self._age(wall_now, self.latest["current_time"]),
            "cmd_age": self._age(wall_now, self.latest["cmd_time"]),
            "record_rate": self._fmt(record_rate, 3),
            "current_valid": 1 if len(current_values) >= 3 else 0,
        }

        self.csv_writer.writerow(row)
        self.csv_file.flush()
        self.sample_id += 1

        if self.sample_id % 500 == 0:
            rospy.loginfo(
                "recorded %d rows, rate=%.1f Hz, current_age=%s",
                self.sample_id,
                record_rate,
                row["current_age"],
            )

    def shutdown(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        rospy.loginfo("data_collector stopped: %d rows", self.sample_id)


def main():
    rospy.init_node("data_collector", anonymous=False)
    collector = DataCollector()
    rospy.on_shutdown(collector.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
