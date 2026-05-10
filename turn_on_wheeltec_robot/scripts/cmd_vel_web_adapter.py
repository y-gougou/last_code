#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Lightweight robot-side safety adapter for browser teleoperation.

Input topics:
  - /cmd_vel_web (geometry_msgs/Twist)
  - /web/heartbeat (std_msgs/Empty)
  - /web/estop (std_msgs/Bool)
  - /web/cruise_cmd (geometry_msgs/Twist) - cruise control target speeds
  - /web/test_cmd (geometry_msgs/Twist) - direct fixed-speed test command
  - /web/test_enable (std_msgs/Bool) - enable direct fixed-speed test mode

Output topic:
  - /cmd_vel (geometry_msgs/Twist)

This node keeps the existing chassis node untouched while adding:
  - Timeout / heartbeat / estop safety
  - Speed limiting and response shaping
  - Cruise control with PID velocity闭环

Cruise control usage:
  rostopic pub /web/cruise_cmd geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}}'
  rostopic pub /web/cruise_cmd geometry_msgs/Twist '{}'  # cancel cruise
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, String
from nav_msgs.msg import Odometry


class PIDController:
    """简单PID控制器"""

    def __init__(self, kp=0.8, ki=0.1, kd=0.3, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, target, current, dt=None):
        """计算PID输出"""
        error = target - current

        # 微分项
        if dt is not None and dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0

        # 积分项（带限幅）
        self.integral += error * (dt or 0.1)
        self.integral = max(-0.5, min(0.5, self.integral))  # 积分限幅

        # PID输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # 输出限幅
        output = max(-self.output_limit, min(self.output_limit, output))

        self.prev_error = error
        return output

    def reset(self):
        """重置PID状态"""
        self.integral = 0.0
        self.prev_error = 0.0


class WebCmdVelAdapter(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_web")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.heartbeat_topic = rospy.get_param("~heartbeat_topic", "/web/heartbeat")
        self.estop_topic = rospy.get_param("~estop_topic", "/web/estop")
        self.status_topic = rospy.get_param("~status_topic", "/web/control_status")
        self.cruise_cmd_topic = rospy.get_param("~cruise_cmd_topic", "/web/cruise_cmd")
        self.test_cmd_topic = rospy.get_param("~test_cmd_topic", "/web/test_cmd")
        self.test_enable_topic = rospy.get_param("~test_enable_topic", "/web/test_enable")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")

        self.publish_rate = rospy.get_param("~publish_rate", 20.0)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)
        self.heartbeat_timeout = rospy.get_param("~heartbeat_timeout", 1.0)

        self.max_linear_x = rospy.get_param("~max_linear_x", 0.4)
        self.max_linear_y = rospy.get_param("~max_linear_y", 0.4)
        self.max_angular_z = rospy.get_param("~max_angular_z", 1.0)
        self.linear_deadband = rospy.get_param("~linear_deadband", 0.02)
        self.angular_deadband = rospy.get_param("~angular_deadband", 0.03)
        self.response_exponent = rospy.get_param("~response_exponent", 0.70)
        self.min_linear_ratio = rospy.get_param("~min_linear_ratio", 0.20)
        self.min_lateral_ratio = rospy.get_param("~min_lateral_ratio", 0.15)
        self.min_angular_ratio = rospy.get_param("~min_angular_ratio", 0.24)

        # PID参数
        self.pid_rate = rospy.get_param("~pid_rate", 10.0)  # PID更新频率

        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)
        self.last_heartbeat_time = rospy.Time(0)
        self.have_cmd = False
        self.estop_latched = False
        self.last_status = ""

        # 巡航控制状态
        self.cruise_active = False
        self.cruise_target = Twist()  # 目标速度
        self.current_velocity = Twist()  # 当前实际速度
        self.cruise_cmd_time = rospy.Time(0)
        self.test_active = False
        self.test_target = Twist()
        self.test_cmd_time = rospy.Time(0)

        # PID控制器（每轴一个）
        self.pid_x = PIDController(kp=0.8, ki=0.1, kd=0.3, output_limit=self.max_linear_x)
        self.pid_y = PIDController(kp=0.8, ki=0.1, kd=0.3, output_limit=self.max_linear_y)
        self.pid_angular = PIDController(kp=1.0, ki=0.05, kd=0.2, output_limit=self.max_angular_z)

        self.cmd_pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.cruise_status_pub = rospy.Publisher("/web/cruise_status", String, queue_size=10, latch=True)

        rospy.Subscriber(self.input_topic, Twist, self.cmd_callback)
        rospy.Subscriber(self.heartbeat_topic, Empty, self.heartbeat_callback)
        rospy.Subscriber(self.estop_topic, Bool, self.estop_callback)
        rospy.Subscriber(self.cruise_cmd_topic, Twist, self.cruise_cmd_callback)
        rospy.Subscriber(self.test_cmd_topic, Twist, self.test_cmd_callback)
        rospy.Subscriber(self.test_enable_topic, Bool, self.test_enable_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber("/web/cruise_enable", Bool, self.cruise_enable_callback)

        rospy.on_shutdown(self.on_shutdown)
        self.publish_status("idle")

        rospy.loginfo(
            "web_cmd_vel_adapter ready: %s -> %s (cmd_timeout=%.2fs, heartbeat_timeout=%.2fs)",
            self.input_topic,
            self.output_topic,
            self.cmd_timeout,
            self.heartbeat_timeout,
        )

    def clamp(self, value, limit):
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def zero_twist(self):
        return Twist()

    def shape_axis(self, value, limit, deadband, min_ratio):
        if limit <= 0.0:
            return 0.0

        magnitude = abs(value)
        if magnitude <= 0.0:
            return 0.0

        normalized = min(1.0, magnitude / limit)
        if normalized <= deadband:
            return 0.0

        span = max(1e-6, 1.0 - deadband)
        normalized = (normalized - deadband) / span
        shaped = min_ratio + (1.0 - min_ratio) * pow(normalized, self.response_exponent)
        return shaped * limit if value >= 0.0 else -shaped * limit

    def publish_status(self, status):
        if status != self.last_status:
            self.last_status = status
            self.status_pub.publish(String(data=status))
            rospy.loginfo("web_cmd_vel_adapter status: %s", status)

    def publish_cruise_status(self):
        """发布巡航状态"""
        if self.cruise_active:
            status = "cruise: vx=%.2f(%.2f) vy=%.2f(%.2f) wz=%.2f(%.2f)" % (
                self.cruise_target.linear.x, self.current_velocity.linear.x,
                self.cruise_target.linear.y, self.current_velocity.linear.y,
                self.cruise_target.angular.z, self.current_velocity.angular.z
            )
        else:
            status = "cruise: off"
        self.cruise_status_pub.publish(String(data=status))

    def cmd_callback(self, msg):
        # 注意：巡航模式下不退出巡航，巡航优先于手动控制
        # 巡航模式只能通过 /web/cruise_enable:=False 或 E-stop 退出

        limited = Twist()
        limited.linear.x = self.shape_axis(
            self.clamp(msg.linear.x, self.max_linear_x),
            self.max_linear_x,
            self.linear_deadband,
            self.min_linear_ratio,
        )
        limited.linear.y = self.shape_axis(
            self.clamp(msg.linear.y, self.max_linear_y),
            self.max_linear_y,
            self.linear_deadband,
            self.min_lateral_ratio,
        )
        limited.angular.z = self.shape_axis(
            self.clamp(msg.angular.z, self.max_angular_z),
            self.max_angular_z,
            self.angular_deadband,
            self.min_angular_ratio,
        )

        self.last_cmd = limited
        self.last_cmd_time = rospy.Time.now()
        self.have_cmd = True

    def cruise_cmd_callback(self, msg):
        """处理巡航命令 - 只设置目标速度，不控制巡航开关"""
        now = rospy.Time.now()

        # 设置巡航目标（无论是否已激活）
        self.cruise_target.linear.x = self.clamp(msg.linear.x, self.max_linear_x)
        self.cruise_target.linear.y = self.clamp(msg.linear.y, self.max_linear_y)
        self.cruise_target.angular.z = self.clamp(msg.angular.z, self.max_angular_z)

        self.cruise_cmd_time = now

        rospy.loginfo("Cruise target set: vx=%.2f vy=%.2f wz=%.2f",
                     self.cruise_target.linear.x,
                     self.cruise_target.linear.y,
                     self.cruise_target.angular.z)

    def cruise_enable_callback(self, msg):
        """处理巡航使能/禁用"""
        if msg.data:
            if self.test_active:
                self.exit_test_mode()
            if not self.cruise_active:
                self.enter_cruise()
                rospy.loginfo("Cruise enabled via /web/cruise_enable")
        else:
            if self.cruise_active:
                self.exit_cruise()
                rospy.loginfo("Cruise disabled via /web/cruise_enable")

    def test_cmd_callback(self, msg):
        self.test_target.linear.x = self.clamp(msg.linear.x, self.max_linear_x)
        self.test_target.linear.y = self.clamp(msg.linear.y, self.max_linear_y)
        self.test_target.angular.z = self.clamp(msg.angular.z, self.max_angular_z)
        self.test_cmd_time = rospy.Time.now()

    def test_enable_callback(self, msg):
        if msg.data:
            if self.cruise_active:
                self.exit_cruise()
            self.test_active = True
            self.test_cmd_time = rospy.Time.now()
            self.publish_status("test")
            rospy.loginfo(
                "Test mode enabled: vx=%.2f vy=%.2f wz=%.2f",
                self.test_target.linear.x,
                self.test_target.linear.y,
                self.test_target.angular.z,
            )
        else:
            self.exit_test_mode()

    def exit_test_mode(self):
        if not self.test_active:
            return
        self.test_active = False
        self.test_target = Twist()
        self.publish_status("idle")
        self.cmd_pub.publish(self.zero_twist())
        rospy.loginfo("Test mode disabled")

    def odom_callback(self, msg):
        """获取当前实际速度"""
        self.current_velocity.linear.x = msg.twist.twist.linear.x
        self.current_velocity.linear.y = msg.twist.twist.linear.y
        self.current_velocity.angular.z = msg.twist.twist.angular.z

    def enter_cruise(self):
        """进入巡航模式"""
        self.cruise_active = True
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_angular.reset()
        self.publish_status("cruise")
        rospy.loginfo("Cruise control activated")

    def exit_cruise(self):
        """退出巡航模式"""
        if not self.cruise_active:
            return
        self.cruise_active = False
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_angular.reset()
        self.publish_status("idle")
        rospy.loginfo("Cruise control deactivated")

    def heartbeat_callback(self, _msg):
        self.last_heartbeat_time = rospy.Time.now()

    def estop_callback(self, msg):
        if msg.data:
            self.estop_latched = True
            if self.cruise_active:
                self.exit_cruise()
            if self.test_active:
                self.exit_test_mode()
            self.publish_status("estop")
            self.cmd_pub.publish(self.zero_twist())
        else:
            self.estop_latched = False
            self.publish_status("idle")

    def compute_cruise_cmd(self, dt):
        """使用PID计算巡航控制输出"""
        cmd = Twist()

        # X轴PID
        cmd.linear.x = self.pid_x.compute(
            self.cruise_target.linear.x,
            self.current_velocity.linear.x,
            dt
        )

        # Y轴PID
        cmd.linear.y = self.pid_y.compute(
            self.cruise_target.linear.y,
            self.current_velocity.linear.y,
            dt
        )

        # 角速度PID
        cmd.angular.z = self.pid_angular.compute(
            self.cruise_target.angular.z,
            self.current_velocity.angular.z,
            dt
        )

        return cmd

    def resolve_status_and_command(self):
        now = rospy.Time.now()

        if self.estop_latched:
            return "estop", self.zero_twist()

        if self.test_active:
            if self.heartbeat_timeout > 0.0:
                if self.last_heartbeat_time.to_sec() == 0.0:
                    return "waiting_heartbeat", self.zero_twist()
                if (now - self.last_heartbeat_time).to_sec() > self.heartbeat_timeout:
                    self.exit_test_mode()
                    return "heartbeat_timeout", self.zero_twist()
            if self.cmd_timeout > 0.0 and (now - self.test_cmd_time).to_sec() > self.cmd_timeout:
                self.exit_test_mode()
                return "cmd_timeout", self.zero_twist()
            return "test", self.test_target

        if self.cruise_active:
            return "cruise", None  # 命令由cruise控制循环计算

        if not self.have_cmd:
            return "idle", self.zero_twist()

        if self.heartbeat_timeout > 0.0:
            if self.last_heartbeat_time.to_sec() == 0.0:
                return "waiting_heartbeat", self.zero_twist()
            if (now - self.last_heartbeat_time).to_sec() > self.heartbeat_timeout:
                return "heartbeat_timeout", self.zero_twist()

        if self.cmd_timeout > 0.0 and (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
            return "cmd_timeout", self.zero_twist()

        return "active", self.last_cmd

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        pid_rate = rospy.Rate(self.pid_rate)

        last_pid_time = rospy.Time.now()
        cruise_cmd = self.zero_twist()

        while not rospy.is_shutdown():
            status, manual_cmd = self.resolve_status_and_command()
            self.publish_status(status)

            if status == "cruise":
                # PID控制循环
                now = rospy.Time.now()
                dt = (now - last_pid_time).to_sec()
                if dt > 0:
                    cruise_cmd = self.compute_cruise_cmd(dt)
                    last_pid_time = now
                self.cmd_pub.publish(cruise_cmd)
                self.publish_cruise_status()
            else:
                self.cmd_pub.publish(manual_cmd if manual_cmd else self.zero_twist())
                # 巡航状态也要更新（显示当前速度）
                if status != "estop":
                    self.publish_cruise_status()

            rate.sleep()

    def on_shutdown(self):
        self.cmd_pub.publish(self.zero_twist())


if __name__ == "__main__":
    rospy.init_node("web_cmd_vel_adapter", anonymous=False)
    WebCmdVelAdapter().run()
