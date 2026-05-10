#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Current sensor reader node
Reads $CURRENT,ch0,ch1,ch2*XX\r\n format current data and publishes to ROS topic

Serial port: /dev/ttyUSB1
Protocol: Text format with XOR checksum
          Format: $CURRENT,-0.0391,-0.0287,-0.1038*7E\r\n
          Checksum: XOR of all bytes from $ to * (exclusive)

Baud rate: 115200

    Published topic: /current_data (std_msgs/Float32MultiArray)
    data[0] = ch0
    data[1] = ch1
    data[2] = ch2

Usage:
    rosrun turn_on_wheeltec_robot current_reader.py _port:=/dev/ttyUSB1 _baud:=115200
"""

import rospy
import serial
from std_msgs.msg import Float32MultiArray

FRAME_HEADER = "$CURRENT,"
FRAME_TAIL = "*"
FRAME_END = "\r\n"


class CurrentReader:
    def __init__(self):
        rospy.init_node('current_reader', anonymous=True)

        # Parameters
        self.port = rospy.get_param('~port', '/dev/ttyUSB1')
        self.baud = rospy.get_param('~baud', 115200)
        self.timeout = rospy.get_param('~timeout', 1.0)

        # Publisher
        self.pub = rospy.Publisher('/current_data', Float32MultiArray, queue_size=10)

        # Open serial port
        rospy.loginfo("Opening serial port: %s @ %d" % (self.port, self.baud))
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout
            )
            rospy.loginfo("Serial port opened: %s" % self.serial_port.name)
        except serial.SerialException as e:
            rospy.logerr("Cannot open serial port: %s" % e)
            raise

        # Clear buffer
        self.serial_port.reset_input_buffer()

        # Statistics
        self.frame_count = 0
        self.error_count = 0
        self.last_report_time = rospy.Time.now().to_sec()

    def parse_frame(self, line):
        """
        Parse $CURRENT,ch0,ch1,ch2*XX\r\n frame
        Returns: [ch0, ch1, ch2] or None
        """
        try:
            # Align to frame header. Some USB-serial adapters may leave a
            # partial line in the buffer when the node starts.
            header_pos = line.find(FRAME_HEADER)
            if header_pos < 0:
                return None
            if header_pos > 0:
                line = line[header_pos:]

            # Find checksum separator
            if FRAME_TAIL not in line:
                return None

            # Extract data between $CURRENT, and *
            data_start = len(FRAME_HEADER)
            data_end = line.index(FRAME_TAIL, data_start)
            data_str = line[data_start:data_end]

            # Extract received checksum
            checksum_start = data_end + 1
            checksum_end = line.find(FRAME_END, checksum_start)
            if checksum_end == -1:
                # Try without \r\n
                recv_checksum_str = line[checksum_start:]
            else:
                recv_checksum_str = line[checksum_start:checksum_end]

            # Parse current values.  The firmware normally sends three values:
            #   $CURRENT,ch0,ch1,ch2*XX
            # A future extended text frame may add a sequence prefix:
            #   $CURRENT,seq,ch0,ch1,ch2*XX
            parts = data_str.split(',')
            if len(parts) == 4:
                parts = parts[1:]
            if len(parts) != 3:
                rospy.logwarn_throttle(5, "Expected 3 current channels, got %d" % len(parts))
                return None

            values = []
            for p in parts:
                try:
                    val = float(p)
                    values.append(val)
                except ValueError:
                    rospy.logwarn_throttle(5, "Invalid float value: %s" % p)
                    return None

            # Calculate checksum (XOR from $ to * exclusive)
            calc_checksum = 0
            for i in range(data_end):
                calc_checksum ^= ord(line[i])

            # Parse received checksum
            try:
                recv_checksum = int(recv_checksum_str, 16)
            except ValueError:
                rospy.logwarn_throttle(5, "Invalid checksum format: %s" % recv_checksum_str)
                return None

            # Verify checksum
            if calc_checksum != recv_checksum:
                rospy.logwarn_throttle(5, "Checksum mismatch: calc=0x%02X, recv=0x%02X" % (calc_checksum, recv_checksum))
                return None

            return values

        except Exception as e:
            rospy.logdebug("Parse frame failed: %s" % e)
        return None

    def read_frame(self):
        """
        Read one frame of data
        Returns: [ch0, ch1, ch2] or None
        """
        try:
            line = self.serial_port.readline()
            if not line:
                return None

            try:
                line = line.decode('ascii', errors='ignore')
            except Exception:
                return None

            return self.parse_frame(line)
        except Exception as e:
            rospy.logdebug("Read failed: %s" % e)
            return None

    def run(self):
        """Main loop"""
        rate = rospy.Rate(100)  # 100Hz
        rospy.loginfo("Start reading current data (3-channel omnidirectional robot)...")

        while not rospy.is_shutdown():
            try:
                data = self.read_frame()
                if data:
                    msg = Float32MultiArray()
                    msg.data = data
                    self.pub.publish(msg)
                    self.frame_count += 1
                else:
                    self.error_count += 1

                # Print statistics every 10 seconds
                now = rospy.Time.now().to_sec()
                if now - self.last_report_time >= 10.0:
                    measured_rate = self.frame_count / max(1e-6, now - self.last_report_time)
                    rospy.loginfo(
                        "Current reader: %.1f Hz in last window, %d total OK, %d errors",
                        measured_rate,
                        self.frame_count,
                        self.error_count,
                    )
                    self.frame_count = 0
                    self.error_count = 0
                    self.last_report_time = now

            except serial.SerialException as e:
                rospy.logerr("Serial error: %s" % e)
                break
            except KeyboardInterrupt:
                rospy.loginfo("User interrupt")
                break

            rate.sleep()

        # Close serial port
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            rospy.loginfo("Serial port closed")
            rospy.loginfo("Final statistics: %d frames OK, %d errors" % (self.frame_count, self.error_count))


if __name__ == '__main__':
    try:
        reader = CurrentReader()
        reader.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Startup failed: %s" % e)
