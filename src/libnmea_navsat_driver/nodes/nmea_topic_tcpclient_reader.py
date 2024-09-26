# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import socket
import sys
import rclpy

from nmea_msgs.msg import Sentence
from libnmea_navsat_driver.driver import Ros2NMEADriver

def main(args=None):
    rclpy.init(args=args)

    driver = Ros2NMEADriver()
    nmea_pub = driver.create_publisher(Sentence, "nmea_sentence", 10)

    try:
        gnss_ip = driver.declare_parameter('ip', '192.168.131.22').value
        gnss_port = driver.declare_parameter('port', 9001).value
        buffer_size = driver.declare_parameter('buffer_size', 4096).value
    except KeyError as e:
        driver.get_logger().err("Parameter %s not found" % e)
        sys.exit(1)

    frame_id = driver.get_frame_id()

    driver.get_logger().info("Using gnss sensor with ip {} and port {}".format(gnss_ip, gnss_port))

    # Connection-loop: connect and keep receiving. If receiving fails, reconnect
    # Connect to the gnss sensor using tcp
    while rclpy.ok():
        try:
            # Create a socket
            gnss_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # Connect to the gnss sensor
            gnss_socket.connect((gnss_ip, gnss_port))
        except socket.error as exc:
            driver.get_logger().error("Caught exception socket.error when setting up socket: %s" % exc)
            sys.exit(1)

        # recv-loop: When we're connected, keep receiving stuff until that fails
        partial = ""
        while rclpy.ok():
            try:
                partial += gnss_socket.recv(buffer_size).decode("ascii")

                # strip the data
                lines = partial.splitlines()
                if partial.endswith('\n'):
                    full_lines = lines
                    partial = ""
                else:
                    full_lines = lines[:-1]
                    partial = lines[-1]

                for data in full_lines:
                    try:
                        sentence = Sentence()
                        sentence.header.stamp = driver.get_clock().now().to_msg()
                        sentence.header.frame_id = frame_id
                        sentence.sentence = data
                        nmea_pub.publish(sentence)
                        
                    except ValueError as e:
                        driver.get_logger().warn(
                            "Value error, likely due to missing fields in the NMEA message. "
                            "Error was: %s. Please report this issue to me. " % e)

            except socket.error as exc:
                driver.get_logger().error("Caught exception socket.error when receiving: %s" % exc)
                gnss_socket.close()
                break


        gnss_socket.close()
