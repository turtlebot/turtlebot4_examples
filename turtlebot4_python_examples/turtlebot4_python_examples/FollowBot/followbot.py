#!/usr/bin/env python3

# Copyright 2021 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from vision_msgs.msg import Detection2DArray, Detection2D

from turtlebot4_msgs.msg import UserLed

from geometry_msgs.msg import Twist

from irobot_create_msgs.action import DockServo, Undock
from irobot_create_msgs.msg import Dock

class FollowBot(Node):
    UNKNOWN = 0
    LEFT = 1
    RIGHT = 2
    FORWARD = 3
    REVERSE = 4
    FORWARD_LEFT = 5
    FORWARD_RIGHT = 6
    STOP = 7

    direction = UNKNOWN
    previous_direction = RIGHT
    image_width = 300
    image_height = 300
    fwd_margin = 20
    turn_margin = 75
    stop_upper_thresh = 60000.0
    stop_lower_thresh = 50000.0
    is_docked = False

    def __init__(self):
        super().__init__('followbot')

        mobilenet_sub = self.create_subscription(Detection2DArray,
                                                     '/color/mobilenet_detections',
                                                     self.mobilenetCallback,
                                                     qos_profile_sensor_data)
        dock_sub = self.create_subscription(Dock,
                                            '/dock',
                                            self.dockCallback,
                                            qos_profile_sensor_data)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile_system_default)
        self.user_led_pub = self.create_publisher(UserLed, '/hmi/led', qos_profile_sensor_data)

        self.undock_action_client = ActionClient(self, Undock, '/undock')

    def getDriveDirection(self, detection: Detection2D):
        if detection is None:
            self.direction = self.UNKNOWN
            return

        position_x = detection.bbox.center.x
        bbox_size = detection.bbox.size_x * detection.bbox.size_y
        center_dist = position_x - self.image_width / 2

        # Person is centered
        if abs(center_dist) < self.fwd_margin:
            # Persons box is smaller than stop lower threshold, drive forward
            if bbox_size < self.stop_lower_thresh:
                self.direction = self.FORWARD
            # Persons box is larger than stop upper threshold, reverse
            elif bbox_size > self.stop_upper_thresh:
                self.direction = self.REVERSE
            # Persons box is larger than stop threshold, stop
            else:
                self.direction = self.STOP
        # Person is near center
        elif abs(center_dist) < self.turn_margin:
            # Person is to the right of center
            if center_dist > 0.0:
                # Persons box is smaller than stop threshold, drive forward and turn right
                if bbox_size < self.stop_lower_thresh:
                    self.direction = self.FORWARD_RIGHT
                # Persons box is larger than stop threshold, turn right
                else:
                    self.direction = self.RIGHT
            else:
                # Persons box is smaller than stop threshold, drive forward and turn left
                if bbox_size < self.stop_lower_thresh:
                    self.direction = self.FORWARD_LEFT
                # Persons box is larger than stop threshold, turn left
                else:
                    self.direction = self.LEFT
        # Person is near edge of frame
        else:
            # Turn right
            if center_dist > 0.0:
                self.direction = self.RIGHT
            # Turn left
            else:
                self.direction = self.LEFT

    def mobilenetCallback(self, msg: Detection2DArray):
        largest_box = 0
        closest_person = None
        if len(msg.detections) > 0:
            for detection in msg.detections:
                # Person detected
                if detection.id == '15':
                    bbox_size = detection.bbox.size_x * detection.bbox.size_y
                    if bbox_size > largest_box:
                        largest_box = bbox_size
                        closest_person = detection

        self.getDriveDirection(closest_person)

    def dockCallback(self, msg: Dock):
        self.is_docked = msg.is_docked

    def led(self, led, color, period, duty):
        msg = UserLed()
        msg.led = led
        msg.color = color
        msg.blink_period = period
        msg.duty_cycle = duty

        self.user_led_pub.publish(msg)

    def drive(self, linear_x, angular_z):
        msg = Twist()
        msg.angular.z = angular_z
        msg.linear.x = linear_x

        self.cmd_vel_pub.publish(msg)

    def undock(self):
        self.undock_action_client.wait_for_server()
        undock_goal_result = self.undock_action_client.send_goal(Undock.Goal())
        if undock_goal_result.result.is_docked:
            print('Undocking failed')

    def run(self):
        # Undock first
        if self.is_docked:
            print('Undocking')
            self.undock()

        while True:
            if self.direction == self.STOP:
                self.drive(0.0, 0.0)
                self.led(0, 0, 1000, 0.0)
                self.led(1, 2, 1000, 1.0)
            elif self.direction == self.FORWARD:
                self.drive(0.3, 0.0)
                self.led(0, 1, 1000, 1.0)
                self.led(1, 1, 1000, 1.0)
            elif self.direction == self.REVERSE:
                self.drive(-0.2, 0.0)
                self.led(0, 1, 1000, 0.5)
                self.led(1, 1, 1000, 0.5)
            elif self.direction == self.LEFT:
                self.drive(0.0, 0.35)
                self.previous_direction = self.LEFT
                self.led(0, 0, 1000, 0.5)
                self.led(1, 1, 1000, 0.5)
            elif self.direction == self.RIGHT:
                self.drive(0.0, -0.35)
                self.previous_direction = self.RIGHT
                self.led(0, 1, 1000, 0.5)
                self.led(1, 0, 1000, 0.5)
            elif self.direction == self.FORWARD_LEFT:
                self.drive(0.2, 0.25)
                self.previous_direction = self.LEFT
                self.led(0, 1, 1000, 1.0)
                self.led(1, 1, 1000, 0.5)
            elif self.direction == self.FORWARD_RIGHT:
                self.drive(0.2, -0.25)
                self.previous_direction = self.LEFT
                self.led(0, 1, 1000, 0.5)
                self.led(1, 1, 1000, 1.0)
            else:
                if self.previous_direction == self.LEFT:
                    self.drive(0.0, 0.75)
                    self.led(0, 0, 500, 0.5)
                    self.led(1, 1, 500, 0.5)
                else:
                    self.drive(0.0, -0.75)
                    self.led(0, 1, 500, 0.5)
                    self.led(1, 0, 500, 0.5)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)

    node = FollowBot()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Allow time for other nodes to start
    time.sleep(5)

    print('Running FollowBot...\n')

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

    thread.join()


if __name__ == '__main__':
    main()
