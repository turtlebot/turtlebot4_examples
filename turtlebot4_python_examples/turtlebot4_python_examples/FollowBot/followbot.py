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

from depthai_ros_msgs.msg import SpatialDetectionArray, SpatialDetection

from turtlebot4_msgs.msg import UserLed

from geometry_msgs.msg import Twist

from irobot_create_msgs.action import DockServo, Undock
from irobot_create_msgs.msg import Dock

class FollowBot(Node):
    UNKNOWN = 0
    LEFT = 1
    RIGHT = 2
    FORWARD = 3
    FORWARD_LEFT = 4
    FORWARD_RIGHT = 5
    STOP = 6

    direction = UNKNOWN
    previous_direction = RIGHT
    image_width = 300
    image_height = 300
    fps = 15
    fwd_margin = 20
    turn_margin = 75
    stop_lower_thresh = 1.0
    is_docked = False
    last_target_person = None

    def __init__(self):
        super().__init__('followbot')

        mobilenet_sub = self.create_subscription(SpatialDetectionArray,
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

    def getDriveDirection(self, detection: SpatialDetection):
        if detection is None:
            self.direction = self.UNKNOWN
            return

        position_x = detection.bbox.center.x
        distance = detection.position.z
        center_dist = position_x - self.image_width / 2

        # Person is centered
        if abs(center_dist) < self.fwd_margin:
            # Persons box is smaller than stop lower threshold, drive forward
            if distance > self.stop_lower_thresh:
                self.direction = self.FORWARD
            # Persons box is larger than stop upper threshold,
            # stop if box width is large enough
            # elif bbox_y > self.stop_upper_y_thresh:
            #     if bbox_x > self.stop_upper_x_thresh:
            #         self.direction = self.STOP
            #     else:
            #         self.direction = self.FORWARD
            # Persons box is larger than stop threshold, stop
            else:
                self.direction = self.STOP
        # Person is near center
        elif abs(center_dist) < self.turn_margin:
            # Person is to the right of center
            if center_dist > 0.0:
                # Persons box is smaller than stop threshold, drive forward and turn right
                if distance > self.stop_lower_thresh:
                    self.direction = self.FORWARD_RIGHT
                # Persons box is larger than stop threshold, turn right
                else:
                    self.direction = self.RIGHT
            else:
                # Persons box is smaller than stop threshold, drive forward and turn left
                if distance > self.stop_lower_thresh:
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

    def mobilenetCallback(self, msg: SpatialDetectionArray):
        closest_target_dist = self.image_width
        target_person = None
        if len(msg.detections) > 0:
            for detection in msg.detections:
                # Person detected
                if detection.results[0].class_id == '15' and detection.results[0].score > 0.90:
                    # No one previously detected, target first detection
                    if self.last_target_person is None:
                        target_person = detection
                        break
                    # Find closest target to previous target
                    if abs(self.last_target_person.bbox.center.x - detection.bbox.center.x) < closest_target_dist:
                        closest_target_dist = abs(self.last_target_person.bbox.center.x - detection.bbox.center.x)
                        target_person = detection

        self.last_target_person = target_person
        self.getDriveDirection(target_person)

    def dockCallback(self, msg: Dock):
        self.is_docked = msg.is_docked

    def setLed(self, led, color, period, duty):
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
                self.setLed(0, 0, 1000, 0.0)
                self.setLed(1, 2, 1000, 1.0)
            elif self.direction == self.FORWARD:
                self.drive(0.3, 0.0)
                self.setLed(0, 1, 1000, 1.0)
                self.setLed(1, 1, 1000, 1.0)
            elif self.direction == self.LEFT:
                self.drive(0.0, 0.3)
                self.previous_direction = self.LEFT
                self.setLed(0, 0, 1000, 0.5)
                self.setLed(1, 1, 1000, 0.5)
            elif self.direction == self.RIGHT:
                self.drive(0.0, -0.3)
                self.previous_direction = self.RIGHT
                self.setLed(0, 1, 1000, 0.5)
                self.setLed(1, 0, 1000, 0.5)
            elif self.direction == self.FORWARD_LEFT:
                self.drive(0.2, 0.2)
                self.previous_direction = self.LEFT
                self.setLed(0, 1, 1000, 1.0)
                self.setLed(1, 1, 1000, 0.5)
            elif self.direction == self.FORWARD_RIGHT:
                self.drive(0.2, -0.2)
                self.previous_direction = self.LEFT
                self.setLed(0, 1, 1000, 0.5)
                self.setLed(1, 1, 1000, 1.0)
            else:
                if self.previous_direction == self.LEFT:
                    self.drive(0.0, 0.75)
                    self.setLed(0, 0, 500, 0.5)
                    self.setLed(1, 1, 500, 0.5)
                else:
                    self.drive(0.0, -0.75)
                    self.setLed(0, 1, 500, 0.5)
                    self.setLed(1, 0, 500, 0.5)
            time.sleep(1/self.fps)


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
