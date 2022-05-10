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

from enum import Enum

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

# FollowBot state
class State(Enum):
    SEARCHING = 0
    FOLLOWING = 1
    TRACKING = 2
    REVERSING = 3

# Direction of target
class Direction(Enum):
    FORWARD = 0
    FORWARD_LEFT = 1
    FORWARD_RIGHT = 2
    LEFT = 3
    RIGHT = 4
    UNKNOWN = 5

class FollowBot(Node):
    image_width = 300
    image_height = 300
    fps = 15
    
    is_docked = False

    target = None
    target_direction = Direction.UNKNOWN
    target_distance = 0.0
    last_target_direction = Direction.UNKNOWN
    state = State.SEARCHING

    forward_margin_px = 20
    forward_turn_margin_px = 75
    reverse_thresh = 1.0
    follow_thresh = 1.5
    thresh_hysteresis = 0.05

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

    def getTargetDirection(self):
        self.last_target_direction = self.target_direction

        if self.target is None:
            self.target_direction = Direction.UNKNOWN
            return

        # Get distance of target to center of image
        center_dist = self.target.bbox.center.x - self.image_width / 2

        if abs(center_dist) < self.forward_margin_px:
            self.target_direction = Direction.FORWARD
        elif abs(center_dist) < self.forward_turn_margin_px:
            if center_dist >= 0.0:
                self.target_direction = Direction.FORWARD_RIGHT
            else:
                self.target_direction = Direction.FORWARD_LEFT
        else:
            if center_dist >= 0.0:
                self.target_direction = Direction.RIGHT
            else:
                self.target_direction = Direction.LEFT

    def stateMachine(self):
        # Searching
        if self.state == State.SEARCHING:
            # No target visible
            if self.target_direction == Direction.UNKNOWN:
                if self.last_target_direction == Direction.UNKNOWN or \
                   self.last_target_direction == Direction.RIGHT or \
                   self.last_target_direction == Direction.FORWARD_RIGHT or \
                   self.last_target_direction == Direction.FORWARD:
                    self.drive(0.0, -0.75)
                    self.setLed(0, 1, 500, 0.5)
                    self.setLed(1, 0, 500, 0.5)
                else:
                    self.drive(0.0, 0.75)
                    self.setLed(0, 0, 500, 0.5)
                    self.setLed(1, 1, 500, 0.5)
            else:
                if self.target_distance < self.reverse_thresh - self.thresh_hysteresis:
                    self.state = State.REVERSING
                elif self.target_distance > self.follow_thresh + self.thresh_hysteresis:
                    self.state = State.FOLLOWING
                else:
                    self.state = State.TRACKING
        elif self.state == State.FOLLOWING:
            # Lost track of target
            if self.target_direction == Direction.UNKNOWN:
                self.state = State.SEARCHING
            else:
                # Target is too close to follow
                if self.target_distance < self.follow_thresh - self.thresh_hysteresis:
                    self.state = State.TRACKING
                else:
                    if self.target_direction == Direction.FORWARD:
                        self.drive(0.3, 0.0)
                        self.setLed(0, 1, 1000, 1.0)
                        self.setLed(1, 1, 1000, 1.0)
                    elif self.target_direction == Direction.FORWARD_LEFT:
                        self.drive(0.2, 0.2)
                        self.setLed(0, 1, 1000, 1.0)
                        self.setLed(1, 1, 1000, 0.5)
                    elif self.target_direction == Direction.FORWARD_RIGHT:
                        self.drive(0.2, -0.2)
                        self.setLed(0, 1, 1000, 0.5)
                        self.setLed(1, 1, 1000, 1.0)
                    elif self.target_direction == Direction.RIGHT:
                        self.drive(0.0, -0.3)
                        self.setLed(0, 1, 1000, 0.5)
                        self.setLed(1, 0, 1000, 0.5)
                    else:
                        self.drive(0.0, 0.3)
                        self.setLed(0, 0, 1000, 0.5)
                        self.setLed(1, 1, 1000, 0.5)
        elif self.state == State.TRACKING:
            # Lost track of target
            if self.target_direction == Direction.UNKNOWN:
                self.state = State.SEARCHING
            else:
                if self.target_distance < self.reverse_thresh - self.thresh_hysteresis:
                    self.state = State.REVERSING
                elif self.target_distance > self.follow_thresh + self.thresh_hysteresis:
                    self.state = State.FOLLOWING
                else:
                    if self.target_direction == Direction.FORWARD:
                        self.drive(0.0, 0.0)
                        self.setLed(0, 0, 1000, 0.0)
                        self.setLed(1, 2, 1000, 1.0)
                    elif self.target_direction == Direction.FORWARD_LEFT:
                        self.drive(0.0, 0.2)
                        self.setLed(0, 1, 1000, 1.0)
                        self.setLed(1, 1, 1000, 0.5)
                    elif self.target_direction == Direction.FORWARD_RIGHT:
                        self.drive(0.0, -0.2)
                        self.setLed(0, 1, 1000, 0.5)
                        self.setLed(1, 1, 1000, 1.0)
                    elif self.target_direction == Direction.RIGHT:
                        self.drive(0.0, -0.3)
                        self.setLed(0, 1, 1000, 0.5)
                        self.setLed(1, 0, 1000, 0.5)
                    else:
                        self.drive(0.0, 0.3)
                        self.setLed(0, 0, 1000, 0.5)
                        self.setLed(1, 1, 1000, 0.5)

        elif self.state == State.REVERSING:
            # Lost track of target
            if self.target_direction == Direction.UNKNOWN:
                self.state = State.SEARCHING
            else:
                if self.target_distance > self.reverse_thresh + self.thresh_hysteresis:
                    self.state = State.TRACKING
                else:
                    if self.target_direction == Direction.FORWARD:
                        self.drive(-0.2, 0.0)
                        self.setLed(0, 0, 1000, 0.0)
                        self.setLed(1, 2, 1000, 1.0)
                    elif self.target_direction == Direction.FORWARD_LEFT:
                        self.drive(-0.2, 0.2)
                        self.setLed(0, 1, 1000, 1.0)
                        self.setLed(1, 1, 1000, 0.5)
                    elif self.target_direction == Direction.FORWARD_RIGHT:
                        self.drive(-0.2, -0.2)
                        self.setLed(0, 1, 1000, 0.5)
                        self.setLed(1, 1, 1000, 1.0)
                    elif self.target_direction == Direction.RIGHT:
                        self.drive(-0.1, -0.3)
                        self.setLed(0, 1, 1000, 0.5)
                        self.setLed(1, 0, 1000, 0.5)
                    else:
                        self.drive(-0.1, 0.3)
                        self.setLed(0, 0, 1000, 0.5)
                        self.setLed(1, 1, 1000, 0.5)

    def mobilenetCallback(self, msg: SpatialDetectionArray):
        closest_target_dist = self.image_width
        target = None
        if len(msg.detections) > 0:
            for detection in msg.detections:
                # Person detected
                if detection.results[0].class_id == '15' and detection.results[0].score > 0.90:
                    # No one previously detected, target first detection
                    if self.target is None:
                        target = detection
                        break
                    # Find closest target to previous target
                    if abs(self.target.bbox.center.x - detection.bbox.center.x) < closest_target_dist:
                        closest_target_dist = abs(self.target.bbox.center.x - detection.bbox.center.x)
                        target = detection

        if target is not None:
            self.target_distance = target.position.z
        self.target = target
        self.getTargetDirection()

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
            # if self.direction == self.STOP:
            #     self.drive(0.0, 0.0)
            #     self.setLed(0, 0, 1000, 0.0)
            #     self.setLed(1, 2, 1000, 1.0)
            # elif self.direction == self.FORWARD:
            #     self.drive(0.3, 0.0)
            #     self.setLed(0, 1, 1000, 1.0)
            #     self.setLed(1, 1, 1000, 1.0)
            # elif self.direction == self.LEFT:
            #     self.drive(0.0, 0.3)
            #     self.previous_direction = self.LEFT
            #     self.setLed(0, 0, 1000, 0.5)
            #     self.setLed(1, 1, 1000, 0.5)
            # elif self.direction == self.RIGHT:
            #     self.drive(0.0, -0.3)
            #     self.previous_direction = self.RIGHT
            #     self.setLed(0, 1, 1000, 0.5)
            #     self.setLed(1, 0, 1000, 0.5)
            # elif self.direction == self.FORWARD_LEFT:
            #     self.drive(0.2, 0.2)
            #     self.previous_direction = self.LEFT
            #     self.setLed(0, 1, 1000, 1.0)
            #     self.setLed(1, 1, 1000, 0.5)
            # elif self.direction == self.FORWARD_RIGHT:
            #     self.drive(0.2, -0.2)
            #     self.previous_direction = self.LEFT
            #     self.setLed(0, 1, 1000, 0.5)
            #     self.setLed(1, 1, 1000, 1.0)
            # else:
            #     if self.previous_direction == self.LEFT:
            #         self.drive(0.0, 0.75)
            #         self.setLed(0, 0, 500, 0.5)
            #         self.setLed(1, 1, 500, 0.5)
            #     else:
            #         self.drive(0.0, -0.75)
            #         self.setLed(0, 1, 500, 0.5)
            #         self.setLed(1, 0, 500, 0.5)
            self.stateMachine()
            print(self.state)
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
