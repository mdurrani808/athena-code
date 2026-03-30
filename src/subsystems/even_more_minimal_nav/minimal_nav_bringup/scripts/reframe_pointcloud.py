#!/usr/bin/env python3
# Copyright (c) 2025 UMD Loop
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

"""reframe_pointcloud.py

Subscribes to a PointCloud2 topic and republishes each message with
header.frame_id replaced by a target frame name.

This is used in simulation to relabel the ZED depth sensor frame
(athena/base_footprint/zed_depth_sensor) as base_link so that
pointcloud_to_laserscan needs no TF lookup.

Parameters
----------
  input_topic   (str, default /zed/zed_node/point_cloud/cloud_registered)
  output_topic  (str, default /zed/cloud_base_link)
  target_frame  (str, default base_link)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2


class ReframePointcloud(Node):
    def __init__(self):
        super().__init__('reframe_pointcloud')

        self.declare_parameter('input_topic',  '/zed/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('output_topic', '/zed/cloud_base_link')
        self.declare_parameter('target_frame', 'base_link')

        input_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.pub = self.create_publisher(PointCloud2, output_topic,
                                         QoSPresetProfiles.SENSOR_DATA.value)

        self.sub = self.create_subscription(PointCloud2, input_topic,
                                             self._cb,
                                             QoSPresetProfiles.SENSOR_DATA.value)

        self.get_logger().info(
            f'reframe_pointcloud: {input_topic} → {output_topic} '
            f'(frame_id → {self.target_frame})'
        )

    def _cb(self, msg: PointCloud2):
        msg.header.frame_id = self.target_frame
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReframePointcloud()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
