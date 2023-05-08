#!/usr/bin/env python3
"""Docstring for the main.py module.

@author juntong
@date   2023-05-02

主文件

"""

from . import Loader, encode_iter

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from gary_msgs.msg import InteractiveDataSend, RobotStatus


class UiNode(Node):
    SENDER_RECEIVER_MAPPING = {1: 0x101,
                               2: 0x102,
                               3: 0x103,
                               4: 0x104,
                               5: 0x105,

                               101: 0x165,
                               102: 0x166,
                               103: 0x167,
                               104: 0x168,
                               105: 0x169}

    def __init__(self):
        super().__init__("gary_ui")

        self.update_counter = 0
        self.robot_id = 0

        self.declare_parameter('ui_path', "/home/gmaster/ros2_ws/src/gary/gary_ui/test/demo_yamls")
        self.declare_parameter('ui_cache_clear_times', 100)

        self.declare_parameter('ui_define_priority', 1)
        self.declare_parameter('ui_define_valid_time', 0.2)

        self.robot_state_listener = self.create_subscription(RobotStatus,
                                                             "/referee/robot_status",
                                                             self.robot_status_callback,
                                                             10)
        self.data_publisher = self.create_publisher(InteractiveDataSend,
                                                    "/referee/interactive_data_send",
                                                    3)

        ui_path = self.get_parameter("ui_path")
        self.ui = Loader(ui_path.value)

        self.create_timer(0.2, self.update_ui)

    def robot_status_callback(self, msg: RobotStatus):
        self.robot_id = msg.robot_id

    def update_ui(self):
        self.update_counter = (self.update_counter + 1) % self.get_parameter("ui_cache_clear_times").value
        for ui_obj in self.ui.update(clear_cache=self.update_counter == 0):
            print(ui_obj)
            with open("/home/gmaster/ros2_ws/src/gary/gary_ui/log.txt", "a") as fp:
                fp.write(repr(ui_obj))
            for data_cmd_id, arr in encode_iter(ui_obj):
                with open("/home/gmaster/ros2_ws/src/gary/gary_ui/log.txt", "a") as fp:
                    fp.write(repr(arr))
                self.publish_array(data_cmd_id, arr)
        with open("/home/gmaster/ros2_ws/src/gary/gary_ui/log.txt", "a") as fp:
            fp.write("--- %.2f ---" % time.time())

    def publish_array(self, data_cmd_id, arr):
        stamp = Time(sec=int(time.time()), nanosec=0)
        header = Header(stamp=stamp, frame_id="ui")
        valid_time = self.get_parameter("ui_define_valid_time")
        priority = self.get_parameter("ui_define_priority")
        msg = InteractiveDataSend(header=header,
                                  priority=priority.value,
                                  valid_time=valid_time.value,
                                  data_cmd_id=data_cmd_id,
                                  sender_id=self.robot_id,
                                  receiver_id=self.SENDER_RECEIVER_MAPPING[self.robot_id],
                                  # data_length=len(arr),
                                  data=arr)
        self.data_publisher.publish(msg)


def run_ui_node():
    rclpy.init()
    ui_node = UiNode()
    try:
        rclpy.spin(ui_node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    run_ui_node()
