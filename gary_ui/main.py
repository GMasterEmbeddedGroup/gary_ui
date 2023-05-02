#!/usr/bin/env python3
"""Docstring for the main.py module.

@author juntong
@date   2023-05-02

主文件

"""

from . import Loader, encode_iter

import rclpy
from rclpy.node import Node


class UiNode(Node):
    def __init__(self):
        super().__init__("gary_ui")

        self.update_counter = 0

        self.declare_parameter('ui_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('ui_cache_clear_times', 100)

        ui_path = self.get_parameter("ui_path")
        self.ui = Loader(ui_path.value)

        self.create_timer(0.2, self.update_ui)

    def update_ui(self):
        self.update_counter = (self.update_counter + 1) % self.get_parameter("ui_cache_clear_times").value
        for ui_obj in self.ui.update(clear_cache=self.update_counter == 0):
            for arr in encode_iter(ui_obj):
                self.publish_array(arr)

    def publish_array(self, arr):
        raise NotImplementedError


def run_ui_node():
    ui_node = UiNode()
    try:
        rclpy.spin(ui_node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    run_ui_node()
