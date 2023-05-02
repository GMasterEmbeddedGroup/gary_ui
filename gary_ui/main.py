#!/usr/bin/env python3
"""Docstring for the main.py module.

@author juntong
@date   2023-05-02

主文件

"""

from . import Loader

import rclpy
from rclpy.node import Node


class UiNode(Node):
    def __init__(self):
        super().__init__("gary_ui")

        self.declare_parameter('ui_path', rclpy.Parameter.Type.STRING)

        ui_path = self.get_parameter("ui_path")
        self.ui = Loader(ui_path.value)

    def update_ui(self):
        for ui_obj in self.ui.update():
            ...  # TODO


def run_ui_node():
    ui_node = UiNode()
    try:
        rclpy.spin(ui_node)
    finally:
        rclpy.shutdown()
