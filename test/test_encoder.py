#!/usr/bin/env python3
"""Docstring for the test_encoder.py module.

@author Juntong
@date   2023-04-20

测试 encoder

"""
from gary_ui.ui_objects import Line
from gary_ui.encoder import encode_line
from gary_ui.hex_reader import byte_to_graph_data


def test_line_encoder():
    line = Line(name=b'012', colour=1, width=1, layout=8, start_x=0, start_y=0, end_x=10, end_y=12)
    arr = encode_line('add', line)
    hex_code = arr.tobytes().hex()
    return byte_to_graph_data(hex_code)


def main():
    graph = test_line_encoder()
    print("Line(name=b'012', colour=1, width=1, layout=8, start_x=0, start_y=0, end_x=10, end_y=12)\n->\n", graph)


if __name__ == "__main__":
    main()
