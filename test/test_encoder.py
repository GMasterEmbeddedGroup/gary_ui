#!/usr/bin/env python3
"""Docstring for the test_encoder.py module.

@author Juntong
@date   2023-04-20

测试 encoder

"""
from gary_ui.ui_objects import Line, Rectangle, Cycle, Float
from gary_ui.encoder import encode_line, encode_rectangle, encode_cycle, encode_float
from gary_ui.hex_reader import byte_to_graph_data

line = Line(name=b'012',
            colour=1,
            width=1,
            layout=8,
            start_x=0,
            start_y=0,
            end_x=10,
            end_y=12)
rectangle = Rectangle(name=b"out",
                      colour=4,
                      width=5,
                      layout=2,
                      start_x=1,
                      start_y=17,
                      diagonal_vertex_x=18,
                      diagonal_vertex_y=33)
cycle = Cycle(name=b"oka",
              colour=3,
              width=6,
              layout=8,
              centre_x=550,
              centre_y=678,
              radius=90)
float_ = Float(name=b"flt",
               colour=1,
               width=5,
               font_size=10,
               significant_digits=4,
               value=1.5,
               start_x=100,
               start_y=111,
               layout=3,
               )


def test_line_encoder():
    arr = encode_line(line, "add")
    hex_code = arr.tobytes().hex()
    return byte_to_graph_data(hex_code)


def test_rectangle_encoder():
    arr = encode_rectangle(rectangle, "add")
    hex_code = arr.tobytes().hex()
    return byte_to_graph_data(hex_code)


def test_cycle_encoder():
    arr = encode_cycle(cycle, "add")
    hex_code = arr.tobytes().hex()
    return byte_to_graph_data(hex_code)


def test_float_encoder():
    """
    理想状态返回:
    GraphData(
    graphic_name: "flt"
    operate_type: 1
    graphic_type: 5
    layer       : 3
    color_code  : 1
    start_angle : 10
    end_angle   : 4
    width       : 5
    start_x     : 100
    start_y     : 111
    radius      : 476
    end_x       : 1
    end_y       : 0)

    """
    arr = encode_float(float_, "add")
    hex_code = arr.tobytes().hex()
    return byte_to_graph_data(hex_code)


def main():
    print(f"{line}\n->\n", test_line_encoder())
    print(f"{rectangle}\n->\n", test_rectangle_encoder())
    print(f"{cycle}\n->\n", test_cycle_encoder())
    print(f"{float_}\n->\n", test_float_encoder())


if __name__ == "__main__":
    main()
