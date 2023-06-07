"""
UI 协议中的结构体
"""
import itertools


class GraphData:
    def __init__(self, graphic_name, operate_type, graphic_type, layer, color_code, start_angle, end_angle,
                 width, start_x, start_y, radius, end_x, end_y):
        """
        一个图形数据类
        """
        self.graphic_name = graphic_name
        self.operate_type = operate_type
        self.graphic_type = graphic_type
        self.layer = layer
        self.color_code = color_code
        self.start_angle = start_angle
        self.end_angle = end_angle
        self.width = width
        self.start_x = start_x
        self.start_y = start_y
        self.radius = radius
        self.end_x = end_x
        self.end_y = end_y

    def __repr__(self):
        return "\n".join(itertools.chain([type(self).__name__ + "("],
                                         [f'graphic_name: "{self.graphic_name}"'],
                                         [i.ljust(12) + ": " + str(getattr(self, i)) for i in (
                                             'operate_type', 'graphic_type', 'layer', 'color_code', 'start_angle',
                                             'end_angle', 'width', 'start_x', 'start_y', "radius", "end_x", "end_y")]
                                         )) + ")"


def hex_str_to_bin_str(hex_code: str) -> str:
    mp = {"0": "0000",
          "1": "0001",
          "2": "0010",
          "3": "0011",
          "4": "0100",
          "5": "0101",
          "6": "0110",
          "7": "0111",
          "8": "1000",
          "9": "1001",
          "a": "1010",
          "b": "1011",
          "c": "1100",
          "d": "1101",
          "e": "1110",
          "f": "1111"}
    return "".join(mp[i] for i in hex_code)


def byte_to_graph_data(hex_code: str):
    if len(hex_code) > 30:
        raise ValueError
    hex_code = hex_code.ljust(30, " ")
    bin_code = hex_str_to_bin_str(hex_code)

    name_code = bin_code[:24]
    name = "".join(chr(int(i, 2)) for i in (name_code[:8], name_code[8:16], name_code[16:24]))

    layer_code_tail = bin_code[24:26]  # 2 / 4
    graphic_type_code = bin_code[26:29]  # 3
    operate_type_code = bin_code[29:32]  # 3

    start_angle_code_tail = bin_code[32:34]  # 2 / 9
    color_code = bin_code[34:38]  # 4
    layer_code_head = bin_code[38:40]  # 2 / 4

    end_angle_code_tail = bin_code[40:41]  # 1 / 9
    start_angle_code_head = bin_code[41:48]  # 7 / 9

    end_angle_code_head = bin_code[48:56]  # 8 / 9

    width_code_tail = bin_code[56:64]  # 8 / 10

    start_x_code_tail = bin_code[64:70]  # 6 / 11[
    width_code_head = bin_code[70:72]  # 2 / 10

    start_y_code_tail = bin_code[72: 75]  # 3 / 11
    start_x_code_head = bin_code[75: 80]  # 5 / 11

    start_y_code_head = bin_code[80:88]  # 8 / 11

    radius_code_tail = bin_code[88:96]  # 8 / 10

    end_x_code_tail = bin_code[96:102]  # 6 / 11
    radius_code_head = bin_code[102:104]  # 2 / 10

    end_y_code_tail = bin_code[104:107]  # 3 / 11
    end_x_code_head = bin_code[107:112]  # 5 / 11

    end_y_code_head = bin_code[112:120]  # 8 / 11

    print("-")

    operate_type = int(operate_type_code, 2)
    graphic_type = int(graphic_type_code, 2)
    layer = int(layer_code_head + layer_code_tail, 2)
    color = int(color_code, 2)
    start_angle = int(start_angle_code_head + start_angle_code_tail, 2)
    end_angle = int(end_angle_code_head + end_angle_code_tail, 2)
    width = int(width_code_head + width_code_tail, 2)
    start_x = int(start_x_code_head + start_x_code_tail, 2)
    start_y = int(start_y_code_head + start_y_code_tail, 2)
    radius = int(radius_code_head + radius_code_tail, 2)
    end_x = int(end_x_code_head + end_x_code_tail, 2)
    end_y = int(end_y_code_head + end_y_code_tail, 2)

    return GraphData(name, operate_type, graphic_type, layer, color, start_angle, end_angle,
                     width, start_x, start_y, radius, end_x, end_y)
