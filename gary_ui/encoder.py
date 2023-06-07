"""
将图像类转换为数据
"""
import dataclasses
from copy import copy
from array import array
from typing import Union, Optional
from functools import singledispatch
from dataclasses import dataclass

from .ui_objects import Line, Rectangle, Cycle, Float, Sentence


def _fast_to_bin(num: int, length: int) -> str:
    """
    一个数字转二进制字符的函数 2 -> 10
    """
    return bin(num)[2:].rjust(length, "0")


# typedef __packed struct{
#         uint8_t graphic_name[3];
#         uint32_t operate_type:3;
#         uint32_t graphic_type:3;
#         uint32_t layer:4;
#         uint32_t color:4;
#         uint32_t start_angle:9;
#         uint32_t end_angle:9;
#         uint32_t width:10;
#         uint32_t start_x:11;
#         uint32_t start_y:11;
#         uint32_t radius:10;
#         uint32_t end_x:11;
#         uint32_t end_y:11;
#         } graphic_data_struct_t

def encode_basic(data: array, mode: Union[str, int],
                 obj: Union[Line, Rectangle, Cycle, Float, Sentence]) -> str:
    """
    转换图形结构的基础内容: 向 data 填入图形名称, 返回前 4 字节中的前 14 位数字 (包含 操作模式, 图形类型, 图层号, 颜色)
    :param data:
    :param mode: 模式：新增(1), 修改(2), 删除(3)
    :param obj:
    :return:
    """
    # 1. (3 * 8) 图形名
    data.extend(obj.name)  # name 长度恒为 3

    # 2. (4 * 8): 1 / 4
    # -- bit 5-8: 图形操作
    if isinstance(mode, str):
        try:
            mode = ("empty", "add", "modify", "delete").index(mode.lower())
        except ValueError as err:
            raise ValueError('need integer 0~3, or string ("empty", "add", "modify", "delete")') from err
    val: str
    # -- byte 1
    # operate type (3 / 3)
    val = _fast_to_bin(mode, 3)
    # graphic_type (3 /3)
    val = _fast_to_bin(obj.graphic_type, 3) + val
    # layer (2 / 4)
    layer_code = _fast_to_bin(obj.layout, 4)
    val = layer_code[-2:] + val
    data.append(int(val, 2))
    # -- byte 2
    # start_angle : pass
    # color (4 / 4)
    val = _fast_to_bin(obj.colour, 4)
    # layer (2 / 4)
    val = val + layer_code[:2]
    return val


@singledispatch
def encode(obj, mode):
    raise TypeError("Unknown UI object", type(obj))


@encode.register
def encode_line(line: Line, mode: Union[int, str] = 2) -> array:
    """
    :param mode: 模式：新增(1), 修改(2), 删除(3)
    :param line: 一条直线
    :return:
    """
    data = array("B", [])
    # 1. 4 byte
    val = encode_basic(data, mode, line)
    main_encoder(data, val, **dataclasses.asdict(line))

    return data


def main_encoder(data: array, head: str, mapping: Optional[dict] = None, **kwargs):
    """
    一个重构的尝试
    head: basic_encode 的返回值
    mapping: 将 kwargs 的某个值映射为对应的参数
    """
    keys = dict(start_angle=0, end_angle=0, width=0, start_x=0, start_y=0, radius=0, end_x=0, end_y=0)
    keys.update(kwargs)

    if mapping:
        for k, key in mapping.items():
            if k in kwargs:
                keys[key] = kwargs[k]

    start_angle_code = _fast_to_bin(keys["start_angle"], 9)
    end_angle_code = _fast_to_bin(keys["end_angle"], 9)
    width_code = _fast_to_bin(keys["width"], 10)
    start_x_code = _fast_to_bin(keys["start_x"], 11)
    start_y_code = _fast_to_bin(keys["start_y"], 11)
    radius_code = _fast_to_bin(keys["radius"], 10)
    end_x_code = _fast_to_bin(keys["end_x"], 11)
    end_y_code = _fast_to_bin(keys["end_y"], 11)

    extend = [int(i, 2) for i in [
        start_angle_code[-2:] + head,
        end_angle_code[-1] + start_angle_code[:7],
        end_angle_code[:-1],
        width_code[-8:],
        start_x_code[-6:] + width_code[:2],
        start_y_code[-3:] + start_x_code[:5],
        start_y_code[:8],
        radius_code[-8:],
        end_x_code[-6:] + radius_code[:2],
        end_y_code[-3:] + end_x_code[:5],
        end_y_code[:8],
    ]
              ]

    data.extend(extend)


@encode.register
def encode_rectangle(rectangle: Rectangle, mode: Union[int, str] = 2) -> array:
    """
    将矩形对象转码为绘制命令, 若想阅读代码, 请参考 "encode_line"
    :param mode:
    :param rectangle:
    :return:
    """
    data = array("B", [])
    val = encode_basic(data, mode, rectangle)
    main_encoder(data, val,
                 {"diagonal_vertex_x": "end_x", "diagonal_vertex_y": "end_y"},
                 **dataclasses.asdict(rectangle))

    return data


@encode.register
def encode_cycle(cycle: Cycle, mode: Union[int, str] = 2) -> array:
    """
    将正圆对象转码为绘制命令, 若想阅读代码, 请参考 "encode_line"
    :param mode:
    :param cycle:
    :return:
    """
    data = array("B", [])
    # 1. 4 byte
    val = encode_basic(data, mode, cycle)
    main_encoder(data, val,
                 {"centre_x": "start_x", "centre_y": "start_y"},
                 **dataclasses.asdict(cycle))

    return data


@encode.register
def encode_float(float_: Float, mode: Union[int, str] = 2) -> array:
    """
    将正圆对象转码为绘制命令, 若想阅读代码, 请参考 "encode_line"
    :param mode:
    :param float_:
    :return:
    """
    data = array("B", [])

    val = encode_basic(data, mode, float_)

    main_encoder(data, val,
                 {"font_size": "start_angle", "significant_digits": "end_angle"},
                 **dataclasses.asdict(float_))
    del data[-4:]
    data.extend(round(float_.value * 1000).to_bytes(4, 'little'))
    return data


def encode_sentence(sentence: Sentence, mode: int = 2) -> array:
    """
    将 Sentence 对象转码为绘制命令
    """
    data = array("B", [])

    val = encode_basic(data, mode, sentence)
    main_encoder(data, val,
                 {"font_size": "start_angle", "length": "end_angle"},
                 **dataclasses.asdict(sentence))

    # 4. the words
    data.extend(sentence.string[:sentence.length].encode("ascii"))
    # 补全剩余长度
    sentence_length = len(sentence.string) if sentence.length is None else sentence.length
    if sentence_length < 30:
        data.extend(bytes(30 - sentence_length))

    return data


def encode_iter(*args):
    """
    编码 UI 对象
    :param args: Iterable: (mode, object)
    :return:
    """
    ret = array("B")
    number = 0

    for obj, mode in args:
        if number > 7:
            yield copy(ret)
            ret.clear()
            number -= 7
        ret.extend(encode(obj, mode))
        number += 1

    mapping = {
        1: 0x0101,
        2: 0x0102,
        5: 0x0103,
        7: 0x0104
    }

    if number in mapping:
        yield number, ret

    elif number < 5:
        ret.extend(bytes(30 * (5 - number)))
        yield mapping[5], ret

    else:  # number == 6
        ret.extend(bytes(30))
        yield mapping[7], ret
