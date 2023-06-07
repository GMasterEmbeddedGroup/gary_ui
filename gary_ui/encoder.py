"""
将图像类转换为数据
"""
from copy import copy
from array import array
from typing import Union
from functools import singledispatch

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
    # -- bit 14-22: 起始角度, 单位: °, 范围 [0, 360]
    # -- bit 23-31: 终止角度, 单位: °, 范围 [0, 360]
    data.append(int(val, 2))
    data.append(0)
    data.append(0)

    # byte 5
    # -- bit 0-8: 线宽 (8 / 10)
    data.append(line.width & 0b0011111111)
    # byte 6
    # -- bit 0-6: 起点 x 坐标 (6 / 11)
    val = line.start_x & 0x00000111111
    # -- bit 6-8: 线宽 (2 / 10)
    val = (val << 2) + (line.width >> 8)
    data.append(val)
    # byte 7
    # -- bit 0-3: 起点 y 坐标 (3 / 11)
    val = line.start_y & 0b111
    # -- bit 3-8: 起点 x 坐标 (5 / 11)
    val = (val << 5) + (line.start_x >> 6)
    data.append(val)
    # byte 8
    # -- 起点 y 坐标 (8 / 11)
    data.append(line.start_y >> 3)

    # byte 9
    # -- bit 0-8: 字体大小或者半径 (8 / 10)
    data.append(0)
    # byte 10
    # -- bit 0-6: 终点 x 坐标 (6 / 11)
    val = line.end_x & 0x00000111111
    # -- bit 6-8: 字体大小或者半径 (2 / 10)
    val <<= 2
    data.append(val)
    # byte 11
    # -- bit 0-3: 终点 y 坐标 (3 / 11)
    val = line.end_y & 0b111
    # -- bit 3-8: 终点 x 坐标 (5 / 11)
    print(line.end_x)
    val = (val << 5) + (line.end_x >> 6)
    data.append(val)
    # -- bit 24-32: 终点 y 坐标 (8 / 11)
    print(line.end_y)
    data.append(line.end_y >> 3)

    return data


@encode.register
def encode_rectangle(rectangle: Rectangle, mode: int = 2) -> array:
    """
    将矩形对象转码为绘制命令, 若想阅读代码, 请参考 "encode_line"
    :param mode:
    :param rectangle:
    :return:
    """
    data = array("B", [])
    # 1. 4 byte
    val = encode_basic(data, mode, rectangle) << 18
    data.extend(val.to_bytes(4, "little"))

    # 2. 4 byte
    # -- bit 0-9: 线宽
    val = (((rectangle.width << 11) + rectangle.start_x) << 11) + rectangle.start_y
    data.extend(val.to_bytes(4, "little"))

    # 3. 4 byte
    # -- bit 0-9: 字体大小或者半径
    # -- bit 10-20: 终点 x 坐标
    val = (rectangle.diagonal_vertex_x << 11) + rectangle.diagonal_vertex_y
    data.extend(val.to_bytes(4, "little"))

    return data


@encode.register
def encode_cycle(cycle: Cycle, mode: int = 2) -> array:
    """
    将正圆对象转码为绘制命令, 若想阅读代码, 请参考 "encode_line"
    :param mode:
    :param cycle:
    :return:
    """
    data = array("B", [])
    # 1. 4 byte
    val = encode_basic(data, mode, cycle) << 18
    data.extend(val.to_bytes(4, "little"))

    # 2. 4 byte
    # -- bit 0-9: 线宽
    val = (((cycle.width << 11) + cycle.centre_x) << 11) + cycle.centre_y
    data.extend(val.to_bytes(4, "little"))

    # 3. 4 byte
    # -- bit 0-9: 字体大小或者半径
    val = cycle.radius << 22
    data.extend(val.to_bytes(4, "little"))

    return data


@encode.register
def encode_float(float_: Float, mode: int = 2) -> array:
    """
    将正圆对象转码为绘制命令, 若想阅读代码, 请参考 "encode_line"
    :param mode:
    :param float_:
    :return:
    """
    data = array("B", [])
    # 1. 4 byte
    val = encode_basic(data, mode, float_)
    # -- bit 14-22: 字体大小
    val = (val << 9) + float_.font_size
    # -- bit 23-31: 小数位有效个数
    val = (val << 9) + float_.significant_digits
    data.extend(val.to_bytes(4, "little"))

    # 2. 4 byte
    val = (((float_.width << 11) + float_.start_x) << 11) + float_.start_y
    data.extend(val.to_bytes(4, "little"))

    # 3. 4 byte
    val = int(round(float_.value, 4) * 1000) % (2 ** 31) * (-1 if val < 0 else 1)
    data.extend(val.to_bytes(4, "little"))

    return data


def encode_sentence(sentence: Sentence, mode: int = 2) -> array:
    """
    将 Sentence 对象转码为绘制命令
    """
    data = array("B", [])
    # 1. 4 byte
    val = encode_basic(data, mode, sentence)
    # -- bit 14-22: 字体大小 and 字符长度
    val = (((val << 9) + sentence.font_size) << 9) + sentence.length
    data.extend(val.to_bytes(4, "little"))

    # 2. 4 byte
    val = (((sentence.width << 11) + sentence.start_x) << 11) + sentence.start_y
    data.extend(val.to_bytes(4, "little"))

    # 3. 4 byte
    data.extend(bytes(4))

    # 4. the words
    data.extend(sentence.string[:sentence.length])
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
