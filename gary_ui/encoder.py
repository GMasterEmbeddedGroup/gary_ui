"""
将图像类转换为数据
"""
from copy import copy
from array import array
from typing import Union
from functools import singledispatch

from .ui_objects import Line, Rectangle, Cycle, Float, Sentence


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
                 obj: Union[Line, Rectangle, Cycle, Float, Sentence]) -> int:
    """
    转换图形结构的基础内容: 向 data 填入图形名称, 返回前 4 字节中的前 14 位数字 (包含 操作模式, 图形类型, 图层号, 颜色)
    :param data:
    :param mode: 模式：新增(1), 修改(2), 删除(3)
    :param obj:
    :return:
    """
    # 1. (3 * 8) 图形名
    data.extend(obj.name)  # name 长度恒为 3

    # 2. (4 * 8)
    # -- bit 0-2: 图形操作
    if isinstance(mode, str):
        try:
            mode = ("empty", "add", "modify", "delete").index(mode.lower())
        except ValueError as err:
            raise ValueError('need integer 0~3, or string ("empty", "add", "modify", "delete")') from err
    val = mode
    # -- bit 3-5: 图形类型
    val = (val << 3) + obj.graphic_type
    # -- bit 6-9: 图层数, 0~9
    val = (val << 4) + obj.layout
    # -- bit 10-13: 颜色
    val = (val << 4) + obj.colour
    # Note: 位移运算符比加号的优先级低!!! 我可去你的吧 CSDN
    return val


@singledispatch
def encode(mode, obj):
    raise TypeError("Unknown UI object", type(obj))


@encode.register
def encode_line(mode: int, line: Line) -> array:
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
    val <<= 18
    data.extend(val.to_bytes(4, "big"))

    # 2. 4 byte
    # -- bit 0-9: 线宽
    val = line.width
    # -- bit 10-20: 起点 x 坐标
    val = (val << 11) + line.start_x
    # -- bit 21-31: 起点 y 坐标
    val = (val << 11) + line.start_y
    data.extend(val.to_bytes(4, "big"))

    # 3. 4 byte
    # -- bit 0-9: 字体大小或者半径
    val = 0
    # -- bit 10-20: 终点 x 坐标
    val = (val << 11) + line.end_x
    # -- bit 21-31: 终点 y 坐标
    val = (val << 11) + line.end_y
    data.extend(val.to_bytes(4, "big"))

    return data


@encode.register
def encode_rectangle(mode: int, rectangle: Rectangle) -> array:
    """
    将矩形对象转码为绘制命令, 若想阅读代码, 请参考 "encode_line"
    :param mode:
    :param rectangle:
    :return:
    """
    data = array("B", [])
    # 1. 4 byte
    val = encode_basic(data, mode, rectangle) << 18
    data.extend(val.to_bytes(4, "big"))

    # 2. 4 byte
    # -- bit 0-9: 线宽
    val = (((rectangle.width << 11) + rectangle.start_x) << 11) + rectangle.start_y
    data.extend(val.to_bytes(4, "big"))

    # 3. 4 byte
    # -- bit 0-9: 字体大小或者半径
    # -- bit 10-20: 终点 x 坐标
    val = (rectangle.diagonal_vertex_x << 11) + rectangle.diagonal_vertex_y
    data.extend(val.to_bytes(4, "big"))

    return data


@encode.register
def encode_cycle(mode: int, cycle: Cycle) -> array:
    """
    将正圆对象转码为绘制命令, 若想阅读代码, 请参考 "encode_line"
    :param mode:
    :param cycle:
    :return:
    """
    data = array("B", [])
    # 1. 4 byte
    val = encode_basic(data, mode, cycle) << 18
    data.extend(val.to_bytes(4, "big"))

    # 2. 4 byte
    # -- bit 0-9: 线宽
    val = (((cycle.width << 11) + cycle.centre_x) << 11) + cycle.centre_y
    data.extend(val.to_bytes(4, "big"))

    # 3. 4 byte
    # -- bit 0-9: 字体大小或者半径
    val = cycle.radius << 22
    data.extend(val.to_bytes(4, "big"))

    return data


@encode.register
def encode_float(mode: int, float_: Float) -> array:
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
    data.extend(val.to_bytes(4, "big"))

    # 2. 4 byte
    val = (((float_.width << 11) + float_.start_x) << 11) + float_.start_y
    data.extend(val.to_bytes(4, "big"))

    # 3. 4 byte
    val = int(round(float_.value, 4) * 1000) % (2 ** 31) * (-1 if val < 0 else 1)
    data.extend(val.to_bytes(4, "big"))

    return data


def encode_sentence(mode: int, sentence: Sentence) -> array:
    """
    将 Sentence 对象转码为绘制命令
    """
    data = array("B", [])
    # 1. 4 byte
    val = encode_basic(data, mode, sentence)
    # -- bit 14-22: 字体大小 and 字符长度
    val = (((val << 9) + sentence.font_size) << 9) + sentence.length
    data.extend(val.to_bytes(4, "big"))

    # 2. 4 byte
    val = (((sentence.width << 11) + sentence.start_x) << 11) + sentence.start_y
    data.extend(val.to_bytes(4, "big"))

    # 3. 4 byte
    data.extend(bytes(4))

    # 4. the words
    data.extend(sentence.string[:sentence.length])
    # 补全剩余长度
    sentence_length = len(sentence.string) if sentence.length is None else sentence.length
    if sentence_length < 30:
        data.extend(bytes(30 - sentence_length))

    return data


def encode_iter(*args) -> array:
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
        ret.extend(encode(mode, obj))
        number += 1

    if number in (7, 5, 2, 1):
        yield ret

    elif number < 5:
        ret.extend(bytes(30 * (5 - number)))
        yield ret

    else:  # number == 6
        ret.extend(bytes(30))
        yield ret
