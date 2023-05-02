"""
"宗教审判廷"提醒您: 请使用英式英语, 如使用 "colour" 而不是 "color", 使用 "centre" 而不是 "center"
"""
from __future__ import annotations

from array import array
from dataclasses import dataclass
from itertools import combinations_with_replacement


def get_name_generator(word_range=range(ord('a'), ord('z') + 1)):
    """
    生成一个迭代器, 这个迭代器可以通过排列组合生成有效的字符名称.
    """
    for i in combinations_with_replacement(word_range, 3):
        yield bytes(i)


class WidgetBasic:
    pass


@dataclass(frozen=True)
class GraphicBasic(WidgetBasic):
    name: bytes  # XXX: 添加 default_factory 默认值工厂函数
    # graphic_type: int
    colour: int
    width: int  # 宽度, 范围是 0~9
    layout: int  # 图层 (范围 0~9)

    def __post_init__(self):
        """
        在执行 __init__ 后被调用的魔术方法, 用于执行参数检查
        :return:
        """
        assert len(self.name) == 3, ValueError("Graphic name need == 3 bytes")
        assert isinstance(self.name, bytes), TypeError("name need a bytes-type value, got", type(self.name))


@dataclass(frozen=True)  # Note: 使用 frozen 时会有性能损失
class Line(GraphicBasic):
    """
    直线类
    """
    start_x: int
    start_y: int
    end_x: int
    end_y: int
    graphic_type = 0


@dataclass(frozen=True)
class Rectangle(GraphicBasic):
    """
    矩形类
    """
    start_x: int
    start_y: int
    diagonal_vertex_x: int  # 对角顶点坐标 X
    diagonal_vertex_y: int  # 对角顶点坐标 Y
    graphic_type = 1


@dataclass(frozen=True)
class Cycle(GraphicBasic):
    """
    正圆类
    """
    centre_x: int
    centre_y: int
    radius: int
    graphic_type = 2


@dataclass(frozen=True)
class Float(GraphicBasic):
    """
    浮点数
    """
    start_x: int
    start_y: int
    font_size: int
    significant_digits: int
    value: float
    graphic_type = 3


@dataclass(frozen=True)
class Sentence(WidgetBasic):
    """
    字符, 注意此类型不是 GraphicBasic 的子类
    """
    name: bytes  # XXX: 添加 default_factory 默认值工厂函数
    colour: int
    width: int  # 宽度, 范围是 0~9
    layout: int  # 图层 (范围 0~9)
    start_x: int
    start_y: int
    font_size: int

    string: str
    length: None | int = None
    graphic_type = 7

    def __post_init__(self):
        # assert self.string.typecode == "B", TypeError("Sentence.string should be B-type array.")
        assert self.length is None or self.length > len(self.string), ValueError(
            "Sentence object property `length` can't > len(self.string). Need < %d, got %d.",
            (len(self.string), self.length))
        assert len(self.string) <= 30, ValueError("The string of sentence can't longer than 30.")
