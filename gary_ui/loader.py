#!/usr/bin/env python3
"""Docstring for the loader.py module.

@author Juntong
@date   2023-04-26

Yaml 文件的加载器

"""
import itertools
import os
import re
from collections import OrderedDict
from dataclasses import dataclass
from functools import partial
from subprocess import getoutput as get_output  # 强迫症行为
from typing import Iterable, Optional, Callable, Any, Union

import yaml

from .ui_objects import Line, Cycle, Float, Rectangle, Sentence, get_name_generator, WidgetBasic


@dataclass()
class UiObject:
    name: str
    value: dict
    load_order: int
    depends: list

    def __le__(self, other):
        return self.load_order < other.load_order

    def __lt__(self, other):
        return self.load_order > other.load_order

    def __getattr__(self, item):
        return self.value[item]


def get_topic(name):
    """
    获取一个topic
    """
    raise NotImplementedError


class ReturnValue:
    """
    Just return what you gave.
    """

    def __init__(self, value):
        self.value = value

    def __repr__(self):
        return f"ReturnValue({self.value})"

    def __call__(self):
        return self.value


class CallChain:
    def __init__(self, func, v1, v2):
        self.func = func
        self.v1 = v1
        self.v2 = v2

    def __call__(self):
        return self.func(self.v1(), self.v2())


def return_value(value):
    """
    Just return what you gave.
    """
    return value


def return_called(func):
    """
    return func()
    """
    return func()


class UiDescription:
    BASIC_UI_WIDGETS = {"line": Line,
                        "cycle": Cycle,
                        "float": Float,
                        "rectangle": Rectangle,
                        "sentence": Sentence}

    RE_TOPIC = re.compile(r"^%(.+)$")
    RE_COMMAND = re.compile(r"^!(.+)$")
    RE_PYTHON = re.compile(r"^\$(.+)$")
    RE_READ = re.compile(r"(.*?)\((.+)\)(.*)")

    def __init__(self, name_generator, **objects):
        """
        加载单个 yaml 描述文件
        """
        self.missing_map = objects.get("__missing__", {})
        self.name_generator = name_generator

        ui_widgets: list[UiObject] = []  # XXX: 去掉了依赖检测, 为依赖检测添加的代码还在
        self.ui_objects: OrderedDict[str, UiObject] = OrderedDict()

        for name, value in objects.items():  # 注意这里的 name 不是 UI 元素里的 name, 而是上层的控件的 name
            if name[0] == "_":
                continue
            ui_obj = self.description_to_ui_object(name, value)
            ui_widgets.append(ui_obj)
        for ui_obj in sorted(ui_widgets, reverse=True):
            self.ui_objects[ui_obj.name] = ui_obj
            self.update_values(ui_obj, tuple(ui_obj.value.keys()))  # XXX: update_attrs 这玩意儿咋填啊
        # TODO

    def description_to_ui_object(self, obj_name, obj_value) -> UiObject:
        """

        """
        values = self.missing_map.copy()
        values.update(obj_value)
        obj_type = values["type"]
        assert obj_type is self.BASIC_UI_WIDGETS, NotImplementedError  # NOTE: 如添加自定义控件, 改这里的代码
        assert all(isinstance(values[key], (int, float)) for key in ())  # ???
        order = 0 if obj_type in self.BASIC_UI_WIDGETS else 100
        if values["name"] is None:
            values["name"] = next(self.name_generator)
        depends = []
        return UiObject(obj_name, obj_value, order, depends)

    def update_values(self, ui_object: UiObject, value_key_to_update: Iterable[str]):
        """
        更新 ui_object 的 depends 传入值 `update_attrs` 所指出的属性
        """
        for key in value_key_to_update:
            statement = getattr(ui_object, key)
            new = self.get_value(statement)
            setattr(ui_object, key, new)

    def get_attr_from_val(self, object_values: Union[dict[str: Any], UiObject, Callable], key: str):
        """
        从 UI 描述中返回 key 对应的属性值 (其实就是多了个 missing_map 查询)
        """
        if callable(object_values):
            object_values = object_values()
        if isinstance(object_values, UiObject):
            object_values = UiObject.value
        return object_values[key] if key in object_values else self.missing_map[key]

    def get_value(self, statement: str):
        """
        将表达式转换为新的属性
        """
        if m := self.RE_TOPIC.match(statement):  # 需要一个 Topic
            return partial(get_topic, m.group(1))

        if m := self.RE_COMMAND.match(statement):  # 执行一个命令
            return partial(get_output, m.group(1))

        if m := self.RE_PYTHON.match(statement):  # Python 格式化
            return self.read_python_value_statement(m.group(1))

        return statement

    def read_python_value_statement(self, statement: str) -> Callable:
        """
        将 python 格式化表达式转换为新的属性, 特殊处理了括号第一个元素为操作符的情况
        """

        return partial(statement.format_map, self.ui_objects)

    def update(self) -> Iterable[WidgetBasic]:
        """
        入口函数, 返回更新后的 UI 控件实例
        """
        ret = []
        for ui_obj in self.ui_objects.values():
            if not ui_obj.value["show"]:
                continue
            widget_cls = self.BASIC_UI_WIDGETS[ui_obj.value["type"]]
            widget_obj_keys = {}
            for key in set(ui_obj.value) - {"type", "show"}:
                val = ui_obj.value[key]
                widget_obj_keys[key] = val() if callable(val) else val
            ret.append(widget_cls(**widget_obj_keys))

        return ret


class Loader:
    """

    """
    IN_FILE_NAME_GENERATOR = get_name_generator()

    def __init__(self, dir_path: str,
                 name_generator: Optional[Iterable] = None, follow_links=True,
                 ui_description_loader: type = UiDescription):
        """

        """
        self.dir_path = dir_path
        self.loaders = []
        if name_generator is None:
            self.name_generate = self.IN_FILE_NAME_GENERATOR
        else:
            self.name_generate = name_generator

        walk = os.walk(dir_path, followlinks=follow_links)
        root, dirs, files = next(walk)
        for file in files:
            if file.endswith((".yaml", ".yml")):
                with open(os.path.join(root, file), "r", encoding="utf-8") as fp:
                    cfg = yaml.safe_load(fp)
                self.loaders.append(ui_description_loader(self.name_generate, **cfg))  # TODO: 支持自定义控件

    def update(self):
        """
        入口函数, 返回全部控件
        """
        return itertools.chain.from_iterable(leaders.update() for leaders in self.loaders)

    def exec_(self, env_local=None, env_global=None):
        """

        """
        if env_global is None:
            env_local = {}
        file = os.path.join(self.dir_path, "main.py")
        with open(file, "r", encoding="utf-8") as fp:
            py_code = fp.read()
        exec(py_code, env_global, env_local)
        return env_local, env_global
