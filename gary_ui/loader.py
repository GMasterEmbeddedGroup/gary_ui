#!/usr/bin/env python3
"""Docstring for the loader.py module.

@author Juntong
@date   2023-04-26

Yaml 文件的加载器

"""
from _typeshed import SupportsNext

from .ui_objects import Line, Cycle, Float, Rectangle, Sentence, get_name_generator

import yaml

import os
import re
import shlex
from itertools import chain
from functools import partial
from dataclasses import dataclass
from collections import OrderedDict
from subprocess import getoutput as get_output  # 强迫症行为
from typing import Iterable, Optional, Callable, Any, Union


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
    RE_READ = re.compile(r"(.*?)\((.+)\)(.*)")

    def __init__(self, **objects):
        """
        标准操作 "." 将被自动设置为 (self.operator_point, 10)
        """
        self.missing_map = objects.get("__missing__", {})
        ui_widgets: list[UiObject] = []
        self.ui_objects: OrderedDict[str, UiObject] = OrderedDict()

        for name, value in objects.items():
            if name[0] == "_":
                continue
            ui_obj = self.description_to_ui_object(name, value)
            ui_widgets.append(ui_obj)
        for ui_obj in sorted(ui_widgets, reverse=True):
            self.ui_objects[ui_obj.name] = ui_obj
        # TODO

    def operator_point(self, left: str, right: str):
        if left.isnumeric():
            return float(f"{left}.{right}")
        return self.ui_objects[left].value[right]

    def update_value(self, ui_object: UiObject, update_attrs: Iterable[str]):
        """

        """
        for name in update_attrs:
            statement = getattr(ui_object, name)
            new, here = self.get_value(statement)

        return

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
            return partial(get_topic, m.group(1)), get_topic(m.group(1))

        if m := self.RE_COMMAND.match(statement):  # 执行一个命令
            return partial(get_output, m.group(1)), get_output(m.group(1))

        func = self.read_shell_like_statement(statement)
        return func, func()

    def read_shell_like_statement(self, statement: str) -> Callable:
        """
        将 类shell 表达式转换为新的属性, 特殊处理了括号第一个元素为操作符的情况
        """
        statements = shlex.shlex(statement)
        statements_new = []
        mapping = {}
        name_generator = get_name_generator()
        for char in statements:
            if char == "." and not statements_new[-1].isnumeric():
                name = next(name_generator)
                mapping[name] = partial(self.get_attr_from_val,
                                        partial(self.ui_objects.get, statements_new[-1]),
                                        next(statements))
                statements_new[-1] = name + "()"
            else:
                statements_new.append(char)
        return partial(eval, "".join(statements_new), None, mapping)


class Loader:
    """

    """
    IN_FILE_NAME_GENERATOR = get_name_generator()

    def __init__(self, dir_path: str,
                 name_generator: Optional[Iterable] = None, follow_links=True,
                 ui_description_loader: Callable = UiDescription):
        """

        """
        self.dir_path = dir_path
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
                ui_description_loader(**cfg)  # TODO

    def update(self):
        """

        """

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
