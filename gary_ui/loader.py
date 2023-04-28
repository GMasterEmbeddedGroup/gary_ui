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

    def get_attr_from_val(self, object_values: Union[dict[str: Any], UiObject], key: str):
        """
        从 UI 描述中返回 key 对应的属性值 (其实就是多了个 missing_map 查询)
        """
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
        if m := self.RE_READ.match(statement):
            sta, mid, end = m.groups()
            call = self.read_shell_like_statement(mid)
            if sta and end:  # a + (XXX) - b
                sta_statements_list = list(shlex.shlex(sta))
                end_statements_list = list(shlex.shlex(end))

                left_o = sta_statements_list[-1]
                right_o = end_statements_list[0]

        statements = shlex.shlex(statement)
        return self._read_shell_like_statements(statements)

    def _read_shell_like_statements(self, statements: SupportsNext[str]) -> Callable:
        """
        将 类shell 表达式转换为新的属性, 特殊处理了第一个元素为操作符的情况
        """
        first = next(statements)
        if first in self.STATEMENT_OPERATORS:
            statements = chain((None, first), statements)
        return self._read_shell_like_statement(statements, first)

    def _read_shell_like_statement(self, statements: SupportsNext[str],
                                   last_value: Union[str, partial, CallChain, ReturnValue],
                                   last_operator_precedence=-1) -> (Callable, Optional[SupportsNext]):
        """
        将 类shell 表达式转换为新的属性
        """
        try:
            this_operator_func, this_operator_precedence = self.STATEMENT_OPERATORS[next(statements)]
        except StopIteration:
            return (self.read_base_shell_like_statement(last_value) if isinstance(last_value, str) else last_value,
                    None)
        except KeyError as err:
            raise ValueError("Unknown operator:", err.args[0])

        try:
            this_value = next(statements)
        except StopIteration:
            return partial(this_operator_func, last_value, None), None

        try:
            next_symbol = next(statements)
        except StopIteration:
            return partial(this_operator_func, last_value, this_value), None

        assert next_symbol in self.STATEMENT_OPERATORS

        next_operator_func, next_operator_precedence = self.STATEMENT_OPERATORS[next_symbol]
        if next_operator_precedence > this_operator_precedence:
            next_symbol, statements = self._read_shell_like_statement(statements, this_value, this_operator_precedence)
        elif next_operator_precedence <= last_operator_precedence:
            this_value = self.read_base_shell_like_statement(last_value)
            return partial(this_operator_func, last_value(this_value)), statements
        return self._read_shell_like_statement(statements,
                                               partial(this_operator_func, last_value, this_value))

        # TODO: 1

    @staticmethod
    def read_base_shell_like_statement(statement: str) -> ReturnValue:
        if statement.isnumeric():
            return ReturnValue(int(statement))
        raise

    def description_to_ui_object(self, obj_name, obj_value) -> UiObject:
        """

        """
        values = self.missing_map.copy()
        values.update(obj_value)
        obj_type = values["type"]
        assert obj_type is self.BASIC_UI_WIDGETS, NotImplementedError
        assert all(isinstance(values[key], (int, float)) for key in ())
        order = 0 if obj_type in self.BASIC_UI_WIDGETS else 100
        depends = []
        return UiObject(obj_name, obj_value, order, depends)


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
