#!/usr/bin/env python3
"""Docstring for the test_loader.py module.

@author Juntong
@date   2023-04-26

TODO: Document Here

"""

from gary_ui.loader import Loader

from time import sleep
from pprint import pprint


def main():
    loader = Loader("demo_yamls")
    for i in range(4):
        ls = loader.update()
        print(f"[{i}]")
        pprint(list(ls))
        sleep(0.4)


if __name__ == "__main__":
    main()
