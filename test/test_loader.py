#!/usr/bin/env python3
"""Docstring for the test_loader.py module.

@author Juntong
@date   2023-04-26

TODO: Document Here

"""

from gary_ui.loader import Loader


def main():
    loader = Loader("demo_yamls")
    ls = loader.update()
    print(list(ls))
    return ls


if __name__ == "__main__":
    main()
