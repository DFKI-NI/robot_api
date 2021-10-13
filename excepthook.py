#!/usr/bin/env python3
#
# BSD Zero Clause License:
#
# Copyright (C) 2021 by Alexander Sung <Alexander.Sung@dfki.de>
#
# Permission to use, copy, modify, and/or distribute this software
# for any purpose with or without fee is hereby granted.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD
# TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT,
# OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
# DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
# ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
from typing import Optional, Type
from types import TracebackType
import sys


_sys_excepthook = sys.excepthook
_expected_exception = None  # type: Optional[BaseException]


def _excepthook(exception_type: Type[BaseException], exception_value: BaseException,
        traceback: TracebackType) -> None:
    if exception_value == _expected_exception:
        # Print exception without traceback for _expected_exception.
        print(f"{exception_type.__name__}: {exception_value}")
    else:
        # Otherwise, use default sys.excepthook.
        _sys_excepthook(exception_type, exception_value, traceback)


def expect_exception(exception: BaseException) -> BaseException:
    """Suppress traceback if exception is raised as expected. Otherwise, use sys.excepthook by default."""
    global _expected_exception
    _expected_exception = exception
    sys.excepthook = _excepthook
    return exception
