#!/usr/bin/env python3
from typing import Dict, List, Tuple, Type, Union
import pytest
from robot_api.core import _isinstance, _get_at


def test_isinstance() -> None:
    assert _isinstance(1, int)
    assert not _isinstance(0, bool)
    assert _isinstance(False, int)
    assert _isinstance((1,), tuple)
    assert _isinstance([2,], List)
    assert not _isinstance((3,), Tuple[str])
    assert not _isinstance((4,), Tuple[int, str])
    assert _isinstance((3,), Tuple[int])
    assert not _isinstance(0, Tuple[int])
    assert _isinstance({1: "one", 2: "two"}, dict)
    assert _isinstance({1: "one", 2: "two"}, Dict[int, str])
    assert not _isinstance({1: "one", 2: "two"}, Dict[str, int])
    assert _isinstance(1, Union[int, str])
    assert _isinstance({1: 2}, Dict)
    assert _isinstance({}, Dict[Tuple[int, str], Union[List[int], List[str]]])
    assert _isinstance({(1, "one"): [0, 0]}, Dict[Tuple[int, str], Union[List[int], List[str]]])
    assert _isinstance({(1, "one"): ["0", "0"]}, Dict[Tuple[int, str], Union[List[int], List[str]]])
    assert not _isinstance({(1, "one"): [0, "0"]}, Dict[Tuple[int, str], Union[List[int], List[str]]])
    assert _isinstance({(1, "one"): [0, "0"]}, Dict[Tuple[int, str], List[Union[int, str]]])
    assert _isinstance((1, 2, 3), Tuple[int, ...])
    assert _isinstance(((1,), [2]), Tuple[tuple, list])
    assert _isinstance(list, type)
    assert _isinstance(type, type)
    with pytest.raises(NotImplementedError):
        assert _isinstance(type, Type)


def test_get_at() -> None:
    args = [0, "one", (2, 3), {4: 5}]
    assert _get_at(args, 0, int) == 0
    assert _get_at(args, 1, int) is None
    assert _get_at(args, 2, Tuple[int, ...]) == (2, 3)
    assert _get_at(args, 3, dict) == {4: 5}
    assert _get_at(0, 1, 2) is None
    assert _get_at("0", 0, str) == "0"
