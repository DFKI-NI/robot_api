from typing import Dict, List, Tuple, Type, Union
import pytest
from robot_api import is_instance, get_at


def test_is_instance() -> None:
    assert is_instance(1, int)
    assert not is_instance(0, bool)
    assert is_instance(False, int)
    assert is_instance((1,), tuple)
    assert is_instance(
        [
            2,
        ],
        List,
    )
    assert not is_instance((3,), Tuple[str])
    assert not is_instance((4,), Tuple[int, str])
    assert is_instance((3,), Tuple[int])
    assert not is_instance(0, Tuple[int])
    assert is_instance({1: "one", 2: "two"}, dict)
    assert is_instance({1: "one", 2: "two"}, Dict[int, str])
    assert not is_instance({1: "one", 2: "two"}, Dict[str, int])
    assert is_instance(1, Union[int, str])
    assert is_instance({1: 2}, Dict)
    assert is_instance({}, Dict[Tuple[int, str], Union[List[int], List[str]]])
    assert is_instance(
        {(1, "one"): [0, 0]}, Dict[Tuple[int, str], Union[List[int], List[str]]]
    )
    assert is_instance(
        {(1, "one"): ["0", "0"]}, Dict[Tuple[int, str], Union[List[int], List[str]]]
    )
    assert not is_instance(
        {(1, "one"): [0, "0"]}, Dict[Tuple[int, str], Union[List[int], List[str]]]
    )
    assert is_instance(
        {(1, "one"): [0, "0"]}, Dict[Tuple[int, str], List[Union[int, str]]]
    )
    assert is_instance((1, 2, 3), Tuple[int, ...])
    assert is_instance(((1,), [2]), Tuple[tuple, list])
    assert is_instance(list, type)
    assert is_instance(type, type)
    with pytest.raises(NotImplementedError):
        assert is_instance(type, Type)


def test_get_at() -> None:
    args = [0, "one", (2, 3), {4: 5}]
    assert get_at(args, 0, int) == 0
    assert get_at(args, 0, float) is None
    assert get_at(args, 1, int) is None
    assert get_at(args, 2, Tuple[int, ...]) == (2, 3)
    assert get_at(args, 2, Tuple[Union[float, int], ...]) == (2, 3)
    assert get_at(args, 3, dict) == {4: 5}
    assert get_at(args, 3, Dict[Union[float, int], Union[float, int]]) == {4: 5}
    assert get_at(0, 1, 2) is None
    assert get_at("0", 0, str) == "0"
