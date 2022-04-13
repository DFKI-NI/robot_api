#!/usr/bin/env python3
import math
from robot_api import get_angle_between


def test_get_angle_between() -> None:
    assert get_angle_between(1.0, 2.0) == -get_angle_between(2.0, 1.0) == 1.0
    assert get_angle_between(1.0, 4.0) == -get_angle_between(4.0, 1.0) == 3.0
    assert get_angle_between(1.0, 6.0) == -get_angle_between(6.0, 1.0) == 5.0 - 2 * math.pi
    assert get_angle_between(1.0, 8.0) == -get_angle_between(8.0, 1.0) == 7.0 - 2 * math.pi
    assert get_angle_between(1.0, 12.0) == 11.0 - 4 * math.pi
