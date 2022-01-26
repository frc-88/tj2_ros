import sys

sys.path.insert(0, "../../tj2_tools")

import math
import random
from tj2_tools.particle_filter.state import FilterState


def assert_equal(state1, state2):
    assert abs(state1 - state2) < 1E-6, "\n%s != \n%s" % (state1, state2)


def test_relative_to():
    state = FilterState()
    state.x = 10.0
    state.vx = 6.0

    odom = FilterState()
    odom.theta = math.pi / 2

    expected_state = FilterState()
    expected_state.y = 10.0
    expected_state.theta = math.pi / 2
    expected_state.vy = 6.0

    new_state = state.relative_to(odom)
    assert_equal(new_state, expected_state)

    state = FilterState()
    state.x = 10.0
    state.y = 10.0
    state.vx = 6.0
    state.vy = 6.0

    odom = FilterState()
    odom.theta = math.pi / 2

    expected_state = FilterState()
    expected_state.x = -10.0
    expected_state.y = 10.0
    expected_state.vx = -6.0
    expected_state.vy = 6.0
    expected_state.theta = math.pi / 2

    new_state = state.relative_to(odom)
    assert_equal(new_state, expected_state)

    state = FilterState()
    state.x = 10.0
    state.y = 9.0
    state.z = 8.0
    state.vx = 7.0
    state.vy = 6.0
    state.vz = 5.0

    odom = FilterState()
    odom.x = 3.0
    odom.y = 2.0
    odom.z = 1.0
    odom.theta = math.pi / 2

    expected_state = FilterState()
    expected_state.x = -6.0
    expected_state.y = 12.0
    expected_state.z = 9.0
    expected_state.vx = -6.0
    expected_state.vy = 7.0
    expected_state.vz = 5.0
    expected_state.theta = math.pi / 2

    new_state = state.relative_to(odom)
    assert_equal(new_state, expected_state)


def main():
    test_relative_to()

    # random.seed(8888)
    state = FilterState()
    state.x = random.random() * 10.0
    state.y = random.random() * 10.0
    state.z = random.random() * 10.0
    state.theta = random.random() * math.pi * 2
    state.vx = random.random() * 10.0
    state.vy = random.random() * 10.0
    state.vz = random.random() * 10.0
    state.vt = random.random() * 10.0
    # state.x = 10.0
    # state.y = 5.0
    # state.theta = 0.1

    odom = FilterState()
    odom.x = random.random() * 10.0
    odom.y = random.random() * 10.0
    odom.z = random.random() * 10.0
    odom.theta = random.random() * math.pi * 2
    odom.vx = random.random() * 10.0
    odom.vy = random.random() * 10.0
    odom.vz = random.random() * 10.0
    odom.vt = random.random() * 10.0
    # odom.x = -1.0
    # odom.y = 2.0
    # odom.theta = math.pi / 2 + 0.2

    state_odom = state.relative_to(odom)
    state_back = state_odom.relative_to_reverse(odom)

    print("original:", state)
    print("odom:", odom)
    print("state in odom:", state_odom)
    print("back to original:", state_back)
    print("diff:", state - state_back)
    assert abs(state - state_back) < 1E-6


if __name__ == '__main__':
    main()
