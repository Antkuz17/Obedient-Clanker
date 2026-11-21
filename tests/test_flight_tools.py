"""Pure logic of the flight-test tools (no FC needed)."""
from tools.flow_signcheck import advise, corr
from tools.fc_profile import PROFILE


def test_corr_basic():
    a = [float(i % 7) for i in range(100)]
    assert corr(a, a) > 0.99
    assert corr(a, [-x for x in a]) < -0.99
    assert corr(a, [1.0] * 100) == 0.0


def test_advise():
    assert advise(0.9, 0.8, 0.1, 0.0) == []
    assert advise(-0.9, 0.8, 0.1, 0.0) == ["--flip-x"]
    assert advise(0.9, -0.8, 0.0, 0.1) == ["--flip-y"]
    assert advise(0.1, 0.0, 0.9, 0.8) == ["--swap-xy"]
    assert advise(0.0, 0.1, -0.9, 0.8) == ["--swap-xy", "--flip-y"]


def test_profile_matches_verified_board_values():
    assert PROFILE["FS_GCS_ENABLE"][0] == 5 and PROFILE["FLTMODE6"][0] == 9
    assert PROFILE["RNGFND1_TYPE"][0] == 10 and PROFILE["FLOW_TYPE"][0] == 5
    assert PROFILE["RNGFND1_MIN"][0] <= 0.05  # must read valid on the ground (SITL)
