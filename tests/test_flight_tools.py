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


def test_flight_report_on_synthetic_flight():
    from tools.flight_report import report
    recs = []
    for i in range(20):
        t = 100 + i * 0.5
        armed = 2 <= i <= 17
        recs.append({"mavpackettype": "HEARTBEAT", "type": 2, "base_mode": 129 if armed else 1,
                     "custom_mode": 4 if armed else 2, "_t": t})
        recs.append({"mavpackettype": "LOCAL_POSITION_NED", "x": 0.01 * i, "y": 0.0,
                     "z": -0.5, "_t": t})
        recs.append({"mavpackettype": "OPTICAL_FLOW", "quality": 200, "_t": t})
        recs.append({"mavpackettype": "SYS_STATUS", "voltage_battery": 12000 - 20 * i, "_t": t})
    out = report(recs)
    assert "armed for 8 s" in out and "GUIDED" in out
    assert "optical flow quality mean 200" in out and "battery" in out
    assert report(recs[:1]) == "no armed period in this file"


def test_gps_profile_keeps_rangefinder_height_and_failsafes():
    from tools.fc_profile import profile_for
    g = profile_for(gps=True)
    assert g["GPS1_TYPE"][0] == 1 and g["EK3_SRC1_POSXY"][0] == 3 and g["EK3_SRC1_VELXY"][0] == 3
    assert g["EK3_SRC1_POSZ"][0] == 2          # height stays on the rangefinder, not the baro
    assert g["EK3_SRC1_VELZ"][0] == 0          # field 2026-11-22: GPS vz drifted on the ground
    assert g["GPS_AUTO_CONFIG"][0] == 0        # field 2026-11-22: "GPS 1 still configuring"
    for k in ("FS_GCS_ENABLE", "FS_EKF_ACTION", "FLTMODE6"):
        assert g[k] == PROFILE[k]
