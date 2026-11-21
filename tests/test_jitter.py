from tools import jitter as J


def test_summary_and_histogram():
    late = [0.05] * 90 + [1.5] * 9 + [30.0]
    s = J.summarize(late)
    assert s["n"] == 100 and s["p50_ms"] == 0.05 and s["max_ms"] == 30.0 and s["over_2ms"] == 1
    h = J.histogram(late)
    assert "<0.1 ms" in h and h.splitlines()[0].endswith(" 90")


def test_measure_runs_at_rate():
    late = J.measure(0.5, hz=50)
    assert 20 <= len(late) <= 26 and min(late) >= 0


def test_run_case_reports_prio_applied():
    r = J.run_case(0.2, 50, hog=False, prio=0)
    assert r["prio_applied"] is False and r["n"] > 0 and len(r["late_ms"]) == r["n"]
