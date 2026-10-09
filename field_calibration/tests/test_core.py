import json
import math
import pytest
from field_calibration.core import StateGate, ErrorMatcher, wrap
from field_calibration.model import Commands, predict, load_model
from field_calibration.storage import Experiment

C = dict(scan_frame='base_scan', imu_frame='imu_link', odom_frame='odom', base_frame='base_footprint',
         max_age=0.5, future_tolerance=0.05, max_sensor_skew=0.12, min_scan_fraction=0.15,
         min_scan_sectors=4, max_position_variance=1., max_heading_variance=.25,
         allow_unknown_covariance=False, min_dt=.01, max_dt=.3, max_speed=3.,
         max_yaw_rate=1.5, max_yaw_disagreement=.6)


def gate():
    g = StateGate(C)
    g.set_scan(dict(stamp=10.1, received=10.1, frame='base_scan', fraction=.9, sectors=8))
    g.set_imu(dict(stamp=10.1, received=10.1, frame='imu_link', r=.1, heading=.01))
    for t, x, y, h in [(10., 0., 0., 0.), (10.1, .05, .01, .01)]:
        g.set_odom(dict(stamp=t, received=t, frame='odom', child='base_footprint',
                        x=x, y=y, heading=h, covariance=[.01]*3))
    return g


def test_body_sway_is_observed_and_timestamp_preserved():
    s = gate().evaluate(10.1)
    assert s['valid'] and s['stamp'] == 10.1 and .09 < s['v'] < .11
    assert s['r'] == .1


@pytest.mark.parametrize('fault,reason', [
    ('scan', 'lidar_missing'), ('imu', 'imu_missing'), ('odom', 'rf2o_missing'),
    ('age', 'rf2o_timestamp_or_age'), ('skew', 'imu_odom_skew'), ('covariance', 'covariance_unknown'),
    ('jump', 'odom_jump'), ('frame', 'odom_frame_mismatch'), ('features', 'scan_frame_or_features'),
    ('nonfinite', 'odom_nonfinite'), ('nan_cov', 'covariance_invalid'), ('backward', 'odom_interval')])
def test_invalid_never_invents_position(fault, reason):
    g = gate()
    now = 10.1
    if fault == 'scan': g.scan = None
    elif fault == 'imu': g.imus.clear()
    elif fault == 'odom': g.odom = None
    elif fault == 'age': now = 11.
    elif fault == 'skew': g.imus[0]['stamp'] = 9.9
    elif fault == 'covariance': g.odom['covariance'] = [0.]*3
    elif fault == 'jump': g.odom['x'] = 10.
    elif fault == 'frame': g.odom['frame'] = 'map'
    elif fault == 'features': g.scan['sectors'] = 1
    elif fault == 'nonfinite': g.odom['x'] = float('nan')
    elif fault == 'nan_cov': g.odom['covariance'][0] = float('nan')
    elif fault == 'backward': g.odom['stamp'] = 9.9
    s = g.evaluate(now)
    assert not s['valid'] and reason in s['reasons']
    assert all(s[k] is None for k in ('x', 'y', 'heading', 'u', 'v', 'r'))


def test_unknown_covariance_opt_in_remains_limited():
    g = gate()
    g.c = dict(C, allow_unknown_covariance=True)
    g.odom['covariance'] = [0]*3
    s = g.evaluate(10.1)
    assert s['valid'] and s['quality'] == 'LIMITED_UNKNOWN_COVARIANCE'


def state(t, x, h=0, epoch=1):
    return dict(stamp=t, valid=True, x=x, y=0, heading=h, u=1., v=0., r=0., epoch=epoch, frame='odom')


def test_baseline_no_coefficients_and_exact_horizons():
    p = predict(state(10., 0.), Commands())
    assert p['model_status'] == 'UNCALIBRATED'
    assert [v['horizon'] for v in p['points']] == [.5, 1., 2.]
    assert [v['x'] for v in p['points']] == pytest.approx([.5, 1., 2.])
    assert predict(dict(valid=False), Commands()) is None


def test_future_truth_interpolation_wrapping_and_gap():
    matcher = ErrorMatcher()
    matcher.observe(state(10., 0.))
    p = predict(state(10., 0.), Commands())
    matcher.add(p)
    for t in [10.1, 10.2, 10.3, 10.4]:
        assert not matcher.observe(state(t, t-10))
    e = matcher.observe(state(10.6, .6))
    assert len(e) == 1 and e[0]['position_error'] < 1e-8
    matcher.observe(dict(valid=False))
    assert not matcher.pending
    assert not matcher.observe(state(11, 1))
    assert abs(wrap(math.pi+.1)+math.pi-.1) < 1e-8


def test_epoch_change_and_long_gap_are_not_scored():
    for target in (state(10.6, .6, epoch=2), state(11., 1.)):
        matcher = ErrorMatcher()
        matcher.observe(state(10., 0.))
        matcher.add(predict(state(10., 0.), Commands()))
        assert not matcher.observe(target)


def test_model_uncalibrated_file_and_flush(tmp_path):
    path = tmp_path/'model.json'
    path.write_text(json.dumps(dict(schema=1, status='UNCALIBRATED')))
    assert load_model(path) is None
    e = Experiment(tmp_path, 'manual_log', 'indoor', {})
    e.row('states', dict(stamp=1., valid=False, status='INVALID'))
    e.sync()
    assert 'INVALID' in (e.path/'states.csv').read_text()
    e.close()
    e.close()
    assert json.loads((e.path/'metadata.json').read_text())['closed']
