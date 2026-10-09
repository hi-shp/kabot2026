import math
import pytest
from field_calibration.model import Commands, predict, rates
from field_calibration.motion_identifier import identify


def dataset(identifier, phase, throttle_delay=0., steering_delay=0.):
    commands = Commands()
    states = []
    u = r = h = x = y = 0.
    model = dict(surge=[-.7, .045, 0.], yaw=[-1.2, 1.8, 1.5, 0.],
                 servo_neutral=90., throttle_neutral=0.)
    for i in range(1800):
        t = 10+i*.05
        throttle = 10+8*math.sin(i*.05*.45+phase)+2*math.sin(i*.05*1.8+phase)
        steering = 90+15*math.sin(i*.05*.7+phase)
        commands.add('throttle', t, throttle)
        commands.add('steering', t, steering)
        states.append(dict(stamp=t, x=x, y=y, heading=h, u=u, v=0., r=r,
                           valid=True, epoch=1, frame='odom', experiment_id=identifier))
        thr = commands.at('throttle', t-throttle_delay)
        steer = commands.at('steering', t-steering_delay)
        du, dr = rates(model, u, r, thr[1] if thr else 0., steer[1] if steer else 90.)
        u += du*.05
        r += dr*.05
        h += r*.05
        x += u*math.cos(h)*.05
        y += u*math.sin(h)*.05
    return dict(path=identifier, sha256=identifier, ids=[identifier], states=states, commands=commands)


def test_empirical_fit_on_independent_experiments():
    m = identify([dataset('training', 0.)], [dataset('holdout', .4)])
    assert m['status'] == 'CALIBRATED'
    assert m['surge'] == pytest.approx([-.7, .045, 0.], abs=1e-8)
    assert m['yaw'] == pytest.approx([-1.2, 1.8, 1.5, 0.], abs=1e-8)
    assert m['throttle_delay'] == 0. and m['steering_delay'] == 0.
    assert m['validation']['model']['0.5']['count'] > 100
    for h in ('0.5', '1.0', '2.0'):
        assert m['validation']['model'][h]['count'] == m['validation']['baseline'][h]['count']


def test_overlap_and_insufficient_excitation_rejected():
    d = dataset('same', 0.)
    with pytest.raises(ValueError, match='overlap'):
        identify([d], [d])
    d['states'] = d['states'][:20]
    with pytest.raises(ValueError, match='insufficient'):
        identify([d], [dataset('different', .4)])


def test_stale_commands_force_labelled_baseline():
    d = dataset('train', 0)
    m = identify([d], [dataset('val', .4)])
    p = predict(d['states'][-1], Commands(), m)
    assert p['model_status'] == 'UNCALIBRATED'


def test_nonzero_independent_response_delays():
    m = identify([dataset('delayed_train', 0., .2, .3)], [dataset('delayed_val', .4, .2, .3)])
    assert m['throttle_delay'] == pytest.approx(.2)
    assert m['steering_delay'] == pytest.approx(.3)
    assert m['surge'] == pytest.approx([-.7, .045, 0.], abs=1e-8)
