"""Laptop-only numeric DDS viewer. No scan/image subscriptions."""
import argparse
import math
import time
import textwrap
from collections import deque
import rclpy
from .common import FieldNode


class TrajectoryMonitor(FieldNode):
    def __init__(self):
        super().__init__('trajectory_monitor')
        self.states = deque(maxlen=2400)
        self.trajectory = deque(maxlen=24000)
        self.prediction = None
        self.errors = deque(maxlen=2400)
        self.recording = None
        self.profile = None
        self.commands = deque(maxlen=2400)
        self.current_commands = dict(throttle=None, steering=None)
        self.last_stamp = None
        self.last_receive = None
        self.subscribe_json('/field/state', self.state_cb)
        self.subscribe_json('/field/prediction', self.prediction_cb)
        self.subscribe_json('/field/prediction_error', self.errors.append)
        self.subscribe_json('/field/recording_status', lambda d: setattr(self, 'recording', d))
        self.subscribe_json('/field/test_profile', lambda d: setattr(self, 'profile', d))
        self.subscribe_json('/field/command', self.command_cb)

    def state_cb(self, data):
        self.last_receive = time.monotonic()
        if data['stamp'] == self.last_stamp and data['valid']:
            return
        self.last_stamp = data['stamp']
        self.states.append(data)
        self.trajectory.append((data['x'], data['y']) if data['valid'] else (float('nan'), float('nan')))
        if not data['valid']:
            self.prediction = None

    def prediction_cb(self, data):
        self.prediction = data

    def command_cb(self, data):
        self.current_commands[data['kind']] = data['value']
        self.commands.append((data['stamp'], dict(self.current_commands)))


def draw(node, figure, axes, twins):
    import numpy as np
    for ax in list(axes)+list(twins):
        ax.clear()
        ax.tick_params(labelsize=10)
    for ax in twins:
        ax.yaxis.set_label_position('right')
        ax.yaxis.tick_right()
    xy, steer, surge, status = axes
    s = node.states[-1] if node.states else None
    stale = node.last_receive is None or time.monotonic()-node.last_receive > 1.5
    trajectory = list(node.trajectory)
    if trajectory:
        xy.plot([p[0] for p in trajectory], [p[1] for p in trajectory], '-', label='Observed')
    p = node.prediction if not stale else None
    if p:
        xy.plot([v['x'] for v in p['path']], [v['y'] for v in p['path']], '--', label=p['model_status'])
    if s and s['valid'] and not stale:
        xy.plot(s['x'], s['y'], 'o')
        xy.quiver(s['x'], s['y'], math.cos(s['heading']), math.sin(s['heading']),
                  angles='xy', scale_units='xy', scale=2)
        evaluated = [e for e in node.errors if e['epoch'] == s['epoch'] and e['frame'] == s['frame']
                     and 0 <= s['stamp']-e['target_stamp'] <= 3]
        if evaluated:
            e = evaluated[-1]
            xy.plot([e['x'], e['actual']['x']], [e['y'], e['actual']['y']], 'r.-',
                    label='Evaluated forecast error')
    xy.set(xlabel='odom x [m]', ylabel='odom y [m]', title='Observed and predicted trajectory')
    xy.set_aspect('equal', adjustable='datalim')
    if xy.lines:
        xy.legend(fontsize=10)
    if s:
        t0 = s['received']
        data = [v for v in node.states if 0 <= t0-v['received'] <= 60]
        steer.plot([v.get('imu_stamp', v['stamp'])-t0 if v.get('imu_stamp') is not None else np.nan for v in data],
                   [v['imu_r'] if v['imu_r'] is not None and v['ages']['imu'] is not None
                    and -.05 <= v['ages']['imu'] <= .5 else np.nan for v in data], label='IMU', color='#006699')
        surge.plot([v['stamp']-t0 for v in data], [v['u'] if v['valid'] else np.nan for v in data],
                   label='Observed surge', color='#006699')
        evaluated = [e for e in node.errors if e['horizon'] == .5 and 0 <= t0-e['target_stamp'] <= 60]
        for ax, kind in ((steer, 'r'), (surge, 'u')):
            forecast_t, forecast_y, previous = [], [], None
            for e in evaluated:
                if previous and (e['epoch'] != previous['epoch'] or e['frame'] != previous['frame']
                                 or e['target_stamp']-previous['target_stamp'] > .5):
                    forecast_t.append(np.nan)
                    forecast_y.append(np.nan)
                forecast_t.append(e['target_stamp']-t0)
                forecast_y.append(e[kind])
                previous = e
            ax.plot(forecast_t, forecast_y,
                    '--', color='#cc6600', label='Evaluated 0.5s forecast')
        cmds = [v for v in node.commands if 0 <= t0-v[0] <= 60]
        for ax, kind in zip(twins, ('steering', 'throttle')):
            ax.step([v[0]-t0 for v in cmds],
                    [v[1][kind] if v[1][kind] is not None else np.nan for v in cmds],
                    where='post', color='#8b4513', label=kind+' command')
            ax.set_ylabel('Servo command [deg]' if kind == 'steering' else 'Thruster command [%]', fontsize=11)
        if p:
            points = p['points']
            label = 'Model' if p['model_status'] == 'CALIBRATED' else 'Uncalibrated baseline'
            steer.plot([v['target_stamp']-t0 for v in points], [v['r'] for v in points], '--o', label=label)
            surge.plot([v['target_stamp']-t0 for v in points], [v['u'] for v in points], '--o', label=label)
    for ax, title, ylabel in [(steer, 'Steering response', 'Yaw rate [rad/s]'),
                              (surge, 'Propulsion response', 'Surge [m/s]')]:
        span = min(60, max(10, s['received']-node.states[0]['received'])) if s else 10
        ax.set(xlabel='Time relative to current sample [s]', ylabel=ylabel, title=title, xlim=(-span, 2.2))
        twin = twins[0] if ax is steer else twins[1]
        handles, labels = ax.get_legend_handles_labels()
        handles2, labels2 = twin.get_legend_handles_labels()
        if handles or handles2:
            ax.legend(handles+handles2, labels+labels2, loc='upper left', fontsize=10)
        ax.grid(alpha=0.3)
    status.axis('off')
    lines = ['PASSIVE / no actuator publishers', 'DDS link: '+('STALE / DISCONNECTED' if stale else 'receiving')]
    if s:
        lines += ['State: '+('INVALID (link stale)' if stale else s['status']), 'Quality: '+s['quality'],
                  'Reasons: '+(', '.join(s['reasons']) or 'none')]
        for name, age in s['ages'].items():
            lines.append(f'{name} age: {age:.3f} s' if age is not None else name+': missing')
    if node.recording:
        lines += ['Recording: '+('last known ' if stale else '')+('ACTIVE' if node.recording['recording'] else 'ERROR'),
                  'Experiment: '+node.recording['experiment_id']]
    else:
        lines += ['Recording: no status received']
    if node.profile:
        lines.append('Test: '+node.profile['mode'])
    lines.append('Model: '+('UNCALIBRATED baseline' if p and p['model_status'] == 'UNCALIBRATED'
                            else p['model_version'] if p else 'UNCALIBRATED / unavailable'))
    for horizon in (0.5, 1.0, 2.0):
        values = [e for e in node.errors if e['horizon'] == horizon]
        if values:
            e = values[-1]
            lines.append(f"{horizon:g}s error: {e['position_error']:.3f} m, {math.degrees(e['heading_error']):+.2f} deg")
    status.text(0, 1, '\n'.join(textwrap.fill(line, width=66) for line in lines), va='top', fontsize=11)
    for ax in axes:
        ax.xaxis.label.set_size(11)
        ax.yaxis.label.set_size(11)
        ax.title.set_size(13)
    figure.canvas.draw_idle()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', help='Save a figure and exit (headless verification).')
    parser.add_argument('--seconds', type=float, default=5.0)
    opts, ros_args = parser.parse_known_args(args)
    import matplotlib
    if opts.snapshot:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rclpy.init(args=ros_args)
    node = TrajectoryMonitor()
    figure, grid = plt.subplots(2, 2, figsize=(15, 9), layout='constrained')
    axes = [grid[0, 0], grid[0, 1], grid[1, 0], grid[1, 1]]
    twins = [axes[1].twinx(), axes[2].twinx()]
    if not opts.snapshot:
        plt.show(block=False)
    started = time.monotonic()
    try:
        while rclpy.ok() and (opts.snapshot or plt.fignum_exists(figure.number)):
            end = time.monotonic()+0.25
            while time.monotonic() < end and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
            draw(node, figure, axes, twins)
            if opts.snapshot and time.monotonic()-started >= opts.seconds:
                figure.savefig(opts.snapshot, dpi=120)
                break
            if not opts.snapshot:
                plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close(figure)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
