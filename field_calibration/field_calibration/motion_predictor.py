from std_msgs.msg import String, Float64
from .common import FieldNode, spin, qos_profile_sensor_data
from .core import ErrorMatcher
from .model import Commands, load_model, predict


class MotionPredictor(FieldNode):
    def __init__(self):
        super().__init__('motion_predictor')
        self.commands = Commands()
        try:
            self.model = load_model(self.param('model_path', ''))
        except (ValueError, OSError, KeyError, TypeError) as e:
            self.model = None
            self.get_logger().warning(f'UNCALIBRATED: model could not be loaded: {e}')
        self.matcher = ErrorMatcher(self.param('error_max_gap', 0.3))
        self.horizons = self.param('horizons', [0.5, 1.0, 2.0])
        if any(h <= 0 or h > 5 for h in self.horizons):
            raise ValueError('horizons must be in (0,5] seconds')
        self.interval = 1/self.param('prediction_hz', 4.0)
        self.max_age = self.param('command_max_age', 2.0)
        self.last_prediction = None
        self.last_state = None
        self.pub = self.json_publisher('/field/prediction')
        self.err = self.json_publisher('/field/prediction_error')
        self.subscribe_json('/field/state', self.state_cb)
        for kind, topic in [('throttle', self.param('throttle_topic', '/actuator/thruster/percentage')),
                            ('steering', self.param('steering_topic', '/actuator/key/degree'))]:
            self.create_subscription(Float64, topic,
                                     lambda m, k=kind: self.commands.add(k, self.now(), m.data),
                                     qos_profile_sensor_data)

    def state_cb(self, state):
        if not state['valid']:
            self.matcher.observe(state)
            self.last_state = None
            self.last_prediction = None
            return
        if self.last_state is not None and state['stamp'] <= self.last_state:
            return
        self.last_state = state['stamp']
        for error in self.matcher.observe(state):
            self.publish_json(self.err, error)
        if self.last_prediction is None or state['stamp']-self.last_prediction >= self.interval:
            p = predict(state, self.commands, self.model, self.horizons, command_max_age=self.max_age)
            if p:
                self.matcher.add(p)
                self.publish_json(self.pub, p)
                self.last_prediction = state['stamp']


def main(args=None):
    spin(MotionPredictor, args)
