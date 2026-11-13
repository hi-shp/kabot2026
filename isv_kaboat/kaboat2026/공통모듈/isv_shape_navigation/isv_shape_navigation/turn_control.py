"""Measured yaw-rate feedback for waypoint differential thrust (CCW positive)."""
import math


class YawTurnController:
    def __init__(self):
        self.last_time=None;self.last_turn=0.;self.status={'active':False}

    def reset(self):
        self.last_time=None;self.last_turn=0.;self.status={'active':False}

    def adjust(self, pwm, error_deg, rate, now, *, heading_age=0., scale=1., max_difference=200):
        if not all(math.isfinite(v) for v in (error_deg,rate,now,heading_age,scale)):
            raise ValueError('Invalid measured turn feedback')
        dt=.1 if self.last_time is None else max(.01,min(.2,now-self.last_time))
        # Brake before the desired heading, including measured heading transport.
        lead=.35+max(0.,min(.2,heading_age))
        error=math.radians(error_deg)
        predicted=error-rate*lead
        desired_rate=max(-.45,min(.45,1.3*predicted))
        if abs(error_deg)<1.5 and abs(rate)<.025:desired_rate=0.
        authority=min(100.,max_difference/2.)*scale
        desired_turn=max(-authority,min(authority,180.*(desired_rate-rate)))
        # Keep starting a turn gentle; release or counter-brake it promptly.
        braking=desired_turn*rate<0 or desired_turn*self.last_turn<0 or abs(desired_turn)<abs(self.last_turn)
        slew=600. if braking else 200.
        turn=self.last_turn+max(-slew*dt,min(slew*dt,desired_turn-self.last_turn))
        mean=sum(pwm)/2.
        room=max(0.,min(mean-1100.,1900.-mean));turn=max(-room,min(room,turn))
        self.last_time=now;self.last_turn=turn
        self.status=dict(active=True,phase='yaw_brake' if braking else 'yaw_track',
            measured_rate_deg_s=math.degrees(rate),target_rate_deg_s=math.degrees(desired_rate),
            predicted_error_deg=math.degrees(predicted),turn_pwm=turn,lead_s=lead)
        return round(mean-turn),round(mean+turn)
