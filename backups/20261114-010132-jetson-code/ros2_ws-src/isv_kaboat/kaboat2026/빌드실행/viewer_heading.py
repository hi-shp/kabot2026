"""Display-only heading reader matching the navigation CommonGroup convention."""
import math

class HeadingPreview:
    def __init__(self,frame='vectornav',offset_deg=0.,timeout_s=.5):
        self.frame=frame;self.offset_deg=offset_deg;self.timeout_s=timeout_s
        self.yaw=None;self.stamp=-math.inf
    def update(self,message,now_s):
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        angle=message.yawpitchroll.x
        if (message.header.frame_id!=self.frame or not math.isfinite(angle)
                or not -.1<=now_s-stamp<=self.timeout_s or stamp<=self.stamp):return
        self.yaw=math.pi/2-math.radians(angle+self.offset_deg)
        self.stamp=stamp
    def snapshot(self,now_s):
        age=now_s-self.stamp
        return dict(yaw=self.yaw if -.1<=age<=self.timeout_s else None,
                    heading_age=max(0.,age) if math.isfinite(age) else None)
