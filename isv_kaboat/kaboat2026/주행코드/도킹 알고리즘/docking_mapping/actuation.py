"""Opt-in adapter for the existing paired output authority. Disabled by default."""
import time


class DockingOutput:
    def __init__(self,node,config):
        self.node=node;self.config=config;self.driver=None;self.owner=None
        self.clients=[];self.fault='';self.lease=None;self.readiness=None
        self.enabled=(config['enable_output'] is True and config['dry_run'] is False
                      and config['hardware_output_authorized'] is True)
        if not self.enabled:return  # No arm/PWM/stop clients or output lock in dry-run.
        from isv_shape_navigation.output import PairedOutput,OutputReadiness
        from isv_shape_navigation.output_runtime import CommandLease
        from std_srvs.srv import SetBool,Trigger
        from mechaship_interfaces.srv import ThrottlePulseWidth
        self.arm_type=SetBool;self.pwm_type=ThrottlePulseWidth;self.stop_type=Trigger
        self.clients=[node.create_client(SetBool,'/actuators/arm'),
            node.create_client(Trigger,'/actuators/stop'),
            node.create_client(ThrottlePulseWidth,'/actuators/throttle_left/set_pulse_width'),
            node.create_client(ThrottlePulseWidth,'/actuators/throttle_right/set_pulse_width')]
        self.driver=PairedOutput(self.arm,self.pair,self.stop_service,self.ready,
                                 stop_ready=self.clients[1].service_is_ready)
        self.lease=CommandLease(.2);self.readiness=OutputReadiness()

    def ready(self):return all(c.service_is_ready() for c in self.clients)

    def arm(self,value):return self.clients[0].call_async(self.arm_type.Request(data=value))

    def pair(self,left,right):
        return [self.clients[2].call_async(self.pwm_type.Request(pulse_width=int(left))),
                self.clients[3].call_async(self.pwm_type.Request(pulse_width=int(right)))]

    def stop_service(self):return self.clients[1].call_async(self.stop_type.Request())

    def start(self,telemetry,age):
        if not self.enabled:return
        from isv_shape_navigation.output_runtime import acquire_output_lock,conflicting_output_nodes
        now=time.monotonic();reason=self.readiness.check(now,telemetry,age,self.ready())
        if reason:raise ValueError(reason)
        if conflicting_output_nodes(self.node.get_node_names(),self.node.get_name()):
            raise ValueError('Another navigation output node is running')
        if self.owner is not None:raise ValueError('Previous output ownership has not been released')
        self.owner=acquire_output_lock()
        if not self.driver.start():
            self.owner.close();self.owner=None;raise ValueError('Output driver is still stopping')
        self.lease.publish(now,(1500,1500),True)
        self.fault=''

    def request_stop(self,reason):
        if self.driver is not None:
            self.lease.publish(time.monotonic(),(1500,1500),False)
            self.driver.request_stop(reason)

    def publish(self,pwm,allowed,reverse=False):
        if not self.enabled:return
        allowed=bool(allowed and (not reverse or self.config['reverse_output_authorized']))
        self.lease.publish(time.monotonic(),pwm,allowed)

    def tick(self,telemetry,age):
        if not self.enabled:return
        now=time.monotonic();self.readiness.observe(now,telemetry,age)
        if self.owner is None:return
        command,reason=self.lease.read(now)
        if reason:self.driver.request_stop(reason)
        self.driver.tick(now,command,telemetry,age)
        if self.driver.error and self.driver.active is False:self.fault=self.driver.error
        if not self.driver.active and not self.driver.pending and (not self.driver.owned or self.driver.stop_ack):
            self.owner.close();self.owner=None

    def snapshot(self):
        return dict(enabled=self.enabled,dry_run=not self.enabled,owned=self.owner is not None,
                    armed=False if self.driver is None else self.driver.armed,fault=self.fault,
                    stop_confirmed=None if self.driver is None else self.driver.stop_ack)
