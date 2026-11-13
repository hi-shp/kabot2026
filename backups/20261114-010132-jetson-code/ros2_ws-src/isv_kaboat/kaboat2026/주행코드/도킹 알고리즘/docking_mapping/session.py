"""Operator-started INITIAL_MAPPING lifecycle; autonomous tracking is separate."""
class MappingSession:
    def __init__(self):
        self.phase='INITIAL_MAPPING'
        self.state='IDLE'

    @property
    def active(self):
        return self.phase=='INITIAL_MAPPING' and self.state=='ACTIVE'

    def start(self,rc_ready):
        if not rc_ready:raise ValueError('RC_NOT_READY: mapping start requires fresh RC telemetry')
        if self.phase!='INITIAL_MAPPING' or self.state!='IDLE':
            raise ValueError('Reset for a new map; use resume for a paused map')
        self.state='ACTIVE'

    def resume(self,rc_ready):
        if not rc_ready:raise ValueError('RC_NOT_READY: mapping resume requires fresh RC telemetry')
        if self.phase!='INITIAL_MAPPING' or self.state not in ('PAUSED','PAUSED_RC_LOST'):
            raise ValueError('Only a paused initial mapping session can resume')
        self.state='ACTIVE'

    def pause(self):
        if self.phase!='INITIAL_MAPPING' or self.state=='FINISHED':
            raise ValueError('Initial mapping is already finished')
        if self.state!='IDLE':self.state='PAUSED'

    def lose_rc(self):
        if self.active:self.state='PAUSED_RC_LOST'

    def finish(self):
        if self.phase!='INITIAL_MAPPING' or self.state=='IDLE':
            raise ValueError('No initial mapping session to finish')
        self.state='FINISHED'

    def can_freeze(self):
        if self.phase!='INITIAL_MAPPING' or self.state!='FINISHED':
            raise ValueError('Finish mapping before selecting/saving a reference map')

    def reference_ready(self):
        self.phase='REFERENCE_READY';self.state='FROZEN'

    def reset(self):
        self.phase='INITIAL_MAPPING';self.state='IDLE'
