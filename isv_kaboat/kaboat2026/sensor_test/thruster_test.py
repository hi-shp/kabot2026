#!/usr/bin/env python3
"""Inspect thrust telemetry by default; --execute explicitly selects continuous output."""
import math
from _common import parser,run,actuator_details

# 이 두 값만 수정: 좌현/우현 논리 PWM (정수, 1100~1900, 중립 1500)
LEFT_PWM = 1600   # 좌현: 1500 초과 전진, 미만 후진
RIGHT_PWM = 1600  # 우현: 1500 초과 전진, 미만 후진

# 시작 시 ESC에 실제 1500us를 3초간 유지한 후 위 추력으로 전환합니다.
# 현재 UNO: 실제 중립 1480us, AUTO 좌우 반전 -> 논리 1481이 실제 1500us로 변환됩니다.
# 이 초기화 값은 현재 펌웨어 변환 기준이며, 펌웨어 중립/방향 변경 시 함께 확인해야 합니다.
INITIALIZE_PWM = (1481, 1481)
INITIALIZE_SECONDS = 3.0

def resolve_pwm(percent=None,side=None):
    if percent is None:
        if side is not None: raise ValueError('--side is used only with --percent')
        pair=(LEFT_PWM,RIGHT_PWM)
    else:
        if not math.isfinite(percent) or not -100<=percent<=100:
            raise ValueError('--percent must be finite and in -100..100')
        side=side or 'left'
        pulse=1500+round(400*percent/100)
        pair=(pulse if side in ('left','both') else 1500,pulse if side in ('right','both') else 1500)
    from thruster_output import validate_pair
    return validate_pair(*pair)


def inspect_thruster(latest,topic):
    result=actuator_details(latest,topic)
    state=result.get('telemetry',{})
    observed={side:state.get('observed_'+side+'_us') for side in ('left','right')}
    reported=all(type(v) is int and 900<=v<=2100 for v in observed.values())
    result.update(command_pwm_us={side:state.get(side) for side in ('left','right')},mcu_reported_output_pwm_us=observed,output_telemetry_valid=reported)
    result['rotation_verified']=False
    result['output_state']=('NO_VALID_PWM' if not reported else
        'NEUTRAL' if state.get('left')==1500 and state.get('right')==1500 else 'NON_NEUTRAL_PWM')
    result['interpretation']='PASS checks telemetry/PWM observation only; shaft rotation and thrust are not measured'
    result['valid']=bool(result['valid'] and reported)
    if not reported: result['issues'].append('No valid MCU-reported output pulse widths')
    return result

def main(argv=None):
    p=parser(__doc__);p.set_defaults(duration=None)
    p.add_argument('--topic',default='/actuators/status')
    p.add_argument('--execute',action='store_true',help='Continuously output LEFT_PWM/RIGHT_PWM until Ctrl+C')
    p.add_argument('--side',choices=('left','right','both'),default=None,help='Only for the optional --percent override')
    p.add_argument('--percent',type=float,help='Optional legacy override, -100..100; otherwise use file variables')
    args=p.parse_args(argv)
    if args.execute:
        if args.duration is not None or args.watch or args.output:
            p.error('--duration/--watch/--output apply to passive inspection only; active output ends with Ctrl+C')
        if args.topic!='/actuators/status': p.error('Active test uses the existing actuator service configuration')
        try: pwm=resolve_pwm(args.percent,args.side)
        except ValueError as exc: p.error(str(exc))
        from thruster_output import run_pair
        print('Continuous PWM: LEFT=%d RIGHT=%d. Ctrl+C requests neutral and disarm.' % pwm,flush=True)
        run_pair(*pwm, initialize_pwm=INITIALIZE_PWM, initialize_seconds=INITIALIZE_SECONDS)
        return 0
    if args.percent is not None or args.side is not None: p.error('--percent/--side require --execute')
    args.duration=10. if args.duration is None else args.duration
    return run('thruster',args,{args.topic:'std_msgs/msg/String'},lambda latest:inspect_thruster(latest,args.topic))

if __name__=='__main__': raise SystemExit(main())
