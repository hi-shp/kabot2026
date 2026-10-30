import argparse
import csv
import json
from pathlib import Path

from .common import dump_json, load_config
from .identification import analyze


def main(argv=None):
    parser = argparse.ArgumentParser(description='Offline analysis; NEVER commands the boat')
    parser.add_argument('session')
    parser.add_argument('--config', help='Override recorded config; explicitly confirm mounting/water conditions here')
    args = parser.parse_args(argv)
    session = Path(args.session).expanduser()
    metadata = json.loads((session / 'metadata.json').read_text())
    cfg = load_config(args.config) if args.config else metadata['config']
    rows, bad = [], 0
    with (session / 'raw.jsonl').open() as f:
        for line in f:
            try:
                row = json.loads(line)
                if not isinstance(row['t'], (int, float)) or not isinstance(row['msg'], dict):
                    raise ValueError('Invalid record')
                rows.append(row)
            except (ValueError, KeyError, TypeError):
                bad += 1
    report, samples = analyze(rows, cfg)
    report['quality']['malformed_raw_lines'] = bad
    report['config_used'] = cfg
    report['input_session'] = str(session)
    dump_json(session / 'report.json', report)
    with (session / 'kinematics.csv').open('w', newline='') as f:
        if samples:
            writer = csv.DictWriter(f, fieldnames=list(samples[0]))
            writer.writeheader()
            writer.writerows(samples)
    summary = ['# 수상 시험 분석', '', f"GPS 유효 표본: {len(samples)}", '',
               f"제원: 질량 {cfg['mass_kg']} kg, 길이 {cfg['length_m']} m, 폭 {cfg['beam_m']} m, 추진기 간격 {cfg['thruster_spacing_m']} m.", '',
               '## 분석 상태', '', '```json', json.dumps(report, ensure_ascii=False, indent=2, default=float), '```', '',
               '표본의 속도는 기본적으로 대지 속도입니다. PWM을 N으로 취급하지 않습니다.',
               '자세 장착 방향 확인 전 선체 좌표 속도와 동역학 계수는 비활성화됩니다.']
    (session / 'report_KO.md').write_text('\n'.join(summary), encoding='utf-8')
    # This export is only a candidate for simulation; real actuation is never enabled.
    models = report['models']
    candidate = {'schema_version': 1, 'source_session': str(session), 'boat': report['boat'],
                 'model_type': 'reduced_surge_yaw', 'deployment_ready': False,
                 'validation_required': ['independent held-out runs', 'actuator delay/deadband',
                                         'sway/coupling validation', 'LiDAR extrinsic and time alignment',
                                         'stop-distance and obstacle-clearance validation'],
                 'config':cfg,
                 'identified_models': {k: v for k, v in models.items() if v['status'] == 'estimated'}}
    dump_json(session / 'mpc_model_candidate.json', candidate)
    print('REPORT=' + str(session / 'report_KO.md'))
    print('MODEL_CANDIDATE=' + str(session / 'mpc_model_candidate.json'))
    for warning in report['warnings']:
        print('WARNING:', warning)
