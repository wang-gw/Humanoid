# Zero-Control Smoke Probe

## 목적

Actuator가 달린 MJCF가 짧은 MuJoCo rollout에서 수치적으로 실행되는지 확인한다. 이 probe는 보행 가능성 판단이 아니라, standing/squat controller를 만들기 전의 최소 실행성 확인이다.

## 실행 명령

```bash
python3 scripts/probe_mujoco_smoke.py
```

## 요약

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f/URDF_F_mujoco.xml`
- 실행 시간: `1.0` sec
- timestep: `0.002`
- step 수: `500`
- 초기 base z: `0.000000`
- 최종 base z: `-0.027935`
- 최대 contact 수: `11`
- 최대 qvel norm: `29.047916`

## 산출물

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/smoke_probe/zero_control_smoke.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/smoke_probe/zero_control_smoke_summary.json`

## 해석

Zero-control 상태에서 로봇이 균형을 잡는 것은 기대하지 않는다. 다음 단계에서는 neutral standing pose와 PD controller를 정의해 넘어짐이 제어 문제인지 형상 문제인지 분리한다.
