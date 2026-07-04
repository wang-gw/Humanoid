# MJCF Actuator 기준 모델

## 목적

MuJoCo가 URDF를 읽을 수는 있지만, URDF transmission은 MuJoCo actuator로 변환되지 않는다. RL policy가 로봇을 제어하려면 MuJoCo actuator가 필요하므로, 준비된 URDF를 MJCF로 저장한 뒤 각 hinge joint에 임시 torque motor를 추가한다.

## 실행 명령

```bash
python3 scripts/build_mujoco_mjcf.py
python3 scripts/check_mujoco_model.py --model envs/robots/urdf_f/URDF_F_mujoco.xml
```

## 산출물

- `envs/robots/urdf_f/URDF_F_mujoco.xml`
- `docs/hardware_validation/mujoco_load_report.json`
- `docs/hardware_validation/mujoco_load_report.md`

## 현재 actuator 가정

- actuator 타입: MuJoCo `motor`
- 명령 의미: 직접 joint torque
- 임시 torque limit: `+/-100 Nm`
- Gear: `1`

이 값은 최종 모터 선정값이 아니다. standing/squat/weight-shift/one-leg-support probe를 시작하기 위한 placeholder다. 이후 후보 모터 사양을 정하면 joint별 `ctrlrange`, `forcerange`, gear, velocity, damping, armature를 업데이트해야 한다.

## 준비 기준

- `nq = 17`, `nv = 16`: floating base + 10 hinge joints
- `nu = 10`: 10개 hinge joint에 actuator 존재
- floor geom 존재
