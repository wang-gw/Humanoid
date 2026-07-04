# 시작 지점

## 목표

우리의 목표는 강화학습으로 보행 정책을 만들 수 있는 휴머노이드 하드웨어를 설계하는 것이다.

현재 검증의 목적은 최종 모터를 확정하는 것이 아니라, 현재 설계한 로봇 형상과 기본 actuator 가정이 RL 보행 문제를 풀 수 있는 범위 안에 있는지 확인하는 것이다. 하드웨어 형상 자체가 부적절하면 reward나 학습 시간을 늘려도 안정적인 보행이 나오기 어렵기 때문에, RL 전에 물리 가능성 검증을 진행한다.

## 현재 모델

- 원본 저장소: `https://github.com/wang-gw/Humanoid.git`
- 작업 복제본: `/home/king0519/projects/Humanoid`
- 기본 모델: `URDF_F_description/URDF_F_description/urdf/URDF_F.xacro`
- 메시 파일: `URDF_F_description/URDF_F_description/meshes/`

## 현재 가정

- URDF/xacro의 mass, inertia, joint axis는 CAD export 기반의 초기값으로 취급한다.
- 모든 actuator limit은 아직 최종 모터 사양이 아니라 임시값으로 취급한다.
- 최종 모터 선정 전에 후보 모터급의 torque/velocity range를 넣어 sanity check를 수행한다.
- RL 학습 전에는 최소한 standing, squat, weight shift, one-leg support가 물리적으로 가능한지 확인한다.

## 초기 검증 질문

1. 현재 link 질량과 관성 값이 누락되지 않았는가?
2. 좌우 다리의 mass, joint axis, joint range가 의도대로 대칭인가?
3. 현재 joint 자유도와 range로 서기, 무게 이동, 한 발 지지가 가능한가?
4. 간단한 PD 제어에서 요구 torque가 후보 actuator 범위 안에 있는가?
5. 접촉 모델과 발바닥 형상이 보행 학습에 충분히 안정적인가?
6. RL에서 실패한다면 하드웨어 문제와 reward/controller 문제를 분리할 수 있는 로그가 있는가?

## 첫 실행 가능한 근거

초기 정적 감사:

```bash
python3 scripts/audit_urdf_model.py
```

예상 산출물:

- `docs/hardware_validation/initial_model_audit.json`
- `docs/hardware_validation/initial_model_audit.md`
