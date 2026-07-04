# 사람이 읽을 수 있는 조인트 이름 적용

## 목적

CAD export 이름인 `Revolute XX`를 그대로 사용하면 torque plot, action vector, policy output을 해석하기 어렵다. 이 단계에서는 임시 joint mapping config를 적용해 검증용 MJCF의 joint/actuator 이름을 사람이 읽을 수 있는 이름으로 바꾼다.

이 이름은 아직 최종 CAD 확인 전의 임시 이름이다.

## 실행 명령

```bash
python3 scripts/apply_joint_mapping.py
```

## 입력과 산출물

- 입력 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f/URDF_F_contact.xml`
- mapping config: `/home/king0519/projects/Humanoid/configs/joint_mapping_provisional.json`
- 출력 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f/URDF_F_named.xml`
- 변환된 pose config: `/home/king0519/projects/Humanoid/configs/standing_pose_candidate_named.json`
- 리포트 JSON: `/home/king0519/projects/Humanoid/docs/hardware_validation/named_model_report.json`

## 컴파일 확인

- nq: `17`
- nv: `16`
- nu: `10`
- joint 수: `11`
- geom 수: `26`

## Joint 이름

- `floating_base`
- `left_hip_roll`
- `left_hip_pitch`
- `left_knee_pitch`
- `left_ankle_pitch`
- `left_ankle_roll`
- `right_hip_roll`
- `right_hip_pitch`
- `right_knee_pitch`
- `right_ankle_pitch`
- `right_ankle_roll`

## Actuator 이름

- `motor_left_hip_roll`
- `motor_left_hip_pitch`
- `motor_left_knee_pitch`
- `motor_left_ankle_pitch`
- `motor_left_ankle_roll`
- `motor_right_hip_roll`
- `motor_right_hip_pitch`
- `motor_right_knee_pitch`
- `motor_right_ankle_pitch`
- `motor_right_ankle_roll`

## 판단

이제 로그와 그래프에서 `left_knee_pitch`, `right_ankle_roll` 같은 이름을 사용할 수 있다. 다만 positive direction과 정확한 관절 역할은 아직 CAD 확인이 필요하다.
