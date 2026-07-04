# Quasi-static Standing Pose Search

## 목적

neutral `0 rad` pose에서 standing이 실패했기 때문에, COM이 support 중심에 가깝고 양발 높이/수평성이 나은 준정적 standing 후보를 탐색한다.

## 실행 명령

```bash
python3 scripts/search_quasistatic_standing_pose.py
```

## 결과

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml`
- 후보 pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_actuator_dynamics.json`
- cost: `0.055434`
- base z: `0.006691`
- COM world: `[0.06523818645156855, -0.05181400073010011, 0.2911013433714675]`
- support margins: `[0.09029607302463175, 0.08918474664117325, 0.03910992696072752, 0.0383848612691686]`
- foot low z: `[0.005274085280450709, 0.0049999999999999455]`
- foot height diff: `0.000274085`
- foot normal penalty: `0.000020014`
- COM center penalty: `0.000125880`

## Joint Targets

- `left_hip_roll`: `0.110137` rad
- `left_hip_pitch`: `-0.057183` rad
- `left_knee_pitch`: `-0.071829` rad
- `left_ankle_pitch`: `0.103184` rad
- `left_ankle_roll`: `-0.029198` rad
- `right_hip_roll`: `0.260314` rad
- `right_hip_pitch`: `0.088046` rad
- `right_knee_pitch`: `0.006663` rad
- `right_ankle_pitch`: `-0.006733` rad
- `right_ankle_roll`: `-0.270889` rad

## 판단

이 pose는 동역학적으로 검증된 standing이 아니라, 다음 PD probe에 넣을 준정적 후보이다. 이 후보에서도 실패하면 단순 pose 문제가 아니라 controller/contact/inertial frame 문제가 남아 있다고 본다.
