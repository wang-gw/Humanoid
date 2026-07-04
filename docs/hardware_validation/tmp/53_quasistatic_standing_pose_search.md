# Quasi-static Standing Pose Search

## 목적

neutral `0 rad` pose에서 standing이 실패했기 때문에, COM이 support 중심에 가깝고 양발 높이/수평성이 나은 준정적 standing 후보를 탐색한다.

## 실행 명령

```bash
python3 scripts/search_quasistatic_standing_pose.py
```

## 결과

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml`
- 후보 pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_toeheel_z_step_visual.json`
- cost: `0.061044`
- base z: `0.007022`
- COM world: `[0.06640024715784057, -0.05208608527345414, 0.2922103142262856]`
- support margins: `[0.09225330596979217, 0.09438462027929315, 0.037145881670600636, 0.03870404775356158]`
- foot low z: `[0.005608441326277861, 0.005000000000000015]`
- foot height diff: `0.000608441`
- foot normal penalty: `0.000040512`
- COM center penalty: `0.000552410`

## Joint Targets

- `left_hip_roll`: `0.127293` rad
- `left_hip_pitch`: `-0.079019` rad
- `left_knee_pitch`: `-0.013438` rad
- `left_ankle_pitch`: `0.009604` rad
- `left_ankle_roll`: `-0.057963` rad
- `right_hip_roll`: `0.256924` rad
- `right_hip_pitch`: `0.055126` rad
- `right_knee_pitch`: `-0.047217` rad
- `right_ankle_pitch`: `-0.027527` rad
- `right_ankle_roll`: `-0.253985` rad

## 판단

이 pose는 동역학적으로 검증된 standing이 아니라, 다음 PD probe에 넣을 준정적 후보이다. 이 후보에서도 실패하면 단순 pose 문제가 아니라 controller/contact/inertial frame 문제가 남아 있다고 본다.
