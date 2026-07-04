# Pose Support 및 Soft-contact 비교 검증

## 목적

71번 영상에서 로봇이 여전히 standing에 실패했기 때문에, 실패 원인을 더 분리한다.

이번 단계의 질문은 두 가지다.

1. 현재 contact pose의 COM 투영이 발 지지 영역 밖에 있는가?
2. MuJoCo 기본 접촉이 너무 딱딱해서 튕기는 것이 주된 원인인가?

## 입력

- 원본 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml`
- pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json`
- pose/support 감사 스크립트: `/home/king0519/projects/Humanoid/scripts/audit_pose_support.py`
- soft-contact 생성 스크립트: `/home/king0519/projects/Humanoid/scripts/build_soft_contact_variant.py`

## 1. Pose/support 감사

실행:

```bash
python3 scripts/audit_pose_support.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml --pose-json configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json --out-dir outputs/analysis/pose_support_audit_toeheel_z_step_visual_contact
```

결과:

| 항목 | 값 |
| --- | ---: |
| base z | `0.001021732 m` |
| total mass | `9.2114 kg` |
| COM x | `0.066400 m` |
| COM y | `-0.052086 m` |
| COM z | `0.286210 m` |
| support x min/max | `-0.025853 / 0.160785 m` |
| support y min/max | `-0.089232 / -0.013382 m` |
| x min/max margin | `0.092253 / 0.094385 m` |
| y min/max margin | `0.037146 / 0.038704 m` |
| minimum support margin | `0.037146 m` |
| lowest contact z | `-0.001000 m` |
| initial contacts | `3` |

판단:

- COM 투영은 support bounds 안에 있다.
- 최소 margin도 약 `37 mm`라서, 현재 실패를 단순히 "초기 COM이 발 밖에 있음"으로 설명하기 어렵다.
- 초기 접촉도 `3`개가 잡히므로, 완전히 공중에서 떨어지는 테스트는 아니다.

산출물:

- `/home/king0519/projects/Humanoid/outputs/analysis/pose_support_audit_toeheel_z_step_visual_contact/pose_support_audit.json`

## 2. Soft-contact 모델 생성

원본 모델은 유지하고, floor와 양발 sole collision geom에만 `solref/solimp/friction`을 명시한 변형 모델을 만들었다.

실행:

```bash
python3 scripts/build_soft_contact_variant.py --input envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml --output envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_soft_contact.xml --report docs/hardware_validation/soft_contact_variant_report.json
```

적용값:

| 항목 | 값 |
| --- | --- |
| solref | `0.02 1` |
| solimp | `0.9 0.95 0.001` |
| friction | `1.2 0.03 0.003` |
| condim | `3` |

생성 모델:

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_soft_contact.xml
```

로드 확인:

| 항목 | 값 |
| --- | ---: |
| nq | `17` |
| nv | `16` |
| nu | `10` |
| nbody | `12` |
| njnt | `11` |
| ngeom | `26` |
| timestep | `0.002` |

## 3. Soft-contact high-gain PD probe

실행:

```bash
python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_soft_contact.xml --pose-json configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json --out-dir outputs/analysis/pd_standing_toeheel_z_step_visual_soft_contact_pose --duration 2.0 --kp 60 --kd 4 --torque-limit 100
```

원본 모델과 soft-contact 모델 비교:

| 항목 | 원본 contact | soft contact |
| --- | ---: | ---: |
| final base z | `0.016142 m` | `-0.009768 m` |
| final roll | `2.607548 rad` | `2.281144 rad` |
| final pitch | `-0.273205 rad` | `-0.328932 rad` |
| max qvel norm | `151.063643` | `160.612306` |
| max contact force | `1684.520266 N` | `1501.616952 N` |
| ankle torque saturation | 있음 | 있음 |

판단:

- soft-contact에서 최대 접촉력과 final roll은 조금 줄었다.
- 하지만 여전히 큰 roll 회전과 ankle torque saturation이 발생한다.
- 따라서 현재 실패는 "접촉이 너무 딱딱해서 생긴 문제"만으로 설명되지 않는다.

## 4. 영상 산출물

### Soft-contact PD 영상

| 항목 | 경로 |
| --- | --- |
| MP4 | `/home/king0519/projects/Humanoid/outputs/analysis/render_pd_standing_toeheel_z_step_visual_soft_contact_pose/pd_standing_render.mp4` |
| GIF | `/home/king0519/projects/Humanoid/outputs/analysis/render_pd_standing_toeheel_z_step_visual_soft_contact_pose/pd_standing_render.gif` |
| 첫 프레임 | `/home/king0519/projects/Humanoid/outputs/analysis/render_pd_standing_toeheel_z_step_visual_soft_contact_pose/first_frame.png` |
| 중간 프레임 | `/home/king0519/projects/Humanoid/outputs/analysis/render_pd_standing_toeheel_z_step_visual_soft_contact_pose/mid_frame.png` |
| 마지막 프레임 | `/home/king0519/projects/Humanoid/outputs/analysis/render_pd_standing_toeheel_z_step_visual_soft_contact_pose/last_frame.png` |

영상 요약:

- 중간 프레임 약 `1.024 s`: contact `1`, roll `+1.90 rad`, torque `100 Nm`
- 마지막 프레임 약 `2.0 s`: contact `0`, roll `+2.28 rad`, torque `100 Nm`

### Soft-contact axis-aware 영상

| 항목 | 경로 |
| --- | --- |
| MP4 | `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_toeheel_z_step_visual_soft_contact/axis_aware_standing_render.mp4` |
| GIF | `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_toeheel_z_step_visual_soft_contact/axis_aware_standing_render.gif` |
| 첫 프레임 | `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_toeheel_z_step_visual_soft_contact/first_frame.png` |
| 중간 프레임 | `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_toeheel_z_step_visual_soft_contact/mid_frame.png` |
| 마지막 프레임 | `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_toeheel_z_step_visual_soft_contact/last_frame.png` |

영상 요약:

- contact frame fraction: `0.1875`
- final roll: `-1.940833 rad`
- final pitch: `0.145799 rad`
- max qvel norm: `213.005217`
- max contact force: `1825.455021 N`
- max torque: `100 Nm`

## 결론

이번 단계에서 두 가지를 분리했다.

1. 현재 pose는 COM 투영만 보면 support 영역 안에 있다.
2. 접촉을 부드럽게 만들어도 standing 실패는 해결되지 않는다.

따라서 다음 우선순위는 접촉 파라미터가 아니라 `joint axis/부호/기구학적 지지 능력`이다. 특히 ankle 쪽 actuator가 하중을 받는 순간 계속 saturation에 들어가므로, 실제 CAD에서 ankle pitch/roll 축과 MuJoCo joint axis가 의도대로 들어갔는지 더 좁혀서 확인해야 한다.

## 다음 조치

1. ankle과 knee의 정적 하중 지지 토크를 계산한다.
2. 현재 pose에서 각 joint가 중력에 대해 요구하는 feedforward torque를 구한다.
3. 그 토크가 AK45-36, AK45-10의 연속/피크 토크 범위와 비교해 가능한 수준인지 확인한다.
4. 이 결과가 괜찮으면 joint axis 방향/부호를 CAD 기준으로 재작성한 모델 변형을 만든다.
