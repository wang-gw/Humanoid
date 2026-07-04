# Multipoint Contact Weight Shift Probe

## 목적

79번에서 standing은 통과했지만 weight shift가 부분 통과에 그쳤다.

이번 단계에서는 발바닥 contact를 단일 box에서 4개 pad로 나누면 좌우 하중 이동이 좋아지는지 확인했다.

## 기준

- 기준 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml`
- 기준 pose: `/home/king0519/projects/Humanoid/configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json`
- contact 생성 스크립트: `/home/king0519/projects/Humanoid/scripts/build_multipoint_foot_contact_variant.py`
- 수정된 support 계산: `/home/king0519/projects/Humanoid/scripts/sweep_stabilized_standing.py`

## 생성한 모델

단일 sole collision을 제거하고, 각 발의 기존 sole footprint를 4개 box pad로 분할했다.

생성 명령:

```bash
python3 scripts/build_multipoint_foot_contact_variant.py --source envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml --out envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics_multipoint_fullheight_contact.xml --report docs/hardware_validation/multipoint_fullheight_foot_contact_variant_report.json --pad-height 0.0485 --overlap 0.002
```

생성 모델:

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics_multipoint_fullheight_contact.xml
```

| 항목 | 값 |
| --- | ---: |
| pad 개수 | 발당 4개, 총 8개 |
| pad height | `0.0485 m` |
| overlap | `0.002 m` |
| friction | `1.0 0.02 0.001` |
| ngeom | `32` |

처음에는 얇은 pad height `0.012 m` 모델도 만들었지만 standing이 깨졌다. 최종 비교에는 원래 sole box와 같은 높이인 `0.0485 m` full-height pad 모델을 사용했다.

## Support Polygon 계산 수정

기존 `support_center()`는 아래 두 geom만 사용했다.

```text
foot_L_1_sole_collision
foot_R_v1_1_sole_collision
```

다점 contact 모델에서는 이 이름의 geom이 제거되고 다음 prefix의 pad들이 생긴다.

```text
foot_L_1_sole_pad_*
foot_R_v1_1_sole_pad_*
```

따라서 `scripts/sweep_stabilized_standing.py`의 `support_center()`를 수정해서 단일 sole과 다점 pad를 모두 support polygon에 포함하도록 했다. 이 수정 없이는 controller가 잘못된 support center를 사용해서 standing 결과가 비정상적으로 나온다.

## 10초 Standing 재검증

결과:

| 항목 | 값 |
| --- | ---: |
| contact frame fraction | `1.0` |
| max qvel norm | `0.576433` |
| max contact force | `99.856370 N` |
| max torque | `3.798335 Nm` |
| final base z | `-0.000517 m` |
| final roll | `0.012219 rad` |
| final pitch | `-0.006346 rad` |
| final contacts | `10` |

판단:

- 다점 full-height contact 모델은 standing을 통과한다.
- 단일 contact 모델보다 final pitch가 작고, contact 수가 많아졌다.
- max torque는 여전히 낮다.

영상:

```text
outputs/analysis/render_axis_aware_standing_actuator_dynamics_multipoint_fullheight_contact_10s_fixed_support/axis_aware_standing_render.mp4
```

## Weight Shift 재검증

### Case 1: amplitude 0.18, Kcom 1

| 항목 | 단일 contact | 다점 contact |
| --- | ---: | ---: |
| contact frame fraction | `1.0` | `1.0` |
| max qvel norm | `0.587915` | `0.576466` |
| max contact force | `105.095404 N` | `99.855234 N` |
| max torque | `3.764081 Nm` | `3.765741 Nm` |
| final roll | `0.004698 rad` | `0.010884 rad` |
| final pitch | `-0.047473 rad` | `-0.007049 rad` |
| final left force ratio | `0.502480` | `0.583180` |

초기 1초 이후 비교:

| 항목 | 단일 contact | 다점 contact |
| --- | ---: | ---: |
| left ratio min/max | `0.502480 / 0.559585` | `0.551193 / 0.583180` |
| left ratio range | `0.057105` | `0.031987` |
| max abs roll | `0.025683 rad` | `0.024482 rad` |
| max abs pitch | `0.047473 rad` | `0.021913 rad` |
| max qvel norm | `0.132330` | `0.120796` |

판단:

- 다점 contact는 pitch 흔들림과 qvel을 줄였다.
- 하지만 좌우 하중 이동 폭은 커지지 않았다.
- 오히려 안정적인 범위 안에서는 left force ratio range가 더 작다.

영상:

```text
outputs/analysis/render_weight_shift_actuator_dynamics_multipoint_fullheight_amp018/weight_shift_render.mp4
```

### Case 2: amplitude 0.25, Kcom 1

| 항목 | 값 |
| --- | ---: |
| contact frame fraction | `1.0` |
| saturation fraction | `0.0` |
| max qvel norm | `0.576478` |
| max contact force | `99.854792 N` |
| max torque | `3.746413 Nm` |
| final roll | `0.010262 rad` |
| final pitch | `-0.007317 rad` |
| final left force ratio | `0.583479` |

초기 1초 이후:

| 항목 | 값 |
| --- | ---: |
| left ratio min/max | `0.551153 / 0.583479` |
| left ratio range | `0.032326` |
| max abs roll | `0.024310 rad` |
| max abs pitch | `0.022048 rad` |
| max qvel norm | `0.118222` |

판단:

- amplitude를 `0.25`로 키워도 Kcom이 낮으면 안정성은 유지된다.
- 하지만 실제 하중 이동 폭은 amplitude `0.18`과 거의 같다.
- 현재 controller가 lateral target을 강하게 추종하지 못하고 있다.

영상:

```text
outputs/analysis/render_weight_shift_actuator_dynamics_multipoint_fullheight_amp025_kcom1/weight_shift_render.mp4
```

### Case 3: amplitude 0.25, Kcom 2

| 항목 | 값 |
| --- | ---: |
| contact frame fraction | `1.0` |
| saturation fraction | `0.0` |
| max qvel norm | `7.492421` |
| max contact force | `623.583309 N` |
| max torque | `15.860342 Nm` |
| final roll | `2.902982 rad` |
| final pitch | `-0.150298 rad` |
| final left force ratio | `0.150935` |

판단:

- 실패다.
- contact는 유지되지만 roll 방향으로 크게 붕괴한다.
- 단일 contact에서의 중간 weight shift 실패와 같은 양상이다.

영상:

```text
outputs/analysis/render_weight_shift_actuator_dynamics_multipoint_fullheight_amp025_kcom2/weight_shift_render.mp4
```

## 결론

다점 contact는 standing 안정성과 pitch 흔들림에는 도움이 된다. 하지만 이번 결과만 보면, weight shift 실패의 주원인은 단순히 발바닥 contact box가 하나라서 생긴 문제만은 아니다.

확인된 점:

1. full-height 다점 contact 모델은 10초 standing을 통과한다.
2. 작은 weight shift에서는 다점 contact가 더 조용한 자세를 만든다.
3. 안정 조건에서 실제 좌우 하중 이동 폭은 여전히 작다.
4. Kcom을 키우면 여전히 roll 붕괴가 발생한다.
5. torque saturation은 발생하지 않으므로, 현재 실패는 모터 토크 부족보다는 제어/기구학/접촉 모멘트 사용 방식 문제에 가깝다.

따라서 다음 단계는 contact를 더 쪼개는 것이 아니라, weight shift 전용 trajectory와 lateral balance controller를 새로 잡는 것이다.

## 다음 조치

1. 좌우 hip roll 역할 joint와 ankle roll 역할 joint를 분리해서 weight shift용 pose trajectory를 만든다.
2. pelvis/base를 목표 발 위로 천천히 이동시키는 quasi-static trajectory를 만든다.
3. 각 phase에서 left/right normal force ratio를 목적함수로 두고 pose를 탐색한다.
4. 목표 force ratio 예시는 `0.75 / 0.25` 또는 `0.85 / 0.15`다.
5. 이 trajectory가 성공하면 그 다음에 single support 직전 pose와 swing foot lift 가능성을 확인한다.

## 산출물

- 다점 contact 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics_multipoint_fullheight_contact.xml`
- 생성 리포트: `/home/king0519/projects/Humanoid/docs/hardware_validation/multipoint_fullheight_foot_contact_variant_report.json`
- standing 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_actuator_dynamics_multipoint_fullheight_contact_10s_fixed_support/axis_aware_standing_render.mp4`
- amplitude 0.18 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_weight_shift_actuator_dynamics_multipoint_fullheight_amp018/weight_shift_render.mp4`
- amplitude 0.25, Kcom 1 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_weight_shift_actuator_dynamics_multipoint_fullheight_amp025_kcom1/weight_shift_render.mp4`
- amplitude 0.25, Kcom 2 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_weight_shift_actuator_dynamics_multipoint_fullheight_amp025_kcom2/weight_shift_render.mp4`
