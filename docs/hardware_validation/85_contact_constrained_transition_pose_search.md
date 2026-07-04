# Contact-constrained Transition Pose Search

## 목적

84번에서 force-ratio feedback만으로는 안정 하중비가 약 `0.62` 근처에서 포화되는 것을 확인했다.

이번 단계에서는 controller를 더 키우는 대신, transition-aware pose search 자체를 개선했다. 기존 `0.65` pose는 seed로 쓰지 않고, 성공한 `0.62 transition-aware` pose를 기준으로 새 `0.65` pose family를 찾았다.

핵심 변화:

- `left_force_ratio`만 보지 않는다.
- `left/right contact 수`, `normal force 유지`, `roll 제한`을 score에 직접 추가한다.
- 붕괴하면서 하중비만 맞는 후보를 강하게 배제한다.

## 수정한 스크립트

```text
scripts/search_weight_shift_transition_pose.py
```

추가한 score 항목:

- `final_left_contacts`
- `final_right_contacts`
- rollout 중 최소 `left_contacts`
- rollout 중 최소 `right_contacts`
- 최종 normal force
- max roll limit 초과 벌점
- final roll limit 초과 벌점

## 0.65 Contact-constrained Search

입력:

```text
start pose: configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json
seed pose:  configs/weight_shift_left062_transition_aware_multipoint.json
target left_force_ratio: 0.65
```

실행:

```bash
python3 scripts/search_weight_shift_transition_pose.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics_multipoint_fullheight_contact.xml --start-pose configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json --seed-end-pose configs/weight_shift_left062_transition_aware_multipoint.json --out configs/weight_shift_left065_contact_constrained_multipoint.json --out-dir outputs/analysis/weight_shift_transition_search_left065_contact_constrained_multipoint --target-left-ratio 0.65 --samples 180 --iterations 3 --elite 18 --seed 311 --sigma 0.055 --min-sigma 0.008 --duration 8.0 --ramp 4.0 --min-left-contacts 8 --min-right-contacts 2 --left-contact-weight 8 --right-contact-weight 8 --min-left-contact-weight 2 --min-right-contact-weight 2 --min-normal-force 85 --normal-force-weight 1 --max-roll-limit 0.25 --final-roll-limit 0.18 --max-roll-limit-weight 80 --final-roll-limit-weight 100
```

탐색 결과:

| 항목 | 값 |
| --- | ---: |
| target left ratio | `0.65` |
| final left ratio | `0.649919` |
| ratio error | `0.000081` |
| final roll | `-0.027503 rad` |
| final pitch | `-0.091675 rad` |
| final yaw | `-0.112615 rad` |
| final qvel | `0.007626` |
| final normal force | `90.363613 N` |
| final left contacts | `11` |
| final right contacts | `4` |
| max abs roll | `0.053874 rad` |
| max qvel | `0.576414` |
| max torque | `3.748796 Nm` |
| contact fraction | `1.0` |
| saturation fraction | `0.0` |

생성 pose:

```text
configs/weight_shift_left065_contact_constrained_multipoint.json
```

## 0.65 영상 검증

8초 전이:

| 항목 | 값 |
| --- | ---: |
| final left ratio | `0.649919` |
| final roll | `-0.027503 rad` |
| final pitch | `-0.091675 rad` |
| final qvel | `0.007626` |
| final contacts | `15` |
| max torque | `3.748796 Nm` |
| saturation fraction | `0.0` |

12초 유지:

| 항목 | 값 |
| --- | ---: |
| final left ratio | `0.654165` |
| final roll | `-0.023042 rad` |
| final pitch | `-0.093073 rad` |
| final qvel | `0.002606` |
| final contacts | `16` |
| max abs roll | `0.053701 rad` |
| max abs pitch | `0.093183 rad` |
| max qvel | `0.567728` |
| max torque | `3.748796 Nm` |
| contact frame fraction | `1.0` |
| saturation fraction | `0.0` |

판단:

- 성공이다.
- 기존 `0.65` static pose는 실패했지만, contact-constrained transition-aware search로 새로 찾은 `0.65` pose는 안정적으로 들어간다.
- 이 결과는 하드웨어/기구 자체가 `0.65` 하중 이동을 절대 못 하는 것은 아니라는 뜻이다.
- 이전 실패의 핵심 원인은 pose family/search objective가 잘못되어 접촉을 잃는 pose를 고른 것이었다.

영상:

```text
outputs/analysis/render_pose_transition_left065_contact_constrained_multipoint_8s/pose_transition_render.mp4
outputs/analysis/render_pose_transition_left065_contact_constrained_multipoint_12s/pose_transition_render.mp4
```

## 0.70 Contact-constrained Search

입력:

```text
start pose: configs/weight_shift_left065_contact_constrained_multipoint.json
seed pose:  configs/weight_shift_left065_contact_constrained_multipoint.json
target left_force_ratio: 0.70
```

탐색 결과:

| 항목 | 값 |
| --- | ---: |
| target left ratio | `0.70` |
| best final left ratio | `0.588701` |
| ratio error | `0.111299` |
| final roll | `2.546033 rad` |
| final pitch | `0.100149 rad` |
| final yaw | `1.059473 rad` |
| max abs roll | `3.140586 rad` |
| max qvel | `5.995463` |
| max contact force | `406.300145 N` |
| max torque | `16.411162 Nm` |
| min left contacts | `0` |
| min right contacts | `0` |

판단:

- 실패다.
- contact-constrained objective를 넣어도 `0.70`은 안정 후보를 찾지 못했다.
- 현재 검증된 안정 weight-shift milestone은 `0.65`까지다.

## 현재 결론

현재 안정적으로 검증된 경로:

```text
standing -> left_force_ratio 약 0.65
```

현재 실패하는 목표:

```text
0.65 -> 0.70
```

중요한 해석:

1. `0.65`는 가능하다.
2. `0.65` 성공에는 단순 하중비 목적함수가 아니라 contact 유지 목적함수가 필요했다.
3. `0.70`은 같은 방식으로도 실패했다.
4. 실패 시 torque saturation은 여전히 주된 원인이 아니다.
5. 이 결과는 RL 전에 반드시 reward/objective에 contact 유지와 roll 안정성을 같이 넣어야 함을 보여준다.

## 다음 조치

다음 단계는 `0.65`에서 swing foot 직전 조건을 확인하는 것이다.

추천:

1. `0.65` pose에서 오른발 하중이 충분히 줄었는지 확인한다.
2. 오른발 normal force가 `31 N` 수준이라 아직 swing foot lift에는 부족할 가능성이 높다.
3. 다음 목표는 `0.70`을 다시 무작정 찾는 것이 아니라, `right foot unload` 목적함수와 `right contact force 감소`를 직접 넣은 search다.
4. 목표를 left ratio 하나로 두지 말고 `right_force < 20 N`, `right_contacts 유지/감소 조건`, `roll < 0.2`를 함께 봐야 한다.

## 산출물

- 새 `0.65` pose: `/home/king0519/projects/Humanoid/configs/weight_shift_left065_contact_constrained_multipoint.json`
- `0.65` 탐색 결과: `/home/king0519/projects/Humanoid/outputs/analysis/weight_shift_transition_search_left065_contact_constrained_multipoint/weight_shift_transition_search_summary.json`
- `0.65` 8초 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_pose_transition_left065_contact_constrained_multipoint_8s/pose_transition_render.mp4`
- `0.65` 12초 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_pose_transition_left065_contact_constrained_multipoint_12s/pose_transition_render.mp4`
- `0.70` 실패 탐색 결과: `/home/king0519/projects/Humanoid/outputs/analysis/weight_shift_transition_search_left070_contact_constrained_multipoint/weight_shift_transition_search_summary.json`
