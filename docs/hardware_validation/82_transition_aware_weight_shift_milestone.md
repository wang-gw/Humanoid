# Transition-aware Weight Shift Milestone

## 목적

81번에서 정적 weight-shift pose를 찾았지만, standing에서 해당 pose로 전이하면 `0.65` 이상에서 넘어졌다.

이번 단계에서는 후보 pose를 정적 초기 상태로 평가하지 않고, 실제로 `standing -> 후보 pose` 전이를 수행한 결과로 직접 점수화했다.

## 추가한 스크립트

- transition-aware pose 탐색: `/home/king0519/projects/Humanoid/scripts/search_weight_shift_transition_pose.py`
- 여러 pose 연속 렌더링: `/home/king0519/projects/Humanoid/scripts/render_pose_sequence.py`

핵심 차이:

- 기존 `search_weight_shift_pose.py`: 후보 pose에서 바로 시작해서 안정성을 본다.
- 새 `search_weight_shift_transition_pose.py`: standing pose에서 후보 pose까지 ramp로 이동한 뒤 최종 상태를 평가한다.

## 정적 pose 탐색의 한계

`0.62` 정적 pose는 6초 유지 기준으로 좋아 보였다.

| 항목 | 값 |
| --- | ---: |
| target left ratio | `0.62` |
| final left ratio | `0.649123` |
| final roll | `0.014294 rad` |
| final qvel | `0.013975` |
| contact fraction | `1.0` |
| saturation fraction | `0.0` |

하지만 standing에서 이 pose로 전이한 뒤 12초 유지하면 실패했다.

| 항목 | 값 |
| --- | ---: |
| final left ratio | `0.485658` |
| final roll | `2.652534 rad` |
| max qvel | `8.186594` |
| max contact force | `341.668075 N` |
| max torque | `15.862254 Nm` |

영상:

```text
outputs/analysis/render_pose_transition_left062_6s_from_standing_multipoint_12s/pose_transition_render.mp4
```

판단:

- pose 자체가 완전히 의미 없는 것은 아니다.
- 하지만 그 pose가 실제 전이 trajectory의 안정 목표가 된다는 보장은 없다.
- 따라서 이후 milestone 탐색은 transition-aware 기준으로 해야 한다.

## Transition-aware 0.62 탐색

실행:

```bash
python3 scripts/search_weight_shift_transition_pose.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics_multipoint_fullheight_contact.xml --start-pose configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json --seed-end-pose configs/weight_shift_left062_pose_6s_multipoint.json --out configs/weight_shift_left062_transition_aware_multipoint.json --out-dir outputs/analysis/weight_shift_transition_search_left062_multipoint --target-left-ratio 0.62 --samples 120 --iterations 2 --elite 16 --seed 223 --sigma 0.05 --min-sigma 0.008 --duration 8.0 --ramp 4.0
```

탐색 결과:

| 항목 | 값 |
| --- | ---: |
| target left ratio | `0.62` |
| final left ratio | `0.617126` |
| ratio error | `0.002874` |
| final roll | `0.101468 rad` |
| final pitch | `-0.061111 rad` |
| final yaw | `-0.116731 rad` |
| final qvel | `0.011757` |
| max qvel | `0.576503` |
| max contact force | `99.857139 N` |
| max torque | `3.757246 Nm` |
| contact fraction | `1.0` |
| saturation fraction | `0.0` |

생성 pose:

```text
configs/weight_shift_left062_transition_aware_multipoint.json
```

## Transition-aware 0.62 영상 검증

8초 전이:

| 항목 | 값 |
| --- | ---: |
| final left ratio | `0.617126` |
| final roll | `0.101468 rad` |
| final pitch | `-0.061111 rad` |
| final qvel | `0.011757` |
| final contacts | `20` |
| max torque | `3.757246 Nm` |

12초 유지:

| 항목 | 값 |
| --- | ---: |
| final left ratio | `0.613134` |
| final roll | `0.105340 rad` |
| final pitch | `-0.066041 rad` |
| final qvel | `0.007418` |
| final contacts | `20` |
| max qvel | `0.576503` |
| max torque | `3.757246 Nm` |
| contact frame fraction | `1.0` |
| saturation fraction | `0.0` |

판단:

- `standing -> 0.62 transition-aware pose`는 성공이다.
- 12초까지 넘어지지 않고 하중비 약 `0.61 / 0.39`를 유지한다.
- 지금까지 확인한 가장 높은 안정 weight-shift milestone이다.

영상:

```text
outputs/analysis/render_pose_transition_left062_transition_aware_multipoint_8s/pose_transition_render.mp4
outputs/analysis/render_pose_transition_left062_transition_aware_multipoint_12s/pose_transition_render.mp4
```

## 0.65 재시도

`0.62` transition-aware pose를 시작점으로 `0.65`를 탐색했다.

결과:

| 항목 | 값 |
| --- | ---: |
| target left ratio | `0.65` |
| final left ratio | `0.645900` |
| final roll | `2.645799 rad` |
| max abs roll | `3.140764 rad` |
| max qvel | `6.115914` |
| max contact force | `423.233887 N` |
| max torque | `15.456075 Nm` |
| contact fraction | `0.998000` |
| saturation fraction | `0.0` |

판단:

- 하중비 숫자는 맞았지만, roll이 너무 커서 실패다.
- torque saturation이 없으므로 모터 토크 부족이 아니라 자세/접촉/제어 안정성 문제다.

## 0.63 재시도

`0.62` transition-aware pose를 시작점으로 `0.63`도 확인했다.

결과:

| 항목 | 값 |
| --- | ---: |
| target left ratio | `0.63` |
| final left ratio | `0.599983` |
| final roll | `2.670430 rad` |
| max abs roll | `3.141347 rad` |
| max qvel | `6.092876` |
| max contact force | `430.996246 N` |
| max torque | `15.507406 Nm` |

판단:

- 이 설정에서는 `0.62` pose를 새 초기 상태로 두는 것 자체가 실제 연속 도달 상태와 다르다.
- 즉 단순 pose-to-pose 초기화 방식은 연속 trajectory의 상태를 정확히 재현하지 못한다.

## 연속 Sequence 검증

한 시뮬레이션 안에서 다음 순서를 연결했다.

```text
standing -> transition-aware 0.62 -> static 0.65
```

결과:

| 항목 | 값 |
| --- | ---: |
| final left ratio | `0.477118` |
| final roll | `3.016031 rad` |
| final pitch | `1.243949 rad` |
| final qvel | `1.958243` |
| max qvel | `8.214139` |
| max contact force | `599.047606 N` |
| max torque | `17.123201 Nm` |

판단:

- 연속 상태를 유지해도 `0.65`로 넘어가면 실패한다.
- 현재 controller/pose family에서 안정 상한은 `0.61~0.62` 부근이다.

영상:

```text
outputs/analysis/render_pose_sequence_standing_062_065_multipoint/pose_sequence_render.mp4
```

## 현재 결론

현재 검증된 안정 경로:

```text
standing -> left_force_ratio 약 0.61
```

아직 실패하는 경로:

```text
standing -> 0.65
standing -> 0.62 -> 0.65
```

중요한 해석:

1. `0.61~0.62`까지의 하중 이동은 실제 전이 가능하다.
2. `0.65` 이상은 현재 방식으로는 roll 붕괴가 발생한다.
3. 실패 시에도 torque saturation은 없다.
4. 따라서 다음 병목은 모터 토크가 아니라 lateral balance controller, 발 접촉 모멘트 사용 방식, 또는 기구학적 하중 이동 능력이다.

## 다음 조치

다음 단계는 `0.62`에서 더 큰 하중 이동을 강제로 만들기보다, roll 붕괴 원인을 분리해야 한다.

추천 순서:

1. `0.62` 안정 pose에서 roll/pitch 역할 actuator별 torque contribution을 기록한다.
2. `0.65` 실패 전이에서 roll이 커지기 시작하는 시점의 contact force 분포와 torque를 기록한다.
3. ankle roll 역할 joint와 hip roll 역할 joint를 분리해서 어느 쪽이 하중 이동을 망가뜨리는지 확인한다.
4. 필요하면 lateral controller를 `COM error`가 아니라 `left/right force ratio feedback` 기반으로 바꾼다.

## 산출물

- transition-aware `0.62` pose: `/home/king0519/projects/Humanoid/configs/weight_shift_left062_transition_aware_multipoint.json`
- transition-aware 탐색 결과: `/home/king0519/projects/Humanoid/outputs/analysis/weight_shift_transition_search_left062_multipoint/weight_shift_transition_search_summary.json`
- `0.62` 8초 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_pose_transition_left062_transition_aware_multipoint_8s/pose_transition_render.mp4`
- `0.62` 12초 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_pose_transition_left062_transition_aware_multipoint_12s/pose_transition_render.mp4`
- `standing -> 0.62 -> 0.65` sequence 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_pose_sequence_standing_062_065_multipoint/pose_sequence_render.mp4`
