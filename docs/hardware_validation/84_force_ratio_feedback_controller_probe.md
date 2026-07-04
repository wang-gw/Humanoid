# Force Ratio Feedback Controller Probe

## 목적

83번에서 `0.62 -> 0.65` 실패는 torque saturation이 아니라, 목표와 반대로 왼발 하중이 빠지고 오른발 rear pad로 하중이 몰리는 문제임을 확인했다.

이번 단계에서는 `left_force_ratio`를 직접 feedback으로 쓰는 controller를 추가해, 하중비를 `0.65`까지 안정적으로 올릴 수 있는지 확인했다.

## 추가한 스크립트

```text
scripts/render_force_ratio_sequence.py
```

기존 `render_pose_sequence.py`에 다음 feedback을 추가했다.

```text
force_error = target_left_ratio - measured_left_force_ratio
force_term = force_sign * kforce * force_error
```

그리고 `force_term`을 선택한 actuator group에 더했다.

지원 옵션:

- `force-role`: `roll`, `pitch`, `both`
- `force-on`: `hip`, `ankle`, `all`
- `force-side-mode`: `same`, `opposite`

`opposite` 모드는 좌우 actuator에 반대 부호의 보정 torque를 넣는다.

## 테스트 1: 기존 0.65 pose에 force feedback 추가

입력 sequence:

```text
standing -> transition-aware 0.62 -> static 0.65
```

대표 결과:

| case | final L | final roll | final pitch | qvel | max qvel | max tau | contacts | stable? |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `bad_pose_no_force` | `0.477` | `3.016` | `1.244` | `1.958` | `8.214` | `17.123` | `3` | no |
| `bad_pose_roll_all_k2_pos` | `0.611` | `2.706` | `1.168` | `2.573` | `8.044` | `17.170` | `4` | no |
| `bad_pose_pitch_ankle_k5_opp` | `0.709` | `2.123` | `0.328` | `2.046` | `8.409` | `20.098` | `2` | no |

판단:

- force feedback을 넣으면 하중비 숫자는 순간적으로 좋아질 수 있다.
- 하지만 자세가 이미 무너진 상태라 성공이 아니다.
- `static 0.65` pose는 안정 trajectory 목표로 부적합하다.

특히 `pitch-role ankle differential, k=5`는 final left ratio `0.709`까지 만들었지만, final roll이 `2.123 rad`라서 실패다.

영상:

```text
outputs/analysis/force_ratio_seq_062_065_pitch_ankle_k5_opp_pos/force_ratio_sequence_render.mp4
```

## 테스트 2: 0.65 pose 없이 0.62 pose를 유지하며 force target만 0.65

입력 sequence:

```text
standing -> transition-aware 0.62 -> transition-aware 0.62
```

목표 force ratio:

```text
0.50 -> 0.62 -> 0.65
```

대표 결과:

| case | final L | final roll | final pitch | qvel | max qvel | max tau | contacts | stable? |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `hold062_roll_all_k5_same` | `0.585` | `0.098` | `-0.083` | `0.008` | `0.612` | `4.450` | `20` | yes |
| `hold062_roll_hip_k5_opp` | `0.616` | `0.103` | `-0.065` | `0.005` | `0.701` | `4.421` | `20` | yes |
| `hold062_roll_ankle_k5_opp` | `0.603` | `0.083` | `-0.081` | `0.010` | `0.686` | `4.837` | `20` | yes |
| `hold062_pitch_ankle_k5_opp` | `0.619` | `0.107` | `-0.075` | `0.005` | `0.613` | `3.757` | `20` | yes |

판단:

- `0.65` pose를 제거하면 force feedback은 안정성을 유지한다.
- 가장 좋은 결과는 `pitch-role ankle differential, k=5`였다.
- 하지만 final left ratio는 `0.619`로, 목표 `0.65`에는 도달하지 못했다.

영상:

```text
outputs/analysis/force_ratio_seq_hold062_target065_pitch_ankle_k5_opp_pos/force_ratio_sequence_render.mp4
```

## Gain/부호 관찰

확인된 경향:

1. `force-sign -1`은 왼발 접촉을 잃는 방향으로 작동했다.
2. `force-sign +1`이 올바른 방향이다.
3. 같은 방향으로 모든 actuator에 넣는 방식은 효과가 작다.
4. 좌우 differential 방식이 더 의미 있다.
5. `kforce=10`은 일부 조건에서 roll 붕괴를 만든다.
6. 안정 조건에서의 하중비 상한은 대략 `0.62` 부근이다.

## 핵심 결론

force-ratio feedback은 유효하지만, 현재 모델/pose family에서는 `left_force_ratio 0.65`까지 안정적으로 밀어 올리지 못한다.

확인된 상한:

```text
stable left_force_ratio ≈ 0.61 ~ 0.62
```

중요한 해석:

- 모터 torque 부족은 여전히 주된 병목으로 보이지 않는다.
- force feedback을 넣어도 `0.65` pose는 자세 붕괴를 만든다.
- 안정 pose를 유지한 채 feedback만 넣으면 시스템은 `0.62` 근처에서 포화된다.
- 따라서 다음 병목은 controller gain이 아니라, 현재 기구학/pose family가 더 큰 left load transfer를 만들지 못하는 점이다.

## 다음 조치

이제 `0.65`를 억지로 controller로 밀기보다, pose family를 바꿔야 한다.

추천 순서:

1. `0.62` 안정 pose에서 pelvis/base 또는 hip roll target을 더 직접적으로 왼발 위로 옮기는 pose search를 만든다.
2. 목적함수에 `left_force_ratio`뿐 아니라 `left contacts >= 8`, `right contacts 유지`, `roll < 0.2`를 강하게 넣는다.
3. `static 0.65` pose는 더 이상 seed로 쓰지 않는다.
4. 새 pose search는 반드시 transition-aware 기준으로 평가한다.

## 산출물

- force-ratio controller script: `/home/king0519/projects/Humanoid/scripts/render_force_ratio_sequence.py`
- 기존 0.65 pose + feedback 실패 영상: `/home/king0519/projects/Humanoid/outputs/analysis/force_ratio_seq_062_065_pitch_ankle_k5_opp_pos/force_ratio_sequence_render.mp4`
- 0.62 pose 유지 + target 0.65 최선 영상: `/home/king0519/projects/Humanoid/outputs/analysis/force_ratio_seq_hold062_target065_pitch_ankle_k5_opp_pos/force_ratio_sequence_render.mp4`
- 0.62 pose 유지 + target 0.65 최선 CSV: `/home/king0519/projects/Humanoid/outputs/analysis/force_ratio_seq_hold062_target065_pitch_ankle_k5_opp_pos/force_ratio_sequence_timeline.csv`
