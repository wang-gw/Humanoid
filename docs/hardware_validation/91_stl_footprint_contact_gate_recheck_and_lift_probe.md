# 91. STL Footprint Contact 기준 Gate 재검증과 Lift Probe

## 목적

이 단계의 목적은 실제 foot STL footprint에 맞춘 contact 모델을 새 기준 후보로 두고, 기존 gate를 처음부터 다시 검증하는 것이다.

검증 순서:

1. standing 10초
2. `0.65` weight shift transition 12초
3. `0.78` right unload 12초
4. 오른발 lift 5 mm 후보 탐색

## 기준 후보 모델

- 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`
- 의미:
  - 기존 contact pad `44 x 74 mm`를 실제 foot STL footprint에 가까운 `70 x 120 mm`로 확장한 모델
- 주의:
  - 아직 최종 기준 모델로 확정한 것은 아니고, 기준 후보 모델이다.

## 1. Standing 10초

### 산출물

- `outputs/analysis/stl_footprint_candidate_standing_10s/axis_aware_standing_render.mp4`

### 결과

| 항목 | 값 |
|---|---:|
| final roll | 0.0249 rad |
| final pitch | 0.0166 rad |
| final qvel norm | 0.0080 |
| max torque | 5.16 Nm |
| contact frame fraction | 1.0 |

### 판단

통과.

STL footprint contact 모델은 standing에서 안정적이다.

## 2. `0.65` Weight Shift Transition 12초

### 산출물

- `outputs/analysis/stl_footprint_candidate_left065_transition_12s/pose_transition_render.mp4`

### 결과

| 항목 | 값 |
|---|---:|
| final left ratio | 0.733 |
| final right force | 24.17 N |
| final roll | -0.006 rad |
| final pitch | -0.055 rad |
| max torque | 5.16 Nm |
| saturation | 0.0 |

### 판단

통과.

기존 `0.65` pose를 그대로 사용했지만, contact footprint가 실제 STL에 가까워지면서 최종 left ratio가 `0.65`가 아니라 약 `0.73`까지 올라갔다. 즉 이전의 낮은 하중 이동 성능은 contact pad가 작게 잡힌 영향이 컸다.

## 3. `0.78` Right Unload 12초

### 산출물

- `outputs/analysis/stl_footprint_candidate_right_unload078_12s/force_ratio_sequence_render.mp4`

### 결과

| 항목 | 값 |
|---|---:|
| final left ratio | 0.759 |
| final right force | 21.82 N |
| final roll | 0.0219 rad |
| final pitch | -0.0556 rad |
| max roll | 0.126 rad |
| max torque | 11.69 Nm |
| saturation | 0.0 |

### 판단

통과.

이 모델에서는 swing 준비 gate를 안정적으로 통과한다.

기존 작은 contact 모델에서는 `0.70+` unload가 roll 붕괴로 실패했지만, STL footprint contact 모델에서는 right force가 `21.82 N`까지 내려가면서 roll도 안정적으로 유지된다.

## 4. 오른발 5 mm Lift 후보 탐색

### 추가 스크립트

- 탐색:
  - `scripts/search_right_foot_lift_pose.py`
- 렌더:
  - `scripts/render_right_foot_lift_sequence.py`

### 탐색 결과

- pose:
  - `configs/right_foot_lift005_stl_footprint_contact.json`
- 탐색 summary:
  - `outputs/analysis/right_foot_lift005_search_stl_footprint_contact/right_foot_lift_search_summary.json`
- candidates:
  - `outputs/analysis/right_foot_lift005_search_stl_footprint_contact/right_foot_lift_candidates.csv`

Best candidate:

| 항목 | 값 |
|---|---:|
| max right clearance | 9.43 mm |
| final right clearance | -0.34 mm |
| final left ratio | 0.831 |
| final right force | 15.27 N |
| final right contacts | 1 |
| final roll | 0.166 rad |
| max torque | 11.57 Nm |
| saturation | 0.0 |

### 5초 렌더 확인

- 영상:
  - `outputs/analysis/render_right_foot_lift005_stl_footprint_contact_5s_matched/right_foot_lift_render.mp4`

| 항목 | 값 |
|---|---:|
| max right clearance | 9.43 mm |
| final right clearance | -0.34 mm |
| final right force | 15.27 N |
| final right contacts | 1 |
| max roll | 0.166 rad |
| max torque | 11.57 Nm |
| saturation | 0.0 |

### 10초 렌더 확인

- 영상:
  - `outputs/analysis/render_right_foot_lift005_stl_footprint_contact_10s_matched/right_foot_lift_render.mp4`

| 항목 | 값 |
|---|---:|
| max right clearance | 17.14 mm |
| final right clearance | -0.25 mm |
| final roll | 2.65 rad |
| max roll | 3.14 rad |
| final right force | 40.66 N |
| max torque | 19.80 Nm |
| saturation | 0.0 |

### 판단

오른발 5 mm lift hold는 아직 실패다.

다만 완전 실패는 아니다. 5초 조건에서는 오른발 clearance가 순간적으로 `9.43 mm`까지 올라가고, 최종 오른발 force도 `15.27 N`, 오른발 contact도 `1`개까지 줄었다. 즉 발을 거의 들어 올리는 상태까지는 도달했다.

하지만 최종 clearance가 아직 음수이고, 10초 hold에서는 roll 붕괴가 발생한다. 따라서 현재 방식은 “정적인 joint target 하나를 찾아서 오른발을 들고 버티기”에는 부족하다.

## 종합 판단

현재까지의 gate 상태는 다음과 같다.

| 단계 | 결과 |
|---|---|
| standing 10초 | 통과 |
| `0.65` weight shift 12초 | 통과 |
| `0.78` right unload 12초 | 통과 |
| 오른발 5 mm transient lift | 부분 성공 |
| 오른발 5 mm lift hold | 실패 |

중요한 결론:

1. 실제 foot STL footprint contact 모델에서는 swing 준비 단계까지 통과한다.
2. 모터 토크는 여전히 병목으로 보이지 않는다.
3. 오른발 lift 자체는 순간적으로 가능하지만, 정적 pose target 방식으로는 hold 안정성이 부족하다.
4. 다음 단계부터는 단일 pose 탐색보다 foot trajectory 또는 IK 기반 swing trajectory가 필요하다.

## 다음 조치

다음 단계에서는 오른발을 한 번에 들어 올리는 static target이 아니라, 다음과 같은 짧은 swing trajectory를 만든다.

1. unload pose 유지
2. 오른발 knee/ankle를 서서히 접어 clearance를 만든다.
3. lift 중 roll이 커지면 즉시 중단하는 trajectory gate를 둔다.
4. 목표 clearance를 `2 mm -> 5 mm -> 10 mm` 순서로 올린다.

추천 gate:

- `2 mm` lift hold 5초 통과를 먼저 목표로 둔다.
- 그 다음 `5 mm` lift hold를 재시도한다.

