# 90. Foot STL vs Contact Footprint 감사

## 목적

이 단계의 목적은 이전 sweep에서 확인한 최소 통과 발바닥 폭이 실제 CAD/STL 발바닥 치수와 어떤 관계인지 확인하는 것이다.

이전 단계 결론:

- 현재 contact pad 유효 폭: 약 `74 mm`
- 오른발 unload gate 최소 통과 scale: `1.75`
- 최소 통과 contact 폭: 약 `129.5 mm`

따라서 이번에는 실제 `foot_L.stl`, `foot_R.stl`의 bounding box와 현재 MuJoCo contact pad bounding box를 비교했다.

## 사용 파일과 스크립트

- STL:
  - `link/foot_L.stl`
  - `link/foot_R.stl`
- 기준 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics_multipoint_fullheight_contact.xml`
- 감사 스크립트:
  - `scripts/audit_foot_stl_vs_contact_width.py`
- 산출물:
  - `outputs/analysis/foot_stl_vs_contact_baseline/foot_stl_vs_contact_width.json`
  - `outputs/analysis/foot_stl_vs_contact_virtual_y1p75/foot_stl_vs_contact_width.json`

## 실제 Foot STL 치수

`foot_L.stl`과 `foot_R.stl`은 좌우 대칭이며 bounding box 크기는 동일하다.

| 항목 | X | Y | Z |
|---|---:|---:|---:|
| foot STL size | 70.0 mm | 120.0 mm | 63.7 mm |

즉 STL 기준 실제 발 형상은 약 `70 x 120 mm` 평면 footprint를 가진다.

## 현재 MuJoCo Contact 치수

현재 기준 모델의 foot contact pad 전체 bounds는 다음과 같다.

| 항목 | X | Y | Z |
|---|---:|---:|---:|
| current contact size | 44.0 mm | 74.0 mm | 48.5 mm |
| STL 대비 비율 | 62.9% | 61.7% | 76.1% |

즉 현재 contact 모델은 실제 foot STL보다 작다.

특히 평면 접촉 치수만 보면:

- 실제 STL: `70 x 120 mm`
- 현재 contact: `44 x 74 mm`

따라서 이전까지의 실패는 실제 발 형상 자체가 부족해서라기보다, MuJoCo contact pad가 실제 발바닥 footprint를 과소 모델링했을 가능성이 크다.

## 최소 통과 Contact와 STL 비교

이전 sweep의 최소 통과 scale `1.75`는 contact Y 치수를 다음과 같이 만든다.

| 항목 | X | Y | Z |
|---|---:|---:|---:|
| scale 1.75 contact size | 44.0 mm | 129.5 mm | 48.5 mm |
| STL 대비 비율 | 62.9% | 107.9% | 76.1% |

이 모델은 Y 방향만 보면 실제 STL의 `120 mm`와 거의 같은 수준이다. 다만 X 방향은 여전히 `44 mm`라서 실제 STL의 `70 mm`보다 작다.

## STL Footprint Contact 후보

실제 STL footprint에 맞춘 contact 후보를 두 개 만들었다.

### 1. Y만 STL 120 mm에 맞춘 모델

- 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_y120_contact.xml`
- 변경:
  - contact Y size: `74 mm -> 120 mm`
  - contact X size: `44 mm` 유지
- 보고서:
  - `outputs/analysis/virtual_design_variants/stl_y120_contact_report.json`

### 2. X/Y 모두 STL 70 x 120 mm에 맞춘 모델

- 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`
- 변경:
  - contact X size: `44 mm -> 70 mm`
  - contact Y size: `74 mm -> 120 mm`
- 보고서:
  - `outputs/analysis/virtual_design_variants/stl_footprint_contact_report.json`

## Support Polygon 결과

`configs/weight_shift_left065_contact_constrained_multipoint.json` 자세에서 계산했다.

| 모델 | 왼발 단독 margin | 양발 margin |
|---|---:|---:|
| 기준 contact | -30.0 mm | -7.1 mm |
| STL Y 120 contact | -8.2 mm | +11.3 mm |
| STL X/Y footprint contact | -8.2 mm | +18.1 mm |

STL footprint contact는 양발 전체 support margin을 크게 개선한다. 왼발 단독 margin은 아직 약 `-8.2 mm`로 완전히 내부는 아니지만, 동역학 gate에는 충분한 개선을 제공했다.

## 오른발 Unload Gate 결과

조건:

- target left ratio: `0.78`
- force feedback:
  - `force_role = both`
  - `force_on = all`
  - `force_side_mode = same`
  - `kforce = 5`
- controller:
  - Kp `20`
  - Kd `12`
  - torque limit `30 Nm`

| 모델 | final left ratio | final right force | max roll | max torque | saturation | 판단 |
|---|---:|---:|---:|---:|---:|---|
| STL Y 120 contact | 0.730 | 24.41 N | 0.139 rad | 11.13 Nm | 0.0 | 통과 |
| STL X/Y footprint contact | 0.753 | 22.29 N | 0.126 rad | 11.69 Nm | 0.0 | 통과 |

산출물:

- `outputs/analysis/force_ratio_virtual_stl_y120_hold065_target078_both_all_k5_pos/force_ratio_sequence_render.mp4`
- `outputs/analysis/force_ratio_virtual_stl_footprint_hold065_target078_both_all_k5_pos/force_ratio_sequence_render.mp4`

## 판단

현재까지의 가장 중요한 결론은 다음이다.

현재 MuJoCo contact pad가 실제 foot STL footprint보다 작게 들어가 있었다.

기준 contact는 약 `44 x 74 mm`이고, 실제 foot STL은 약 `70 x 120 mm`다. 실제 STL footprint에 맞춘 contact 모델에서는 오른발 unload gate를 통과했다. 따라서 이전의 오른발 unload 실패를 “실제 발 설계가 부족하다”고 바로 판단하면 안 된다.

더 정확한 판단은 다음이다.

1. 실제 발 STL 자체는 `70 x 120 mm` footprint를 가진다.
2. 기존 MuJoCo contact 모델은 이를 `44 x 74 mm`로 과소 표현했다.
3. contact를 STL footprint에 맞추면 오른발 unload gate가 통과한다.
4. 따라서 현재 단계에서는 CAD 발 형상 수정 전에 contact collision 모델을 실제 발바닥 치수에 맞게 수정하는 것이 우선이다.

## 하드웨어 설계 관점 결론

이번 결과만 보면 발 크기를 당장 키워야 한다고 결론내리기는 어렵다.

오히려 현재 실제 발 형상 `70 x 120 mm`는 단순 unload gate를 통과할 가능성이 있다. 문제는 시뮬레이션에서 그 발 형상을 contact collision으로 충분히 반영하지 않았다는 점이다.

즉 다음 작업 우선순위는 다음과 같다.

1. MuJoCo contact pad를 실제 foot STL footprint 기준으로 수정
2. 수정 모델을 기준 모델 후보로 승격
3. standing, `0.65` weight shift, `0.78` unload를 다시 검증
4. 그 다음 swing foot lift를 시도

## 다음 조치

다음 단계에서는 `URDF_F_link_virtual_stl_footprint_contact.xml`을 임시 실험 모델이 아니라 새로운 contact 기준 모델 후보로 정리한다.

검증 순서:

1. standing transition 10초
2. `0.65` weight shift transition 8초/12초
3. `0.78` right unload 8초/12초
4. 성공 시 오른발 lift 높이 `5 mm`, `10 mm` 순서로 probe

