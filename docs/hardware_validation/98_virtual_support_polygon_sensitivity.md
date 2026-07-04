# 98. Virtual Support Polygon Sensitivity

## 목적

이 단계의 목적은 실제 CAD를 수정하기 전에 MuJoCo 접촉 패드만 가상 변경해서 one-leg support gate가 개선되는지 확인하는 것이다.

이전 단계까지의 병목:

- 오른발 하중은 `0 N`까지 줄일 수 있다.
- 오른발 접촉도 순간적으로 끊을 수 있다.
- 하지만 `right force <= 5 N`, `right contacts = 0`, `roll <= 0.12 rad` 동시 유지 시간이 `0.330~0.360 s` 근처에서 멈춘다.
- 작은 IK lift를 추가해도 gate 시간이 늘지 않았다.

따라서 이번에는 발바닥 support polygon 자체를 가상으로 키워서 설계 형상 민감도를 확인했다.

## 사용 모델

원본:

- `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

기준 pose:

- `configs/weight_shift_left065_contact_constrained_multipoint.json`

## 추가/수정 스크립트

- `scripts/sweep_virtual_support_design_gate.py`

역할:

1. `scripts/build_virtual_design_variant.py`로 접촉 패드 스케일 변경 모델 생성
2. `scripts/audit_com_support_polygon.py`로 COM/support polygon 감사
3. `scripts/search_one_leg_support_gate.py --narrow`로 one-leg support gate 평가
4. 결과를 CSV와 summary JSON으로 저장

## Focused Sweep

실행:

```bash
python3 scripts/sweep_virtual_support_design_gate.py \
  --source-model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/virtual_support_design_gate_sweep_stl_footprint_contact \
  --duration 6.0 --target-gate-time 0.5 --variants focused
```

산출물:

- summary:
  - `outputs/analysis/virtual_support_design_gate_sweep_stl_footprint_contact/virtual_support_design_gate_sweep_summary.json`
- CSV:
  - `outputs/analysis/virtual_support_design_gate_sweep_stl_footprint_contact/virtual_support_design_gate_sweep.csv`

결과:

| Variant | scale_x | scale_y | left area | left margin | gate time | max roll | max torque | 판정 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| baseline | 1.00 | 1.00 | 99.26 cm² | -8.19 mm | 0.360 s | 0.117 rad | 11.82 Nm | 실패 |
| wide_y125 | 1.00 | 1.25 | 121.41 cm² | +6.02 mm | 0.334 s | 0.159 rad | 15.67 Nm | 실패 |
| wide_y150 | 1.00 | 1.50 | 143.57 cm² | +20.24 mm | 0.266 s | 0.168 rad | 19.91 Nm | 실패 |
| long_x125 | 1.25 | 1.00 | 121.52 cm² | -8.19 mm | 0.468 s | 0.131 rad | 12.92 Nm | 실패 |
| long_x150 | 1.50 | 1.00 | 143.78 cm² | -8.19 mm | 0.234 s | 0.108 rad | 14.45 Nm | 실패 |
| both_x125_y125 | 1.25 | 1.25 | 148.57 cm² | +6.02 mm | 0.304 s | 0.147 rad | 16.07 Nm | 실패 |
| both_x150_y150 | 1.50 | 1.50 | 207.68 cm² | +20.24 mm | 0.260 s | 0.168 rad | 15.78 Nm | 실패 |

Best:

- `long_x125`
- 발바닥 contact pad의 local X 방향을 `1.25x` 확장
- gate time `0.468 s`
- 목표 `0.5 s`에 가장 근접

## Best Variant Render

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_force_ratio_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_design_gate_long_x125.xml \
  --poses configs/weight_shift_left065_contact_constrained_multipoint.json \
  --target-left-ratios 0.90 \
  --out-dir outputs/analysis/render_virtual_design_gate_long_x125_best_6s \
  --duration 6.0 --ramp 2.0 --hold 2.0 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 12 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_virtual_design_gate_long_x125_best_6s/force_ratio_sequence_render.mp4`
- timeline:
  - `outputs/analysis/render_virtual_design_gate_long_x125_best_6s/force_ratio_sequence_timeline.csv`

렌더 timeline 기준 비교:

| 항목 | baseline | long_x125 |
|---|---:|---:|
| gate time | 0.330 s | 0.462 s |
| right force <= 5 N | 0.528 s | 0.594 s |
| right contacts = 0 | 0.330 s | 0.462 s |
| right contacts <= 1 | 2.244 s | 3.300 s |
| max roll | 0.117 rad | 0.131 rad |
| max pitch | 0.071 rad | 0.085 rad |
| max torque | 11.82 Nm | 12.92 Nm |
| final right force | 15.79 N | 14.97 N |
| final right contacts | 11 | 4 |

렌더 기준에서도 `long_x125`는 baseline보다 명확히 개선됐다. 다만 엄격 gate 목표 `0.5 s`에는 아직 부족하다.

## X 방향 추가 Scan

`long_x125`가 가장 좋았기 때문에, 발바닥 local X 방향만 추가로 좁혀서 확인했다.

실행:

```bash
python3 scripts/sweep_virtual_support_design_gate.py \
  --source-model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/virtual_support_design_gate_xscan_stl_footprint_contact \
  --duration 6.0 --target-gate-time 0.5 --variants xscan
```

산출물:

- summary:
  - `outputs/analysis/virtual_support_design_gate_xscan_stl_footprint_contact/virtual_support_design_gate_sweep_summary.json`
- CSV:
  - `outputs/analysis/virtual_support_design_gate_xscan_stl_footprint_contact/virtual_support_design_gate_sweep.csv`

결과:

| Variant | scale_x | gate time | low force time | no contact time | max roll | max torque |
|---|---:|---:|---:|---:|---:|---:|
| long_x115 | 1.15 | 0.348 s | 0.380 s | 0.348 s | 0.104 rad | 11.99 Nm |
| long_x120 | 1.20 | 0.362 s | 0.442 s | 0.362 s | 0.109 rad | 12.03 Nm |
| long_x125 | 1.25 | 0.468 s | 0.536 s | 0.468 s | 0.131 rad | 12.92 Nm |
| long_x130 | 1.30 | 0.340 s | 0.450 s | 0.340 s | 0.121 rad | 12.12 Nm |
| long_x135 | 1.35 | 0.260 s | 0.290 s | 0.260 s | 0.096 rad | 12.11 Nm |
| long_x140 | 1.40 | 0.254 s | 0.284 s | 0.254 s | 0.096 rad | 12.43 Nm |

추가 scan에서도 `x=1.25`가 최선이다. 단순히 길이를 계속 늘린다고 좋아지는 구조가 아니다.

## 판단

이번 단계에서 설계 민감도는 확인됐다.

확인된 사실:

- 발바닥 local X 방향, 즉 현재 접촉 패드의 앞뒤 방향 확장은 one-leg support gate를 개선한다.
- `x=1.25`에서 baseline 대비 gate가 `0.360 s -> 0.468 s`로 증가했다.
- 렌더 timeline 기준도 `0.330 s -> 0.462 s`로 증가했다.
- 발바닥 local Y 방향 확장은 COM margin은 개선하지만 gate time은 오히려 감소했다.
- 폭과 길이를 모두 키우는 것도 gate 개선으로 이어지지 않았다.
- `x=1.25`보다 더 길게 키우면 gate가 다시 나빠진다.

따라서 현재 모델은 support polygon 크기 자체에 민감하지만, 단순한 발바닥 확대만으로는 충분하지 않다. 특히 local X 방향으로 약 `25%` 확장했을 때 가장 좋아지는 것은 실제 CAD에서 발 접촉 길이 또는 toe/heel 접촉 위치가 중요한 설계 변수라는 뜻이다.

## 현재 Gate 상태

| 단계 | 결과 |
|---|---|
| standing | 통과 |
| weight shift | 통과 |
| right unload | 통과 |
| transient toe-off | 통과 |
| foot-frame IK pose generation | 통과 |
| unload + small IK lift | 실패 |
| virtual support polygon sensitivity | 개선 확인, gate 미통과 |
| one-leg support 0.5초 | 실패 |

## 설계 해석

현재 결과만 보면 “발바닥을 넓히면 무조건 해결”은 아니다.

더 정확한 해석:

- 전후 방향 접촉 길이 또는 toe/heel 접촉점 배치가 한발 지지 안정성에 영향을 준다.
- 좌우 폭 확장은 정적 COM margin을 늘리지만, 현재 controller/contact 동역학에서는 roll 안정성 또는 접촉 해제 지속시간을 개선하지 못했다.
- 최적점이 `x=1.25` 근처에 존재하므로, 실제 설계에서는 발 전체 길이 확대보다 접촉 패드 위치, toe/heel edge 형상, 발목 pitch/roll 축과 접촉점의 상대 위치를 함께 봐야 한다.

## 다음 조치

다음 단계는 COM 위치 민감도 분석이다.

권장 probe:

1. base_link inertial COM을 가상으로 좌측 지지발 쪽으로 이동한다.
2. 같은 one-leg support gate를 평가한다.
3. COM 이동만으로 gate가 통과되면 상체 배터리/제어보드/프레임 질량 배치가 핵심 설계 변수다.
4. COM 이동과 `long_x125`를 결합했을 때 gate가 통과되는지 확인한다.
5. 두 가상 변경을 결합해야 통과한다면 실제 CAD 수정 후보는 `발 접촉 길이 보강 + 상체 질량 중심 조정`이다.
