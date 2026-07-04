# Foot Local Z Toe/Heel 확정 Contact 모델

## 목적

63번 문서에서 남아 있던 `foot local frame` 축 불확실성을 하나로 확정한다. 사용자가 `Toe/Heel 방향 = foot local Z`로 확정했으므로, 이번 단계부터 발바닥 contact pad는 다음 축 정의를 기준으로 만든다.

## 확정한 foot local 축

| foot local 축 | 의미 | 현재 MJCF contact box에 반영한 축 |
| --- | --- | --- |
| local X | 좌우 폭 방향 | MuJoCo local Y |
| local Y | 발바닥 두께/수직 방향 | MuJoCo local Z |
| local Z | Toe/Heel 전후 방향 | MuJoCo local X |

주의할 점:

- 현재 MJCF의 `foot_L_1`, `foot_R_v1_1` body는 local X/Y/Z가 world X/Y/Z와 동일하게 놓여 있다.
- 따라서 사용자가 말한 CAD/Fusion의 `foot local frame`은 현재 MJCF foot body frame과 동일하지 않다고 해석한다.
- 전역 좌표계의 `MuJoCo Z = 위쪽`은 그대로 유지한다. 바꾼 것은 발바닥 contact pad 입력 좌표를 MJCF contact box로 옮기는 해석이다.

## 생성한 모델

생성 명령:

```bash
python3 scripts/build_user_pad_contact_variant.py \
  --mode toeheel_z_grounded \
  --out envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml
```

생성 모델:

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml
```

변환 결과:

| foot | MJCF geom pos | MJCF geom halfsize |
| --- | --- | --- |
| left | `(0.02000, 0.00000, -0.02425)` | `(0.02000, 0.03500, 0.02425)` |
| right | `(-0.02000, 0.00000, -0.02425)` | `(0.02000, 0.03500, 0.02425)` |

즉 contact pad는 다음 의미를 가진다.

- 전후 Toe/Heel 길이: `40 mm`
- 좌우 폭: `70 mm`
- 수직 두께: `48.5 mm`

## Standing Geometry 결과

실행 명령:

```bash
python3 scripts/analyze_standing_geometry.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml \
  --doc-dir docs/hardware_validation/user_pad_toeheel_z_grounded_geometry
```

| 항목 | 값 |
| --- | ---: |
| total mass | `9.2114 kg` |
| base_z | `0.0135 m` |
| COM world | `(0.053076, -0.047922, 0.289397) m` |
| support x range | `[-0.1270, 0.1070] m` |
| support y range | `[-0.0565, 0.0135] m` |
| COM inside support x/y | `true / true` |
| x margin min/max | `0.180076 / 0.053924 m` |
| y margin min/max | `0.008578 / 0.061422 m` |

판단:

- 63번의 `axis_grounded` 모델보다 전후 support margin이 크게 개선되었다.
- COM은 support AABB 내부에 들어온다.
- 하지만 좌우 방향 한쪽 margin은 여전히 약 `8.6 mm`로 작다.

## PD Standing Probe

실행 조건:

```bash
python3 scripts/probe_pd_standing.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml \
  --pose-json configs/quasistatic_standing_pose_inertia_direct_nobase.json \
  --out-dir outputs/analysis/pd_standing_user_pad_toeheel_z_grounded_quasistatic_pose \
  --duration 2.0 \
  --kp 60 \
  --kd 4 \
  --torque-limit 100
```

| 항목 | 값 |
| --- | ---: |
| final base z | `0.030716 m` |
| final roll | `2.815632 rad` |
| final pitch | `-0.230678 rad` |
| max qvel norm | `187.135240` |
| max contact normal force | `2115.798913 N` |

토크:

| actuator | max abs torque |
| --- | ---: |
| motor_left_ankle_pitch | `100.0 Nm` |
| motor_left_ankle_roll | `100.0 Nm` |
| motor_right_ankle_pitch | `100.0 Nm` |
| motor_right_ankle_roll | `100.0 Nm` |

판단:

- Z=Toe/Heel contact 모델에서도 2초 PD standing은 실패한다.
- 실패 양상은 여전히 ankle pitch/roll 토크 포화와 base roll 발산이다.
- 따라서 contact pad 축을 확정한 뒤에도, 현재 하드웨어 모델이 “그대로 RL만 하면 걷는다”고 판단할 수는 없다.

## Roll Impulse 결과

실행 조건:

```bash
python3 scripts/audit_roll_impulse_response.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml \
  --pose-json configs/quasistatic_standing_pose_inertia_direct_nobase.json \
  --base-z-offset -0.006 \
  --duration 0.25 \
  --impulse-start 0.01 \
  --impulse-duration 0.04 \
  --joint-kp 60 \
  --joint-kd 4 \
  --torque-limit 100
```

No impulse baseline:

| final roll | final pitch | max qvel | max contact force | final L/R contact |
| ---: | ---: | ---: | ---: | --- |
| `0.047285 rad` | `0.221601 rad` | `202.591` | `1817.999 N` | `0.000 / 0.000 N` |

25 Nm impulse:

| case | final roll |
| --- | ---: |
| left hip roll + | `0.052876 rad` |
| left hip roll - | `0.042063 rad` |
| left ankle roll + | `0.047285 rad` |
| left ankle roll - | `0.052143 rad` |
| right hip roll + | `0.052835 rad` |
| right hip roll - | `0.043260 rad` |
| right ankle roll + | `0.062269 rad` |
| right ankle roll - | `0.048150 rad` |

판단:

- 0.25초 단기 roll impulse에서는 이전 일부 후보보다 final roll 크기가 작다.
- 하지만 마지막 접촉력이 `0 / 0 N`으로 끝나는 케이스가 많아, 안정적인 양발 지지 상태라고 볼 수 없다.
- 단기 impulse 결과만으로 RL 가능성을 통과시키면 안 된다. 2초 standing 실패를 더 우선해서 봐야 한다.

## 현재 결론

이번 단계부터 contact pad 해석은 다음 모델을 기준으로 진행한다.

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml
```

다만 결과는 “축 확정”이지 “하드웨어 검증 통과”가 아니다. 현재 모델은 support polygon 내부에 COM이 들어오지만, ankle actuator가 토크 한계에 닿고 base roll이 발산한다.

다음 검증은 contact 모델을 더 바꾸기보다, 이 확정 contact 모델을 기준으로 다음 항목을 확인하는 것이 맞다.

1. quasi-static standing pose를 다시 탐색해서 COM을 support polygon 중앙으로 이동시킨다.
2. ankle roll/pitch joint axis와 actuator 위치가 실제 CAD 조립과 일치하는지 재확인한다.
3. ankle actuator/감속기 모델에 rotor inertia, gear ratio, torque-speed limit을 넣을 준비를 한다.
4. 위 항목 후에도 ankle torque saturation이 계속되면, 발목 구조/발바닥 폭/질량 배치 수정 후보를 만든다.

## 산출물

- 생성 스크립트: `scripts/build_user_pad_contact_variant.py`
- 확정 contact 모델: `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml`
- 생성 리포트: `docs/hardware_validation/user_pad_contact_variant_toeheel_z_grounded_report.json`
- geometry 결과: `docs/hardware_validation/user_pad_toeheel_z_grounded_geometry/standing_geometry_analysis.json`
- PD standing 결과: `outputs/analysis/pd_standing_user_pad_toeheel_z_grounded_quasistatic_pose/neutral_pd_standing_summary.json`
- roll impulse no-impulse 결과: `outputs/analysis/roll_impulse_response_user_pad_toeheel_z_grounded_no_impulse/roll_impulse_response_summary.json`
- roll impulse 25 Nm 결과: `outputs/analysis/roll_impulse_response_user_pad_toeheel_z_grounded/roll_impulse_response_summary.json`

