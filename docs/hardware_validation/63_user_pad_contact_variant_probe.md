# 사용자 제공 발바닥 Contact Pad 변형 검증

## 목적

사용자가 제공한 `foot local frame` 기준 contact pad 좌표를 MuJoCo 모델에 반영하고, 기존에 확인한 CAD-MuJoCo 축 대응과 충돌하는 부분이 있는지 확인한다. 목표는 RL 학습으로 넘어가기 전에 발바닥 support polygon이 물리적으로 타당한지, 그리고 준정적 standing에서 여전히 무너지는 원인이 contact 입력인지 다른 동역학/형상 문제인지 분리하는 것이다.

## 사용자 입력

발바닥 contact pad 입력값:

| foot | local center mm | size mm | halfsize mm |
| --- | --- | --- | --- |
| `foot_L:1` | `(0.000, -35.750, 20.000)` | `(70.000, 48.500, 40.000)` | `(35.000, 24.250, 20.000)` |
| `foot_R:1` | `(0.000, -35.750, -20.000)` | `(70.000, 48.500, 40.000)` | `(35.000, 24.250, 20.000)` |

사용자가 함께 준 설명:

- `foot_L:1` occurrence transform: world origin `(76.750, -60.000, 25.000) mm`
- rotation: identity, 즉 local 방향과 world 방향이 같다고 설명됨
- 모서리 표에서는 local `Z` 변화가 `Toe/Heel` 방향으로 라벨링됨

이전 단계에서 확인한 전체 축 대응:

| 의미 | 대응 |
| --- | --- |
| MuJoCo X | CAD Y |
| MuJoCo Y | CAD X |
| MuJoCo Z | CAD Z |
| 지면 높이 | CAD Z=0, MuJoCo Z=0 |
| 전방 | CAD +Y, MuJoCo +X |
| 좌측 | CAD +X, MuJoCo +Y |
| 위쪽 | CAD +Z, MuJoCo +Z |

## 핵심 불일치

현재 입력에는 중요한 모순이 있다.

1. `rotation: identity`와 기존 축 대응을 따르면, 사용자 local `Z`는 CAD Z이고 MuJoCo Z, 즉 상하 방향이어야 한다.
2. 그런데 모서리 라벨에서는 local `Z`의 `0 -> 40 mm` 변화가 `Toe -> Heel` 방향으로 설명된다.
3. Toe/Heel은 전후 방향이므로 기존 축 대응에서는 CAD Y, MuJoCo X에 해당해야 한다.

따라서 이번 단계에서는 하나의 contact 모델로 확정하지 않고, 가능한 해석을 나눠 진단 모델을 만들었다.

## 생성한 모델

생성 스크립트:

```bash
python3 scripts/build_user_pad_contact_variant.py
```

| 모델 | 해석 | 판단 |
| --- | --- | --- |
| `URDF_F_link_user_size_mass_inertia_user_pad_axis_raw.xml` | 기존 축 대응 적용, 입력 z값 그대로 사용 | 왼발 contact bottom이 지면보다 40 mm 위라 제외 |
| `URDF_F_link_user_size_mass_inertia_user_pad_axis_grounded.xml` | 기존 축 대응 적용, z center를 `-half_z`로 접지 보정 | 주 후보 A |
| `URDF_F_link_user_size_mass_inertia_user_pad_mjcf_raw.xml` | 입력 local XYZ를 현재 MJCF foot frame으로 직접 해석 | 왼발 contact bottom이 지면보다 40 mm 위라 제외 |
| `URDF_F_link_user_size_mass_inertia_user_pad_mjcf_grounded.xml` | MJCF local 직접 해석, z center를 `-half_z`로 접지 보정 | 진단 후보 B |

raw 모델 2개는 왼발 contact pad가 지면에서 떠 있으므로 standing/RL 후보에서 제외한다.

## Standing Geometry 비교

### 후보 A: 기존 축 대응을 따른 grounded 모델

모델:

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_axis_grounded.xml
```

| 항목 | 값 |
| --- | ---: |
| total mass | `9.2114 kg` |
| base_z | `0.0050 m` |
| COM world | `(0.053076, -0.047922, 0.280897) m` |
| support x range | `[-0.1470, 0.0555] m` |
| support y range | `[-0.0565, 0.0135] m` |
| COM inside support x/y | `true / true` |
| x margin min/max | `0.200076 / 0.002424 m` |
| y margin min/max | `0.008578 / 0.061422 m` |

판단:

- 기존 CAD-MuJoCo 축 대응을 가장 충실히 따르는 모델이다.
- 하지만 COM이 support polygon의 +X 경계에서 약 `2.4 mm`밖에 떨어져 있지 않다.
- 즉, 정지 상태만 보더라도 전후 방향 안정 여유가 매우 작다.
- RL이 균형을 잡더라도 초기 학습 난이도가 매우 높고, 실제 하드웨어에서는 제작 오차/바닥 마찰/제어 지연만으로도 쉽게 넘어질 수 있다.

### 후보 B: 입력 local XYZ를 MJCF foot local로 직접 해석한 grounded 모델

모델:

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_mjcf_grounded.xml
```

| 항목 | 값 |
| --- | ---: |
| total mass | `9.2114 kg` |
| base_z | `0.0050 m` |
| COM world | `(0.053076, -0.047922, 0.280897) m` |
| support x range | `[-0.1220, 0.1020] m` |
| support y range | `[-0.0815, -0.0330] m` |
| COM inside support x/y | `true / true` |
| x margin min/max | `0.175076 / 0.048924 m` |
| y margin min/max | `0.033578 / 0.014922 m` |

판단:

- support polygon 여유는 후보 A보다 좋아 보인다.
- 그러나 이 해석은 기존에 확인한 `MuJoCo X = CAD Y`, `MuJoCo Y = CAD X`, `MuJoCo Z = CAD Z` 대응과 맞지 않는다.
- 따라서 현재로서는 최종 모델이 아니라, 사용자가 제공한 local 좌표표의 라벨이 실제로 MJCF foot frame에 가까운지 확인하기 위한 진단 모델로 봐야 한다.

## PD Standing Probe

실행 조건:

```bash
python3 scripts/probe_pd_standing.py \
  --pose-json configs/quasistatic_standing_pose_inertia_direct_nobase.json \
  --duration 2.0 \
  --kp 60 \
  --kd 4 \
  --torque-limit 100
```

| 모델 | final base z | final roll | final pitch | max qvel | max contact force | 판단 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| axis grounded | `0.071909 m` | `1.998482 rad` | `0.504845 rad` | `173.578` | `1966.602 N` | 실패 |
| mjcf grounded | `0.004833 m` | `2.226741 rad` | `-0.734174 rad` | `169.540` | `1926.537 N` | 실패 |

토크 포화:

| 모델 | 주 포화 actuator |
| --- | --- |
| axis grounded | 양쪽 ankle pitch, 양쪽 ankle roll이 `100 Nm` 한계에 도달 |
| mjcf grounded | 양쪽 ankle pitch, 양쪽 ankle roll이 `100 Nm` 한계에 도달 |

판단:

- contact pad 입력을 보완해도 현재 quasi-static pose + PD 제어 조건에서는 standing이 유지되지 않는다.
- 실패 원인은 단순히 “발바닥 contact가 점/edge라서”만은 아니다.
- ankle pitch/roll 쪽에 과도한 보상 토크가 걸리고, base roll/pitch가 크게 발산한다.

## Roll Impulse 비교

실행 조건:

```bash
python3 scripts/audit_roll_impulse_response.py \
  --pose-json configs/quasistatic_standing_pose_inertia_direct_nobase.json \
  --base-z-offset -0.006 \
  --duration 0.25 \
  --impulse-start 0.01 \
  --impulse-duration 0.04 \
  --impulse-torque 25 \
  --joint-kp 60 \
  --joint-kd 4 \
  --torque-limit 100
```

### No impulse baseline

| 모델 | final roll | final pitch | max qvel | max contact force | final L/R contact |
| --- | ---: | ---: | ---: | ---: | --- |
| axis grounded | `0.026085 rad` | `0.171457 rad` | `208.279` | `2451.522 N` | `0.000 / 0.000 N` |
| mjcf grounded | `0.101963 rad` | `0.036132 rad` | `244.738` | `2585.699 N` | `0.000 / 177.461 N` |

### 25 Nm roll impulse

| 모델 | 관찰 |
| --- | --- |
| axis grounded | hip roll impulse는 roll 변화를 일부 양/음 방향으로 만들지만, ankle roll impulse는 큰 roll 발산을 유발한다. 왼쪽 ankle roll `+25 Nm`에서 final roll `0.318376 rad`, 오른쪽 ankle roll `+25 Nm`에서 final roll `0.172228 rad`까지 증가한다. |
| mjcf grounded | 모든 roll impulse case가 대체로 같은 양의 roll 방향으로 끝난다. final roll 범위는 약 `0.090311 ~ 0.185217 rad`이다. 이는 impulse sign보다 contact/자세 붕괴 동역학이 지배적이라는 뜻이다. |

판단:

- 후보 A는 축 대응은 맞지만 support 여유가 너무 작고, ankle roll 입력에 매우 민감하다.
- 후보 B는 support 여유는 낫지만 축 해석이 불확실하며, impulse sign 관측이 깨끗하지 않다.
- 두 모델 모두 “현재 하드웨어 모델 + 현재 standing pose + 단순 PD” 조건에서는 RL로 바로 걷기 학습을 시작하기에 부족하다.

## 현재 결론

이번 사용자 contact pad 정보는 충분히 도움이 된다. 특히 이전 edge/contact ambiguity보다 훨씬 구체적인 발바닥 support 영역을 만들 수 있었다.

후속 단계에서 사용자가 `foot local Z = Toe/Heel 방향`으로 확정했다. 따라서 이 문서의 후보 비교는 축 확정 전 진단 기록으로 남기고, 실제 다음 검증 기준 모델은 64번 문서의 `toeheel_z_grounded` 모델로 한다.

```text
docs/hardware_validation/64_toeheel_z_contact_axis_confirmation.md
```

## 하드웨어 설계 관점 판단

현재 후보 A가 실제 축 대응에 더 가까운 모델이라면, 전후 support margin `2.4 mm`는 너무 작다. 이 경우 설계 수정 후보는 다음 순서가 합리적이다.

1. 발바닥 contact pad를 전방/후방, 특히 MuJoCo +X 또는 CAD +Y 방향으로 넓힌다.
2. standing pose에서 COM을 support polygon 중앙 쪽으로 이동시키는 hip/knee/ankle 중립각을 찾는다.
3. ankle roll/pitch actuator의 실제 torque-speed limit, gear ratio, rotor inertia를 더 정확히 넣는다.
4. contact pad 확정 후에야 RL standing/balance 학습으로 넘어간다.

후보 B가 실제 foot local frame에 더 가깝다면 support margin은 후보 A보다 낫지만, 여전히 ankle 토크 포화와 roll 붕괴가 남아 있다. 이 경우 contact geometry만으로 해결되지 않으므로, ankle roll 축 위치/방향, 발목 감속기 질량 배치, base COM, quasi-static pose를 함께 재검증해야 한다.

## 산출물

- contact pad 생성 스크립트: `scripts/build_user_pad_contact_variant.py`
- 후보 A 모델: `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_axis_grounded.xml`
- 후보 B 모델: `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_mjcf_grounded.xml`
- 후보 A geometry: `docs/hardware_validation/user_pad_axis_grounded_geometry/standing_geometry_analysis.json`
- 후보 B geometry: `docs/hardware_validation/user_pad_mjcf_grounded_geometry/standing_geometry_analysis.json`
- 후보 A PD 결과: `outputs/analysis/pd_standing_user_pad_axis_grounded_quasistatic_pose/neutral_pd_standing_summary.json`
- 후보 B PD 결과: `outputs/analysis/pd_standing_user_pad_mjcf_grounded_quasistatic_pose/neutral_pd_standing_summary.json`
- 후보 A roll impulse: `outputs/analysis/roll_impulse_response_user_pad_axis_grounded/roll_impulse_response_summary.json`
- 후보 B roll impulse: `outputs/analysis/roll_impulse_response_user_pad_mjcf_grounded/roll_impulse_response_summary.json`
- 축 확정 후속 문서: `docs/hardware_validation/64_toeheel_z_contact_axis_confirmation.md`
