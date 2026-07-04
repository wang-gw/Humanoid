# Exact Base Actuator Sweep

> 추가 주의: 이후 `56_stabilizer_actuator_limit_recheck.md`에서 확인했듯이, 기존 `apply_actuator_limit.py`는 motor range만 바꾸고 joint `actuatorfrcrange`는 바꾸지 않았다. 따라서 이 문서의 200/300 Nm 결과는 실제 joint clamp가 100 Nm로 남았을 가능성을 고려해 해석해야 한다. 수정된 actuator limit 검증은 56번 문서를 기준으로 한다.

## 목적

46번 actuator sweep은 `base_z=0.057284 m` 조건을 포함하고 있었는데, 이후 확인 결과 이 값은 foot contact box의 실제 하단 기준이 아니라 `geom_rbound`에 의해 과대 추정된 값이었다.

이번 문서는 정확한 foot contact 기준인 `base_z=0.005 m`에서 actuator limit `100/200/300 Nm` 결과를 다시 정리한다.

## 사용 모델

| torque limit | 모델 |
| ---: | --- |
| 100 Nm | `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml` |
| 200 Nm | `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact_act200.xml` |
| 300 Nm | `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact_act300.xml` |

초기 조건:

- standing pose: 모든 revolute joint `0 rad`
- foot contact box halfsize: `[0.035, 0.060, 0.020] m`
- initial `base_z`: `0.005 m`
- controller: joint-space PD, `Kp=60`, `Kd=4`

## 결과 비교

| actuator limit | final base z | final roll | final pitch | max qvel norm | max contact force |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 100 Nm | 0.131486 | -2.453577 | 0.150364 | 857.401243 | 4711.977344 |
| 200 Nm | 0.131486 | -2.453577 | 0.150364 | 857.401243 | 4711.977344 |
| 300 Nm | 0.131486 | -2.453577 | 0.150364 | 857.401243 | 4711.977344 |

## Ankle RMS Torque

| actuator | 100 Nm | 200 Nm | 300 Nm |
| --- | ---: | ---: | ---: |
| `motor_left_ankle_pitch` | 95.848049 | 187.133869 | 272.088093 |
| `motor_left_ankle_roll` | 97.083598 | 192.310071 | 283.797641 |
| `motor_right_ankle_pitch` | 93.344675 | 179.544500 | 259.829121 |
| `motor_right_ankle_roll` | 93.298375 | 182.430134 | 269.246555 |

## 산출물

- 100 Nm summary: `outputs/analysis/pd_standing_user_size_mass_contact_zero_basez_exact005/neutral_pd_standing_summary.json`
- 100 Nm plot: `outputs/analysis/pd_standing_user_size_mass_contact_zero_basez_exact005/neutral_pd_standing_plot.png`
- 200 Nm summary: `outputs/analysis/pd_standing_user_size_mass_contact_act200_exact005/neutral_pd_standing_summary.json`
- 200 Nm plot: `outputs/analysis/pd_standing_user_size_mass_contact_act200_exact005/neutral_pd_standing_plot.png`
- 300 Nm summary: `outputs/analysis/pd_standing_user_size_mass_contact_act300_exact005/neutral_pd_standing_summary.json`
- 300 Nm plot: `outputs/analysis/pd_standing_user_size_mass_contact_act300_exact005/neutral_pd_standing_plot.png`

## 판단

정확한 foot contact 높이에서도 actuator limit을 키우면 ankle torque 사용량은 증가한다. 하지만 base 자세, 속도, contact force 결과는 개선되지 않았다.

따라서 현재 standing 실패를 “100 Nm 모터가 약해서 생긴 문제”로 해석하면 안 된다. 모터 최종 선정은 아직 이르다.

현재 우선순위는 다음과 같다.

1. neutral `0 rad` pose가 실제 균형 자세인지 준정적 IK/COM 기준으로 재검증
2. floating-base standing에 맞는 base/COM stabilizer 또는 quasi-static controller 추가
3. CAD component-local CG를 MJCF body frame으로 정확히 변환해 mass/inertia 재구성
4. 그 다음 squat/weight-shift/one-leg support에서 토크 envelope를 확인

이 단계를 통과한 뒤에야 actuator torque/RPM/감속비 후보를 설계 수치로 좁히는 것이 타당하다.
