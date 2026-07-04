# Inertia Variant Probe Comparison

## 목적

51번에서 만든 실험용 full-inertia 모델이 기존 mass-only 모델보다 동역학적으로 나아지는지 확인한다. 이 단계는 최종 inertial 확정이 아니라, 제공된 inertia 데이터가 시뮬레이션 안정성에 의미 있는 영향을 주는지 보는 sanity check다.

## 비교 모델

| 구분 | 모델 |
| --- | --- |
| baseline | `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml` |
| inertia experimental | `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml` |

실험 모델 적용 조건:

- `base_link` inertial은 기존 값 유지
- 다리 하위 10개 body에 component aggregate `fullinertia` 적용
- foot contact box와 전체 질량은 기존 user-size/mass 모델과 동일
- initial `base_z=0.005 m`

## 실행 명령

```bash
python3 scripts/analyze_standing_geometry.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml \
  --doc-dir docs/hardware_validation/user_size_mass_inertia_direct_nobase_geometry

python3 scripts/probe_pd_standing.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml \
  --base-z 0.005 \
  --out-dir outputs/analysis/pd_standing_user_size_mass_inertia_direct_nobase_exact005 \
  --doc-dir docs/hardware_validation/tmp \
  --doc-name inertia_direct_nobase_exact005.md

python3 scripts/plot_pd_standing.py \
  --csv outputs/analysis/pd_standing_user_size_mass_inertia_direct_nobase_exact005/neutral_pd_standing.csv \
  --out-dir outputs/analysis/pd_standing_user_size_mass_inertia_direct_nobase_exact005
```

## Geometry 결과

| 항목 | 값 |
| --- | ---: |
| total mass | `9.211400 kg` |
| initial base z | `0.005000 m` |
| COM world | `[0.053076, -0.047922, 0.280897]` |
| COM inside support x | `True` |
| COM inside support y | `True` |

baseline COM은 `[0.053358, -0.073768, 0.168206]`였고, inertia experimental 모델은 COM z가 더 높게 계산된다. 이는 다리 하위 body의 inertial COM이 바뀌었기 때문이다.

## Neutral PD Standing 비교

| 모델 | final base z | final roll | final pitch | max qvel norm | max contact force |
| --- | ---: | ---: | ---: | ---: | ---: |
| baseline | 0.131486 | -2.453577 | 0.150364 | 857.401243 | 4711.977344 |
| inertia experimental | 0.026609 | 2.588957 | 1.212512 | 266.242408 | 1303.468953 |

## Ankle RMS Torque 비교

| actuator | baseline | inertia experimental |
| --- | ---: | ---: |
| `motor_left_ankle_pitch` | 95.848049 | 84.464979 |
| `motor_left_ankle_roll` | 97.083598 | 84.870726 |
| `motor_right_ankle_pitch` | 93.344675 | 50.794615 |
| `motor_right_ankle_roll` | 93.298375 | 86.609677 |

## 산출물

- 실험 모델: `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml`
- geometry JSON: `docs/hardware_validation/user_size_mass_inertia_direct_nobase_geometry/standing_geometry_analysis.json`
- PD summary: `outputs/analysis/pd_standing_user_size_mass_inertia_direct_nobase_exact005/neutral_pd_standing_summary.json`
- PD plot: `outputs/analysis/pd_standing_user_size_mass_inertia_direct_nobase_exact005/neutral_pd_standing_plot.png`

## 판단

실험 inertia 모델은 neutral PD standing에 여전히 실패한다. roll/pitch가 크게 무너지는 점에서, 이 모델을 곧바로 “걸을 수 있는 하드웨어”로 판단할 수는 없다.

하지만 baseline 대비 다음 수치가 크게 줄었다.

- max qvel norm: `857.401243 -> 266.242408`
- max contact force: `4711.977344 -> 1303.468953`
- ankle RMS torque도 전반적으로 감소

따라서 제공된 inertia 데이터는 시뮬레이션 안정성과 토크 추정에 실제 영향을 준다. 방향성은 의미가 있지만, frame 가정이 아직 완전히 확정되지 않았으므로 최종 설계 검증 모델로 승격하기 전 추가 검증이 필요하다.

## 다음 단계

다음 단계는 두 갈래다.

1. `base_link`와 hip-roll actuator 계열의 정확한 body-frame transform을 복원해 base inertial까지 적용한다.
2. 현재 실험 inertia 모델에서 quasi-static standing pose 또는 COM stabilizer를 추가해, 단순 joint-space PD 실패와 하드웨어 형상 문제를 분리한다.

