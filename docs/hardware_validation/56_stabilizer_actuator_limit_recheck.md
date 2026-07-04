# Stabilizer Actuator Limit Recheck

## 목적

55번 stabilizer sweep에서 best config도 torque saturation fraction이 `0.995025`로 거의 계속 포화되었다. 따라서 actuator limit을 올렸을 때 stabilizer가 실제로 standing을 개선하는지 다시 확인한다.

이 과정에서 `scripts/apply_actuator_limit.py`의 중요한 문제도 수정했다.

## Actuator Limit 스크립트 수정

기존 `apply_actuator_limit.py`는 actuator motor의 `ctrlrange/forcerange`만 바꿨고, joint의 `actuatorfrcrange`는 바꾸지 않았다. MJCF에는 각 hinge joint에 `actuatorfrcrange="-100 100"`이 남아 있었기 때문에, 이전 actuator sweep은 joint force clamp에 의해 실제로는 100 Nm 제한을 계속 받았을 가능성이 있다.

수정 후에는 다음 두 항목을 함께 변경한다.

- motor `ctrlrange`
- motor `forcerange`
- joint `actuatorfrcrange`

## 새 200 Nm 모델

```bash
python3 scripts/apply_actuator_limit.py \
  --source envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml \
  --limit 200 \
  --out envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact_act200full.xml \
  --doc-dir docs/hardware_validation
```

확인 결과 새 모델은 motor와 joint clamp가 모두 `-200 200`으로 변경되었다.

## 200 Nm Stabilizer Sweep

```bash
python3 scripts/sweep_stabilized_standing.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact_act200full.xml \
  --pose-json configs/quasistatic_standing_pose_inertia_direct_nobase.json \
  --torque-limit 200 \
  --out-dir outputs/analysis/stabilized_standing_sweep_act200full \
  --doc-dir docs/hardware_validation/tmp
```

## 결과 비교

| 조건 | final base z | final roll | final pitch | max abs roll | max abs pitch | max qvel norm | max contact force | saturation |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| quasi-static pose, no stabilizer, 100 Nm | 0.132326 | 3.023535 | 0.613897 | - | - | 195.292698 | 1951.860573 | - |
| stabilizer sweep best, 100 Nm | 0.178721 | -2.970172 | 0.004192 | 3.129725 | 0.252447 | 197.272821 | 1858.071654 | 0.995025 |
| stabilizer sweep best, 200 Nm full clamp | 0.096523 | 2.167100 | 0.451000 | 3.137035 | 0.459182 | 347.794474 | 2341.996354 | 0.985075 |

## Best 200 Nm Config

- `kp_att`: `8.0`
- `kd_att`: `4.0`
- `kcom`: `8.0`
- `hip_roll_sign`: `+1`
- `ankle_roll_sign`: `+1`
- `hip_pitch_sign`: `-1`
- `ankle_pitch_sign`: `+1`

## 산출물

- 200 Nm full clamp 모델: `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact_act200full.xml`
- 200 Nm sweep summary: `outputs/analysis/stabilized_standing_sweep_act200full/stabilizer_sweep_summary.json`
- 200 Nm sweep CSV: `outputs/analysis/stabilized_standing_sweep_act200full/stabilizer_sweep_summary.csv`
- 200 Nm best run CSV: `outputs/analysis/stabilized_standing_sweep_act200full/best_stabilized_standing.csv`
- 200 Nm best run plot: `outputs/analysis/stabilized_standing_sweep_act200full/neutral_pd_standing_plot.png`

## 판단

200 Nm까지 올리고 joint clamp까지 제대로 풀어도 standing은 안정화되지 않았다. saturation fraction은 여전히 `0.985075`로 높고, max qvel/contact force는 100 Nm보다 오히려 커졌다.

따라서 현재 실패를 단순히 “100 Nm 모터가 약해서”라고 보기 어렵다. 특히 roll 방향은 stabilizer와 torque 증대에도 계속 무너진다.

다음 원인 후보의 우선순위가 올라간다.

1. roll 방향 foot contact 모델이 실제 발 지지 모멘트를 충분히 만들지 못함
2. ankle/hip roll joint axis 또는 sign 해석이 실제 CAD와 다름
3. `base_link`와 hip-roll actuator 계열 inertial frame이 아직 틀림
4. 현재 heuristic stabilizer가 실제 ZMP/COM 제어 구조를 충분히 반영하지 못함

다음 단계는 roll 방향을 집중적으로 분해해야 한다. 즉 양발 contact patch를 더 현실적으로 만들고, roll joint positive torque가 발/몸통을 어느 방향으로 회전시키는지 torque impulse response로 확인한다.

