# Actuator Limit Sweep

> 주의: 이 문서는 당시 `base_z=0.057284 m` 조건에서 수행한 sweep 기록이다. 이후 `47_exact_base_height_correction.md`에서 확인했듯이 이 높이는 foot contact box의 실제 하단이 아니라 `geom_rbound` 기준으로 과대 추정된 값이었다. 정확한 foot contact 기준 sweep은 `48_exact_base_actuator_sweep.md`를 기준 문서로 사용한다.

> 추가 주의: 이후 `56_stabilizer_actuator_limit_recheck.md`에서 `apply_actuator_limit.py`가 motor range만 바꾸고 joint `actuatorfrcrange`를 바꾸지 않았던 문제가 확인되어 수정되었다. 따라서 이 문서의 200/300 Nm 결과는 역사적 기록으로만 둔다.

## 목적

43번 probe에서 ankle pitch/roll torque가 `+/-100 Nm` 제한에 계속 닿았다. 따라서 actuator limit을 `200 Nm`, `300 Nm`로 올린 별도 MJCF variant를 만들어, standing 실패가 단순 토크 제한 때문인지 확인한다.

## 실행 명령

```bash
python3 scripts/apply_actuator_limit.py --source envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml --limit 200 --out envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact_act200.xml
python3 scripts/apply_actuator_limit.py --source envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml --limit 300 --out envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact_act300.xml

python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact_act200.xml --base-z 0.05728416147400483 --torque-limit 200 --out-dir outputs/analysis/pd_standing_user_size_mass_contact_act200_zero_basez
python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact_act300.xml --base-z 0.05728416147400483 --torque-limit 300 --out-dir outputs/analysis/pd_standing_user_size_mass_contact_act300_zero_basez
```

## 결과 비교

| actuator limit | final base z | final roll | final pitch | max qvel norm | max contact force |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 100 Nm | 2.912016 | 0.421224 | -0.464444 | 791.293011 | 5696.328774 |
| 200 Nm | 2.912016 | 0.421224 | -0.464444 | 791.293011 | 5696.328774 |
| 300 Nm | 2.912016 | 0.421224 | -0.464444 | 791.293011 | 5696.328774 |

## Ankle RMS Torque

| actuator | 100 Nm model | 200 Nm model | 300 Nm model |
| --- | ---: | ---: | ---: |
| `motor_left_ankle_pitch` | 94.680263 | 182.650058 | 264.008649 |
| `motor_left_ankle_roll` | 95.707099 | 188.960925 | 278.412387 |
| `motor_right_ankle_pitch` | 96.865001 | 188.338486 | 274.527326 |
| `motor_right_ankle_roll` | 96.723419 | 190.857053 | 281.039606 |

## 산출물

- 200 Nm 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact_act200.xml`
- 300 Nm 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact_act300.xml`
- 200 Nm plot: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_size_mass_contact_act200_zero_basez/neutral_pd_standing_plot.png`
- 300 Nm plot: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_size_mass_contact_act300_zero_basez/neutral_pd_standing_plot.png`

## 판단

actuator limit을 올리면 ankle 계열 torque는 실제로 더 크게 사용된다. 하지만 base 자세, 속도, contact force 결과는 개선되지 않았다.

따라서 현재 standing 실패는 단순히 `100 Nm` 제한이 부족해서 생긴 문제로 보기 어렵다. 더 큰 모터를 고르는 방향으로 바로 넘어가면 안 된다.

다음 원인 후보는 다음과 같다.

- ankle joint axis/sign 또는 link frame 해석 문제
- foot contact가 기하적으로는 맞지만 동역학 contact로는 너무 두껍거나 위치가 부적절한 문제
- mass/inertia aggregate에서 COM/inertia가 정확히 변환되지 않은 문제
- 단순 joint-space PD controller가 floating-base standing에 부적합한 문제

다음 단계는 ankle pitch/roll을 중심으로 joint sign과 contact response를 더 세밀하게 확인하고, 필요하면 안정화용 base/COM PD 또는 quasi-static IK standing 검증으로 넘어가는 것이다.
