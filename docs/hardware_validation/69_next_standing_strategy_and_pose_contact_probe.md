# Standing을 만들기 위한 다음 전략

## 목적

렌더링 문제가 해결되어 로봇 형체를 확인할 수 있게 되었으므로, 이제 실제로 서게 하려면 무엇을 해야 하는지 정리한다. 이번 단계에서는 `Toe/Heel Z contact + STEP visual hybrid` 모델을 기준으로 pose 재탐색, base height 보정, 단순 PD, stabilizer, gain sweep를 실행했다.

## 기준 모델

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml
```

이 모델은 다음 특징을 가진다.

- physics/contact는 64번 확정 모델과 동일
- visual mesh는 STEP assembly 기준으로 보정
- 전체 렌더에서 로봇 형체를 확인하기 쉬움

## 1. 새 기준 모델에서 standing pose 재탐색

실행:

```bash
python3 scripts/search_quasistatic_standing_pose.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml \
  --samples 80000 \
  --iterations 10 \
  --elite 384 \
  --seed 11 \
  --limit 0.8 \
  --out configs/quasistatic_standing_pose_toeheel_z_step_visual.json
```

결과:

| 항목 | 값 |
| --- | ---: |
| cost | `0.061044` |
| base_z | `0.007022 m` |
| joint norm | `0.403267` |
| COM world | `(0.066400, -0.052086, 0.292210) m` |
| support x margin | `0.092253 / 0.094385 m` |
| support y margin | `0.037146 / 0.038704 m` |

문제:

- 이 pose의 발바닥 최저점은 바닥보다 약 `5 mm` 위에 있다.
- 따라서 그대로 시뮬레이션하면 시작 순간 contact가 `0`이고, “서기”가 아니라 “공중에서 떨어지기”에 가깝다.

## 2. Contact 시작용 base_z 보정

위 문제를 피하기 위해 같은 joint target을 유지하고 base_z만 `0.006 m` 낮춘 pose를 만들었다.

```text
configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json
```

보정값:

| 항목 | 값 |
| --- | ---: |
| 기존 base_z | `0.007022 m` |
| contact용 base_z | `0.001022 m` |

초기 상태:

- contacts: `3`
- contact force: 약 `61.4 N`

즉 이제는 접촉 상태에서 standing을 테스트한다.

## 3. Contact pose 단순 PD 결과

실행:

```bash
python3 scripts/probe_pd_standing.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml \
  --pose-json configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json \
  --out-dir outputs/analysis/pd_standing_toeheel_z_step_visual_contact_pose \
  --duration 2.0 \
  --kp 60 \
  --kd 4 \
  --torque-limit 100
```

결과:

| 항목 | 값 |
| --- | ---: |
| final base_z | `0.016142 m` |
| final roll | `2.607548 rad` |
| final pitch | `-0.273205 rad` |
| max qvel norm | `151.063643` |
| max contact force | `1684.520 N` |

판단:

- 접촉 상태에서 시작해도 단순 joint PD는 실패한다.
- 토크가 빠르게 포화되고 roll 방향으로 넘어진다.

영상:

```text
outputs/analysis/render_pd_standing_toeheel_z_step_visual_contact_pose_wide/pd_standing_render.mp4
```

## 4. Stabilizer Sweep 결과

base roll/pitch와 COM 오차를 hip/ankle torque에 추가하는 간단한 stabilizer를 432개 조합으로 sweep했다.

실행:

```bash
python3 scripts/sweep_stabilized_standing.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml \
  --pose-json configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json \
  --out-dir outputs/analysis/stabilized_standing_toeheel_z_step_visual_contact_pose \
  --duration 2.0 \
  --joint-kp 60 \
  --joint-kd 4 \
  --torque-limit 100
```

Best result:

| 항목 | 값 |
| --- | ---: |
| best score | `32.313245` |
| final base_z | `-0.017862 m` |
| final roll | `-2.041031 rad` |
| final pitch | `-0.142492 rad` |
| max abs roll | `3.133306 rad` |
| max qvel norm | `215.187207` |
| max contact force | `1582.383 N` |
| saturation fraction | `0.990050` |

Best config:

| 항목 | 값 |
| --- | ---: |
| kp_att | `8.0` |
| kd_att | `2.0` |
| kcom | `0.0` |
| hip_roll_sign | `-1` |
| ankle_roll_sign | `-1` |
| hip_pitch_sign | `+1` |
| ankle_pitch_sign | `+1` |

판단:

- 간단한 stabilizer를 추가해도 서지 못한다.
- 거의 모든 샘플에서 torque saturation이 발생한다.
- 즉 “제어기를 조금 보태면 서는 상태”가 아니다.

## 5. Joint PD Gain Sweep

토크가 너무 빠르게 포화되므로, 더 약한 gain도 확인했다.

테스트 범위:

- `Kp`: 5, 10, 20, 40, 60
- `Kd`: 0.5, 1, 2, 4, 8, 12
- torque limit: 30, 60, 100 Nm

가장 안정적으로 보인 low-gain 후보:

| 항목 | 값 |
| --- | ---: |
| Kp | `10` |
| Kd | `1` |
| torque limit | `30 Nm` |
| final roll | `-0.414763 rad` |
| final pitch | `0.068601 rad` |
| final base_z | `-0.253613 m` |
| max qvel norm | `4.226658` |
| max contact force | `92.037 N` |
| contact fraction | `1.0` |
| saturation fraction | `0.0` |

판단:

- 낮은 gain에서는 접촉은 유지되고 튕겨 나가지는 않는다.
- 하지만 base가 `-0.25 m`까지 내려가므로 서는 것이 아니라 주저앉는다.
- 즉 “강한 PD는 튕겨 넘어짐, 약한 PD는 버티지 못함”이라는 상태다.

Low-gain 영상:

```text
outputs/analysis/render_pd_standing_toeheel_z_step_visual_contact_pose_low_gain/pd_standing_render.mp4
```

## 현재 결론

서게 하려면 단순히 다음 중 하나만 하면 되는 상태가 아니다.

- torque limit 증가
- PD gain 증가
- COM을 support 안에 넣기
- contact pad 위치 보정

위 항목들은 이미 일부 확인했고, 아직 실패한다.

현재 실패 양상은 다음과 같다.

1. contact 상태에서 시작해도 high-gain PD는 빠르게 튕기며 roll 방향으로 넘어진다.
2. stabilizer를 추가해도 roll을 잡지 못하고 torque가 거의 항상 포화된다.
3. low-gain PD는 튕기지는 않지만 다리 구조가 하중을 지탱하지 못하고 주저앉는다.

따라서 다음에 봐야 할 핵심은 `ankle/leg support 기하와 joint axis/actuator direction`이다.

## 서게 만들기 위한 우선순위

### 1. 초기 접촉 조건 고정

앞으로 standing 테스트는 다음 pose를 기준으로 한다.

```text
configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json
```

이전 pose처럼 foot이 바닥에서 5 mm 떠 있는 상태로 시작하면 안 된다.

### 2. Joint axis / actuator direction 검증

특히 다음 4개가 중요하다.

- `left_ankle_pitch`
- `left_ankle_roll`
- `right_ankle_pitch`
- `right_ankle_roll`

확인해야 할 질문:

- ankle pitch torque가 실제로 발바닥을 지면에 밀어 하중을 받는 방향인가?
- ankle roll torque가 base roll을 복원하는 방향인가?
- 좌우 ankle sign이 대칭적으로 맞는가?
- hip roll과 ankle roll이 같은 roll 오차에 대해 서로 보완하는가, 아니면 서로 싸우는가?

### 3. Standing pose를 “COM 중심”이 아니라 “하중 지지” 기준으로 다시 찾기

지금 pose search는 COM/support/foot height 위주다. 하지만 low-gain에서 주저앉는 것을 보면, knee/ankle가 하중을 받기 좋은 자세인지가 더 중요하다.

다음 pose search에는 다음 항목을 추가해야 한다.

- knee가 너무 접히거나 펴져 torque arm이 불리하지 않게 제한
- foot pitch/roll이 지면과 평행하게 유지
- base 높이가 실제 기구학적으로 충분히 유지
- ankle pitch/roll 중립 근처에서 큰 보상 토크가 필요하지 않게 제한

### 4. Motor/gear 모델은 아직 최종 선정 단계가 아님

현재 `100 Nm`에서 포화되지만, 단순히 더 큰 모터를 넣는 것으로 해결된다고 보면 안 된다. 이미 actuator limit을 올렸을 때도 안정성이 근본적으로 개선되지 않았던 기록이 있다.

먼저 joint axis, pose, contact, inertia가 맞는지 확인해야 한다.

## 다음 실행 제안

다음 단계는 `ankle pitch/roll axis response audit`를 새 기준 모델에서 다시 실행하는 것이다.

목표:

- 각 ankle joint에 작은 +/− 각도 또는 torque를 넣었을 때 foot contact pad와 base가 어느 방향으로 움직이는지 시각/수치로 확인
- “서기 위한 복원 방향”과 현재 actuator sign이 맞는지 확정

이 검증을 통과한 뒤에야 standing controller를 다시 설계하는 것이 맞다.

## 산출물

- 새 pose: `configs/quasistatic_standing_pose_toeheel_z_step_visual.json`
- contact 시작 pose: `configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json`
- 단순 PD 결과: `outputs/analysis/pd_standing_toeheel_z_step_visual_contact_pose/neutral_pd_standing_summary.json`
- 단순 PD 영상: `outputs/analysis/render_pd_standing_toeheel_z_step_visual_contact_pose_wide/pd_standing_render.mp4`
- stabilizer sweep: `outputs/analysis/stabilized_standing_toeheel_z_step_visual_contact_pose/stabilizer_sweep_summary.json`
- low-gain 영상: `outputs/analysis/render_pd_standing_toeheel_z_step_visual_contact_pose_low_gain/pd_standing_render.mp4`

