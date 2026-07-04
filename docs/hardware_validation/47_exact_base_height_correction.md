# Exact Base Height Correction

## 목적

이전 standing/actuator probe에서 초기 `base_z`를 자동 또는 수동으로 잡을 때, 일부 결과가 실제 foot contact box의 바닥면이 아니라 MuJoCo geom의 `rbound` 기준으로 해석되었다. `rbound`는 박스의 실제 z 반폭이 아니라 박스를 감싸는 구의 반경이므로, 발바닥 box 하단 높이 계산에 쓰면 base 높이가 과대 추정된다.

이번 단계의 목적은 다음과 같다.

- `base_z` 산정 기준을 실제 접촉 박스 하단으로 수정
- 현재 모델의 정확한 초기 발바닥 위치 재확인
- 수정 전 결과를 해석할 때 주의할 점 문서화

## 수정한 스크립트

- `scripts/analyze_standing_geometry.py`
- `scripts/probe_pd_standing.py`
- `scripts/audit_joint_sign_response.py`

수정 내용은 다음과 같다.

- box geom은 `geom_size`와 `geom_xmat`을 사용해 실제 world z 방향 반폭을 계산한다.
- non-box geom만 기존 `geom_rbound` fallback을 사용한다.
- `probe_pd_standing.py`의 자동 `base_z` 추정은 contact geom만 대상으로 한다.

## 실행 명령

```bash
python3 scripts/analyze_standing_geometry.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml \
  --doc-dir docs/hardware_validation/user_size_mass_contact_geometry_exact

python3 scripts/probe_pd_standing.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml \
  --out-dir outputs/analysis/pd_standing_user_size_mass_contact_zero_basez_auto_exact \
  --doc-dir docs/hardware_validation/tmp \
  --doc-name auto_exact.md
```

## 정확한 초기 기하 결과

사용 모델:

`envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml`

| 항목 | 값 |
| --- | ---: |
| 정확한 자동 `base_z` | `0.005000 m` |
| 전체 질량 | `9.211400 kg` |
| COM world | `[0.053358, -0.073768, 0.168206]` |
| support x 범위 | `[-0.122000, 0.102000] m` |
| support y 범위 | `[-0.081500, 0.038500] m` |
| COM inside support x | `True` |
| COM inside support y | `True` |

Foot collision geom:

| geom | world pos | halfsize |
| --- | --- | --- |
| `foot_L_1_sole_collision` | `[0.067000, -0.021500, 0.025000]` | `[0.035000, 0.060000, 0.020000]` |
| `foot_R_v1_1_sole_collision` | `[-0.087000, -0.021500, 0.025000]` | `[0.035000, 0.060000, 0.020000]` |

따라서 contact box 하단은 `0.025 - 0.020 = 0.005 m`이고, clearance `0.005 m` 기준에서는 `base_z=0.005 m`가 맞다. 완전 접촉 상태로 시작하려면 `base_z=0.000 m`가 된다.

## 수정 후 PD Probe 결과

자동 높이 산정 결과:

| 항목 | 값 |
| --- | ---: |
| initial `base_z` | `0.005000 m` |
| final base z | `0.530889 m` |
| final roll | `1.591079 rad` |
| final pitch | `0.670890 rad` |
| max qvel norm | `800.387554` |
| max contact normal force | `3688.948443` |

기존에 같은 `base_z=0.005 m`를 수동으로 넣은 결과는 다음과 같았다.

| 항목 | 값 |
| --- | ---: |
| initial `base_z` | `0.005000 m` |
| final base z | `0.131486 m` |
| final roll | `-2.453577 rad` |
| final pitch | `0.150364 rad` |
| max qvel norm | `857.401243` |
| max contact normal force | `4711.977344` |

두 실행은 초기 높이가 사실상 동일하지만, 접촉이 불안정해진 뒤 최종 자세가 크게 갈라진다. 따라서 현재 단계에서는 최종 pose 하나를 정량 설계 기준으로 쓰면 안 된다. 더 중요한 관찰은 두 경우 모두 standing 안정화에 실패하고, 속도/접촉력/토크가 크게 튄다는 점이다.

## 판단

이전 `base_z=0.057284 m` 기준 결과는 실제 발바닥 contact가 바닥보다 떠 있는 조건이었으므로, 정확한 standing/contact 판정에는 사용하지 않는다.

수정 후 기준은 다음과 같이 고정한다.

- 기하/COM sanity check: 자동 `base_z=0.005 m`
- 바닥 접촉 시작 probe: 필요 시 `--base-z 0.0`
- clearance 포함 시작 probe: `--base-z 0.005`

현재 모델은 정확한 contact 높이에서도 neutral joint-space PD standing에 실패한다. 따라서 다음 단계는 단순히 모터 토크를 키우는 것이 아니라, 균형 제어/준정적 자세 탐색 또는 mass/inertia frame 정밀화를 먼저 진행해야 한다.

