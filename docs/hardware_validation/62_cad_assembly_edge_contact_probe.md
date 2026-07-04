# CAD Assembly Edge Contact Probe

## 목적

사용자가 새로 제공한 `foot_L:1`, `foot_R:1` assembly world 좌표를 이용해 CAD 기반 edge contact 모델을 만들고, 기존 contact 모델과 비교한다.

## 입력 좌표

축 대응:

```text
MuJoCo_X = CAD_Y
MuJoCo_Y = CAD_X
MuJoCo_Z = CAD_Z
```

새 CAD 좌표는 좌우 발이 CAD X 방향으로 분리되어 있어 이전 좌표보다 assembly world 좌표로서 더 타당하다.

다만 CAD Z가 `25~35 mm` 범위이므로, 이번 실험에서는 `min CAD Z = 25 mm`를 contact ground 기준으로 정규화했다.

## 생성 모델

스크립트:

```bash
python3 scripts/build_cad_assembly_edge_contact_variant.py
```

모델:

`envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_cad_assembly_edge_contact.xml`

생성된 qpos0 contact 위치:

| foot | world center | halfsize | bottom | support range |
| --- | --- | --- | ---: | --- |
| left | `[-0.120, 0.07675, 0.003]` | `[0.006, 0.035, 0.003]` | `0.000` | x `[-0.126, -0.114]`, y `[0.04175, 0.11175]` |
| right | `[-0.120, -0.07725, 0.003]` | `[0.006, 0.035, 0.003]` | `0.000` | x `[-0.126, -0.114]`, y `[-0.11225, -0.04225]` |

## Geometry Check

```bash
python3 scripts/analyze_standing_geometry.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_cad_assembly_edge_contact.xml \
  --doc-dir docs/hardware_validation/cad_assembly_edge_contact_geometry
```

결과:

| 항목 | 값 |
| --- | ---: |
| total mass | `9.211400 kg` |
| COM world | `[0.053076, -0.047922, 0.280897]` |
| support x range | `[-0.126000, -0.114000]` |
| support y range | `[-0.112250, 0.111750]` |
| COM inside support x | `False` |
| COM inside support y | `True` |

COM X가 support x range보다 훨씬 앞쪽에 있다. 따라서 이 CAD edge contact를 실제 지지 접촉으로 보면, 현재 자세는 정적으로 설 수 없다.

## Roll Impulse 비교

| contact model | no-impulse final roll | no-impulse delta roll | max contact force | max qvel |
| --- | ---: | ---: | ---: | ---: |
| single box | 0.167927 | 0.167385 | 2334.177866 | 195.124686 |
| CAD assembly edge | 0.131973 | 0.131931 | 1278.291379 | 178.085506 |

CAD assembly edge contact는 roll drift와 contact force peak를 약간 줄였지만, contact patch가 edge 형태라 support x 폭이 `12 mm`뿐이다.

## PD Probe

```bash
python3 scripts/probe_pd_standing.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_cad_assembly_edge_contact.xml \
  --pose-json configs/quasistatic_standing_pose_inertia_direct_nobase.json \
  --out-dir outputs/analysis/pd_standing_cad_assembly_edge_contact_quasistatic_pose
```

결과:

| 항목 | 값 |
| --- | ---: |
| final base z | `-0.002553` |
| final roll | `2.907909 rad` |
| final pitch | `0.468277 rad` |
| max qvel norm | `189.535196` |
| max contact normal force | `1376.989520` |

standing은 실패한다.

## 판단

이번 새 좌표는 좌우 stance width를 만들기 때문에 이전 좌표보다 훨씬 타당하다. 하지만 이 좌표가 나타내는 실제 접촉은 발바닥 면이 아니라 `X=-0.120 m` 근처의 얇은 edge contact다.

현재 COM X는 `0.053 m`이고, CAD edge support X는 `[-0.126, -0.114] m`이다. 즉 COM projection이 contact edge보다 약 `167 mm` 앞쪽에 있다. 이 상태는 정적 standing 조건을 만족하지 않는다.

따라서 다음 중 하나가 필요하다.

1. CAD 좌표가 실제 “발바닥 접촉면 전체”가 아니라 특정 edge만 준 것이라면, 발바닥 전체 패드/면 좌표를 다시 받아야 한다.
2. 실제 설계가 edge contact라면, 현재 발 형상으로는 정적 standing이 매우 어렵고 foot sole/pad 설계 수정이 필요하다.
3. 좌표의 Toe/Heel 방향 라벨이 CAD 축과 맞지 않을 수 있으므로, toe/heel을 구분하는 실제 앞뒤 좌표가 추가로 필요하다.

현재 하드웨어 검증 관점에서는 foot contact 형상이 가장 큰 blocking issue다. RL 이전에 발바닥 접촉면이 COM projection을 포함할 수 있도록 설계 또는 contact 정의를 확정해야 한다.
