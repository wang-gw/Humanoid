# CAD-MuJoCo Axis and Edge Contact Audit

## 목적

사용자가 제공한 CAD World ↔ MuJoCo 축 대응과 발바닥 edge 좌표를 현재 MJCF contact 모델에 적용할 수 있는지 확인한다.

## 확정된 축 대응

| 물리적 방향 | CAD World 축 | MuJoCo 축 |
| --- | --- | --- |
| 앞 Forward | `+Y` | `+X` |
| 뒤 Backward | `-Y` | `-X` |
| 위 Up | `+Z` | `+Z` |
| 아래/중력 | `-Z` | `-Z` |
| 왼쪽 Left | `+X` | `+Y` |
| 오른쪽 Right | `-X` | `-Y` |

변환식:

```text
MuJoCo_X = CAD_Y
MuJoCo_Y = CAD_X
MuJoCo_Z = CAD_Z
```

행렬:

```text
[ X_mj ]   [ 0  1  0 ] [ X_cad ]
[ Y_mj ] = [ 1  0  0 ] [ Y_cad ]
[ Z_mj ]   [ 0  0  1 ] [ Z_cad ]
```

주의: 이 축 대응이 맞다면 ground height는 `CAD Z=0 -> MuJoCo Z=0`이다. `CAD Y=-60 mm`는 발바닥 접촉 edge의 앞뒤 위치이며, MuJoCo에서는 `X=-0.060 m` 위치가 된다. 따라서 `Y=-60 mm`를 MuJoCo 높이 보정값으로 쓰면 안 된다.

## 입력 좌표 해석

사용자가 제공한 실제 지면 edge:

- `foot_L`: `#0`, `#2`
- `foot_R`: `#1`, `#3`

축 변환 후 CAD world edge는 다음과 같다.

| point | MuJoCo world X | MuJoCo world Y | MuJoCo world Z |
| --- | ---: | ---: | ---: |
| left/right edge A | -0.060 | -0.035 | 0.000 |
| left/right edge B | -0.060 | 0.035 | 0.000 |

이 값만 보면 좌우 발의 world contact edge가 같은 위치에 겹친다. 따라서 제공된 `World X/Y/Z`는 현재 MJCF의 global world 좌표로 바로 쓰기 어렵다.

CAD local 좌표를 축 변환하면 다음과 같다.

| foot | MuJoCo local center candidate | edge half span |
| --- | --- | ---: |
| left | `[0.000, -0.07675, -0.025]` | `0.035 m` |
| right | `[0.000, -0.07725, 0.025]` | `0.035 m` |

이 값은 좌우 발의 local Z가 `50 mm` 차이난다. 현재 MJCF foot body local frame에 그대로 넣으면 좌우 contact 높이가 크게 달라진다.

## 생성한 실험 모델

스크립트:

```bash
python3 scripts/build_cad_edge_contact_variant.py
```

생성 모델:

| mode | 모델 |
| --- | --- |
| `cad_world_fit` | `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_cad_edge_worldfit_contact.xml` |
| `cad_local` | `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_cad_edge_local_contact.xml` |

## qpos0 Contact 위치 비교

| model | left world center | right world center | left bottom | right bottom | 판단 |
| --- | --- | --- | ---: | ---: | --- |
| `cad_world_fit` | `[-0.060, 0.000, 0.003]` | `[-0.060, 0.000, 0.003]` | `0.000` | `0.000` | 좌우 contact가 겹침 |
| `cad_local` | `[0.067, -0.09825, 0.018]` | `[-0.087, -0.09875, 0.068]` | `0.015` | `0.065` | 좌우 높이 50 mm 차이 |
| current | `[0.067, -0.02150, 0.020]` | `[-0.087, -0.02150, 0.020]` | `0.000` | `0.000` | 높이는 맞지만 CAD edge 위치와 다름 |

## 판단

축 대응 정보는 충분하고 유효하다. 그러나 제공된 발바닥 edge 좌표는 현재 MJCF contact 모델에 바로 넣기에는 아직 frame 해석이 맞지 않는다.

현재 확인된 불일치:

1. CAD world edge 좌표를 global world로 보면 좌우 발 contact가 같은 위치에 겹친다.
2. CAD local edge 좌표를 현재 MJCF foot body local로 보면 좌우 발 contact 높이가 `50 mm` 차이난다.
3. 따라서 CAD `World`가 실제 assembly global인지, part-local을 world라고 부른 것인지 추가 확인이 필요하다.

## 다음 조치

다음 입력 중 하나가 필요하다.

1. 좌우 발 각각의 실제 assembly world 좌표. 즉 왼발/오른발 contact edge가 global 좌표에서 서로 다른 stance 위치를 가져야 한다.
2. CAD foot local frame과 MJCF foot body frame 사이의 transform.
3. Fusion에서 `foot_L:1`, `foot_R:1` occurrence transform을 포함한 contact edge 좌표.

이 중 하나가 확정되면 CAD edge contact 모델을 최종 후보로 다시 만들고 roll impulse/standing probe를 재실행한다.

