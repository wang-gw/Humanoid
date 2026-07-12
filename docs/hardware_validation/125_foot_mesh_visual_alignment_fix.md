# 125. Foot Mesh Visual Alignment Fix

## 목적

렌더링 영상에서 왼발 시각적 mesh와 collision box(초록 박스)가 크게 어긋나 보이는 문제를 수정한다. 물리(contact) geom은 변경하지 않으므로 기존 학습 policy에 영향 없다.

## 증상

`outputs/e2e_walk_tracking.mp4` 기준 측면 영상(azimuth=90)에서:
- 왼발: 회색 발바닥 mesh가 초록 collision box 옆으로 크게 벗어남
- 오른발: 상대적으로 양호해 보임

## 원인 분석

**파일**: `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

원본 mesh geom pos (두 발 공통):
```xml
pos="0.00975 -0.0385 -0.015"
```

### 3축 어긋남

**Y축 (+3.85cm)**
- STL foot_L/foot_R 모두 Y: [-6cm, +6cm] 범위 (centered)
- 원본 mesh local Y = -0.0385로 배치 → 발이 -Y 방향으로 3.85cm 이동해 보임

**Z축 (+3.4cm)**
- Collision box local Z: [-0.0485, 0] (발바닥이 body origin 아래 4.85cm)
- 원본 mesh local Z = -0.015 → mesh 하단이 body origin 아래 1.5cm (box 하단보다 3.35cm 위)
- 결과: 시각적 발이 contact 지점보다 공중에 떠 있는 것처럼 보임

**X축 (좌우 발 부호 반전)**
- `foot_L.stl` STL bbox X center: [−3.5cm, +3.5cm] → center 0
- `foot_R.stl`은 quat `(0,0,1,0)` (180°Y 회전) 적용 → X 부호 뒤집힘
- 따라서 두 발의 필요 X offset 부호가 서로 반대여야 함
- 원본은 두 발 모두 X=+0.00975 → 오른발 X가 ~2cm 어긋남

### 계산 방법

각 발의 올바른 geom pos = **collision box bbox 중심** − **STL bbox 중심 (회전 적용 후)**

```python
R = quat_to_mat(geom_quat)
mesh_center = ((R @ verts.T).T.min(0) + (R @ verts.T).T.max(0)) / 2
box_center  = (box_bbox_min + box_bbox_max) / 2
new_pos     = box_center - mesh_center
```

계산 결과:
- `foot_L_1`: `box_center = [+0.020, 0, -0.02425]`, `mesh_center ≈ [0, 0, +0.03186]`
  → `new_pos = [+0.020, 0, -0.05611]`
- `foot_R_v1_1`: `box_center = [-0.020, 0, -0.02425]`, `mesh_center ≈ [0, 0, +0.03186]`
  → `new_pos = [-0.020, 0, -0.05611]`

## 수정

```xml
<!-- 수정 전 (두 발 공통) -->
<geom pos="0.00975 -0.0385 -0.015" ... type="mesh" mesh="foot_L_1"   ... />
<geom pos="0.00975 -0.0385 -0.015" ... type="mesh" mesh="foot_R_v1_1" ... />

<!-- 수정 후 -->
<geom pos=" 0.020 0 -0.05611" ... type="mesh" mesh="foot_L_1"   ... />
<geom pos="-0.020 0 -0.05611" ... type="mesh" mesh="foot_R_v1_1" ... />
```

변경 항목: Y (−0.0385→0), Z (−0.015→−0.05611), X (0.00975→±0.020, 발마다 다름)

## 검증

수정 후 mesh bbox 중심 = collision box bbox 중심 (body frame 기준):

| 발 | mesh Z range | box Z range |
|---|---|---|
| foot_L_1 | [−0.0561, +0.0076] | [−0.0485, 0] |
| foot_R_v1_1 | [−0.0561, +0.0076] | [−0.0485, 0] |

발바닥(STL Z=0 면)이 collision box 하단 근처에 위치. 완전 일치하지 않는 이유는 발 body frame이 약간 기울어져 있어 mesh의 Z=0 면이 엄밀한 수평이 아니기 때문.

## 영향

- 물리(contact geom) 변경 없음 → 학습된 policy 행동 동일
- 시각적 발 형상이 실제 contact 위치와 일치
- 이후 렌더링 영상에서 mesh/collision box 정렬 확인 가능

## 산출물

| 파일 | 설명 |
|---|---|
| `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml` | mesh geom pos 수정 |
| `outputs/e2e_walk_tracking.mp4` | 수정 후 재렌더링 영상 |
