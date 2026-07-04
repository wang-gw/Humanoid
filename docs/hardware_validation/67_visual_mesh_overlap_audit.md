# Visual Mesh Overlap Audit

## 목적

66번 PD standing 렌더링에서 로봇 형체가 명확히 보이지 않는 문제가 있었다. 이 문서는 그 원인이 카메라/조명 문제인지, 실제 MJCF visual mesh 배치 문제인지 확인한다.

## 결론

렌더링이 뭉개져 보인 주된 이유는 visual mesh들이 실제로 많이 겹쳐 있기 때문이다.

현재 모델은 동역학용 body/joint 체인은 어느 정도 다리 형태로 배치되어 있지만, STL visual mesh들의 frame/origin 정렬이 깨끗하지 않다. 특히 여러 actuator mesh가 같은 위치에 렌더링되어 회색 덩어리처럼 보인다.

따라서 66번 영상은 다음 용도로만 신뢰한다.

- base roll/pitch가 커지는지
- 접촉이 끊기는지
- contact pad가 어느 쪽에서 남는지
- 시뮬레이션이 동역학적으로 발산하는지

하지만 다음 용도로는 아직 신뢰하면 안 된다.

- 실제 로봇 외형 확인
- 링크 간 물리적 간섭 판단
- CAD 조립 형상 검증
- 어떤 링크가 정확히 어느 방향으로 움직이는지 시각적으로 판정

## 확인한 body 위치

neutral pose에서 주요 body/joint 위치는 다음처럼 다리 체인 형태를 가진다.

| body | world position m |
| --- | --- |
| `base_link` | `(0.000, 0.000, 0.000)` |
| `thighJ_L_1` | `(0.040, -0.029, 0.440)` |
| `thigh_L_1` | `(0.067, 0.000, 0.370)` |
| `calf_L_1` | `(0.067, 0.000, 0.240)` |
| `footJ_L_1` | `(0.067, 0.000, 0.040)` |
| `foot_L_1` | `(0.067, -0.0215, 0.040)` |
| `thighJ_R_1` | `(-0.060, -0.029, 0.440)` |
| `thigh_R_1` | `(-0.087, 0.000, 0.370)` |
| `calf_R_1` | `(-0.087, 0.000, 0.240)` |
| `footJ_R_1` | `(-0.087, 0.000, 0.040)` |
| `foot_R_v1_1` | `(-0.087, -0.0215, 0.040)` |

즉 joint/body tree 자체가 완전히 한 점에 collapsed 된 것은 아니다.

## 확인한 visual mesh overlap

MuJoCo mesh는 `geom_xpos`만으로 실제 visual 중심을 볼 수 없으므로, `mesh_pos`까지 포함한 visual center를 계산했다.

계산식:

```text
visual_center = geom_xpos + geom_xmat * mesh_pos
```

대표 문제:

| mesh group | visual center 문제 |
| --- | --- |
| `AK45-36_trL_1`, `AK45-36_tpL_1`, `AK45-36_kpL_1`, `AK45-36_fpL_1` | 모두 거의 `(0, 0, +0.054) m`에 겹침 |
| `AK45-36_trR_1`, `AK45-36_tpR_1`, `AK45-36_kpR_1`, `AK45-36_fpR_1` | 모두 거의 `(0, 0, -0.054) m`에 겹침 |
| `footJ_L_1`, `footJ_R_1`, foot mesh 일부 | 중심 간 거리가 수 mm~수 cm 수준으로 매우 가까움 |

예시:

| mesh A | mesh B | visual center distance |
| --- | --- | ---: |
| `AK45-36_trL_1` | `AK45-36_tpL_1` | `0.0000 m` |
| `AK45-36_trL_1` | `AK45-36_kpL_1` | `0.0000 m` |
| `AK45-36_trL_1` | `AK45-36_fpL_1` | `0.0000 m` |
| `AK45-36_trR_1` | `AK45-36_tpR_1` | `0.0000 m` |
| `AK45-36_trR_1` | `AK45-36_kpR_1` | `0.0000 m` |
| `AK45-36_trR_1` | `AK45-36_fpR_1` | `0.0000 m` |

이 때문에 전체 로봇 렌더링에서 회색 actuator/링크가 뭉쳐 보인다.

## 원인 추정

현재 STL 파일들은 link-local 원점 기준으로 정리된 mesh라기보다, CAD assembly/world 좌표가 섞인 상태로 export된 것으로 보인다.

MJCF 안에서는 각 link body가 joint 위치에 배치되어 있지만, visual geom에는 다시 큰 offset이 들어가 있다.

예:

```xml
<body name="thighJ_L_1" pos="0.04 -0.029 0.44">
  <geom pos="-0.04 0.029 -0.44" type="mesh" mesh="thighJ_L_1" />
  <geom pos="-0.04 0.029 -0.44" type="mesh" mesh="AK45-36_tpL_1" />
</body>
```

이런 방식은 neutral assembly 위치를 억지로 맞출 수는 있지만, 각 link가 회전할 때 visual mesh가 link-local geometry처럼 자연스럽게 움직인다는 보장을 주지 않는다.

## 현재 검증에 미치는 영향

동역학 시뮬레이션은 주로 다음 요소에 의해 결정된다.

- body mass/inertia
- joint 위치와 축
- actuator limit
- contact collision geom

현재 visual mesh들은 대부분 `contype="0"`, `conaffinity="0"`이므로 충돌에는 참여하지 않는다. 따라서 visual mesh가 겹쳐 보인다고 해서 그 자체가 contact force를 직접 만든 것은 아니다.

하지만 visual mesh 정렬이 깨졌다는 것은 다음 위험을 의미한다.

1. CAD link frame과 MJCF body frame 대응이 아직 완전히 검증되지 않았다.
2. joint 위치/축도 CAD 기준으로 다시 확인해야 한다.
3. 영상으로 외형을 보고 하드웨어 간섭을 판단하면 안 된다.
4. RL 학습용 최종 모델로 쓰기 전, link-local mesh 또는 simplified visual/collision model을 다시 만들어야 한다.

## 다음 조치

다음 단계는 두 갈래로 나누는 것이 좋다.

1. 동역학 검증용 모델:
   - 현재처럼 단순 contact geom과 mass/inertia 중심으로 사용
   - 대신 body frame, joint axis, actuator 방향을 숫자로 검증

2. 시각 검증용 모델:
   - CAD에서 각 link mesh를 link-local frame 기준으로 다시 export
   - 또는 STEP assembly transform을 사용해서 mesh pose를 정확히 재구성
   - 각 링크에 서로 다른 색을 부여
   - actuator mesh가 실제 joint 위치마다 따로 보이는지 확인

## 현재 판단

사용자가 본 “형체를 알아볼 수 없는 로봇”은 실제로 링크들이 상당히 겹쳐 보이는 상태가 맞다. 다만 이것은 collision/contact link가 모두 물리적으로 겹쳐 충돌한다는 뜻은 아니다. 대부분은 visual mesh frame 문제다.

따라서 66번 standing 실패 영상은 “넘어진다”는 동역학 결과 확인용으로는 유효하지만, “로봇 형상이 실제 CAD와 맞다”는 시각 검증용으로는 아직 유효하지 않다.

