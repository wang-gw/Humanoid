# link STL 기반 named 모델 생성

## 목적

새로 export된 `link/` STL 파일을 기존 모델에 덮어쓰지 않고, 별도 MJCF asset 세트로 구성한다.

## 확정한 매핑

- `AK45-36_trL (1).stl`은 오른쪽 `AK45-36_trR` 역할로 사용한다.
- 오른쪽 foot mesh는 `foot_R.stl`을 사용한다.

## 실행 명령

```bash
python3 scripts/build_link_stl_named_model.py
```

## 산출물

- 새 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_named.xml`
- 리포트 JSON: `/home/king0519/projects/Humanoid/docs/hardware_validation/link_stl_named_model_report.json`

## 컴파일 확인

- nq: `17`
- nv: `16`
- nu: `10`
- joint 수: `11`
- geom 수: `26`

## Mesh 파일 매핑

| mesh name | source STL | copied STL |
| --- | --- | --- |
| `base_link` | `base_link.stl` | `base_link.stl` |
| `thighJR_L_1` | `thighJR_L.stl` | `thighJR_L.stl` |
| `AK45-36_trL_1` | `AK45-36_trL.stl` | `AK45-36_trL.stl` |
| `thigh_L_1` | `thigh_L.stl` | `thigh_L.stl` |
| `AK45-36_kpL_1` | `AK45-36_kpL.stl` | `AK45-36_kpL.stl` |
| `calf_L_1` | `calf_L.stl` | `calf_L.stl` |
| `footJ_L_1` | `footJ_L.stl` | `footJ_L.stl` |
| `AK45-10_frL_1` | `AK45-10_frL.stl` | `AK45-10_frL.stl` |
| `foot_L_1` | `foot_L.stl` | `foot_L.stl` |
| `thighJ_L_1` | `thighJ_L.stl` | `thighJ_L.stl` |
| `AK45-36_tpL_1` | `AK45-36_tpL.stl` | `AK45-36_tpL.stl` |
| `AK45-36_fpL_1` | `AK45-36_fpL.stl` | `AK45-36_fpL.stl` |
| `thighJR_R_1` | `thighJR_R.stl` | `thighJR_R.stl` |
| `AK45-36_trR_1` | `AK45-36_trL (1).stl` | `AK45-36_trR.stl` |
| `thigh_R_1` | `thigh_R.stl` | `thigh_R.stl` |
| `AK45-36_kpR_1` | `AK45-36_kpR.stl` | `AK45-36_kpR.stl` |
| `calf_R_1` | `calf_R.stl` | `calf_R.stl` |
| `AK45-36_fpR_1` | `AK45-36_fpR.stl` | `AK45-36_fpR.stl` |
| `footJ_R_1` | `footJ_R.stl` | `footJ_R.stl` |
| `AK45-10_R_1` | `AK45-10_frR.stl` | `AK45-10_frR.stl` |
| `thighJ_R_1` | `thighJ_R.stl` | `thighJ_R.stl` |
| `AK45-36_tpR_1` | `AK45-36_tpR.stl` | `AK45-36_tpR.stl` |
| `foot_R_v1_1` | `foot_R.stl` | `foot_R.stl` |

## 판단

모델은 MuJoCo에서 컴파일된다. 다만 새 STL은 기존 STL과 로컬 축 방향이 다른 파일들이 있으므로, 시각적 정렬과 contact primitive 위치를 별도로 확인해야 한다.
