# Component Inertia Experimental Variant

## 목적

사용자가 제공한 component별 full inertia를 실험용 MJCF variant에 적용한다. 단, frame 불확실성이 가장 큰 `base_link`는 유지한다.

## 실행 명령

```bash
python3 scripts/apply_component_inertia_config.py
```

## 산출물

- 새 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml`
- 적용 리포트: `/home/king0519/projects/Humanoid/docs/hardware_validation/component_inertia_application_report.json`

## 적용 방식

- component별 inertia는 해당 body frame 기준이라고 가정하고 body별로 합산했다.
- 합산에는 parallel-axis theorem을 사용했다.
- MJCF에는 `fullinertia`로 기록했다.
- `base_link`는 기존 inertial을 유지했다.

## Compile 결과

- total mass: `9.211400 kg`
- nq: `17`
- nv: `16`
- nu: `10`

## 적용 Body

| body | 상태 | components |
| --- | --- | --- |
| `base_link` | `skipped` | `` |
| `thighJ_L_1` | `applied` | `thighJ_L, AK45-36_tpL` |
| `thigh_L_1` | `applied` | `thigh_L, AK45-36_kpL` |
| `calf_L_1` | `applied` | `calf_L, AK45-36_fpL` |
| `footJ_L_1` | `applied` | `footJ_L, AK45-10_frL` |
| `foot_L_1` | `applied` | `foot_L` |
| `thighJ_R_1` | `applied` | `thighJ_R, AK45-36_tpR` |
| `thigh_R_1` | `applied` | `thigh_R, AK45-36_kpR` |
| `calf_R_1` | `applied` | `calf_R, AK45-36_fpR` |
| `footJ_R_1` | `applied` | `footJ_R, AK45-10_frR` |
| `foot_R_v1_1` | `applied` | `foot_R` |

## 판단

이 모델은 최종 설계 검증 모델이 아니라 frame 가정 검증용 실험 모델이다. 다음 단계에서 geometry sanity check와 neutral PD standing probe를 기존 모델과 비교해, inertia 적용이 물리적으로 납득되는 방향인지 확인한다.
