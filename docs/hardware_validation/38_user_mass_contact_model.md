# 사용자 제공 Mass/Contact 적용 모델

## 목적

사용자가 제공한 mass property와 foot contact box를 반영한 새 MJCF variant를 만든다. 기존 모델은 덮어쓰지 않는다.

## 실행 명령

```bash
python3 scripts/apply_body_mass_config.py --source envs/robots/urdf_f_link/URDF_F_link_user_contact.xml --mass-csv configs/cad_body_mass_aggregate_user.csv --out envs/robots/urdf_f_link/URDF_F_link_user_mass_contact.xml
```

## 산출물

- 새 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml`
- mass CSV: `/home/king0519/projects/Humanoid/configs/cad_body_mass_aggregate_user.csv`
- 리포트 JSON: `/home/king0519/projects/Humanoid/docs/hardware_validation/user_mass_application_report.json`

## 질량 요약

- 기존 MJCF body mass 합계: `8.623980 kg`
- 사용자 제공 CAD 합산 mass: `9.211400 kg`
- MuJoCo compile 후 body mass 합계: `9.211400 kg`

## Body별 적용값

| body | 기존 kg | 적용 kg | ratio | components |
| --- | ---: | ---: | ---: | --- |
| `base_link` | 2.800000 | 2.921400 | 1.043357 | `base_link + thighJR_L + AK45-36_trL + thighJR_R + AK45-36_trR` |
| `thighJ_L_1` | 0.400000 | 0.469700 | 1.174250 | `thighJ_L + AK45-36_tpL` |
| `thigh_L_1` | 0.900000 | 0.911300 | 1.012556 | `thigh_L + AK45-36_kpL` |
| `calf_L_1` | 0.900000 | 1.094500 | 1.216111 | `calf_L + AK45-36_fpL` |
| `footJ_L_1` | 0.300000 | 0.348200 | 1.160667 | `footJ_L + AK45-10_frL` |
| `foot_L_1` | 0.300000 | 0.321300 | 1.071000 | `foot_L` |
| `thighJ_R_1` | 0.400000 | 0.469700 | 1.174250 | `thighJ_R + AK45-36_tpR` |
| `thigh_R_1` | 0.920360 | 0.911300 | 0.990156 | `thigh_R + AK45-36_kpR` |
| `calf_R_1` | 1.103620 | 1.094500 | 0.991736 | `calf_R + AK45-36_fpR` |
| `footJ_R_1` | 0.300000 | 0.348200 | 1.160667 | `footJ_R + AK45-10_frR` |
| `foot_R_v1_1` | 0.300000 | 0.321300 | 1.071000 | `foot_R` |

## 주의

제공된 CG는 각 컴포넌트 local frame 기준으로 보인다. 현재 MJCF는 rigid component를 MuJoCo body 하나로 합친 구조이므로, 정확한 aggregate COM/inertia를 만들려면 각 컴포넌트 CG를 해당 body frame으로 변환해야 한다.

따라서 이번 variant에서는 mass를 CAD 합산값으로 교체하고, 기존 diagonal inertia를 mass ratio로 스케일했다. 최종 설계 검증 전에는 aggregate COM/inertia 계산을 별도 확정해야 한다.
