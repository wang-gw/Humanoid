# STEP 기반 foot contact config 적용

## 목적

foot contact 값을 코드에 하드코딩하지 않고 JSON config에서 읽어 새 MJCF variant를 생성한다. 앞으로 Fusion 360에서 확정한 발바닥 값을 받으면 같은 스크립트로 새 모델을 만들 수 있다.

## 실행 명령

```bash
python3 scripts/apply_foot_contact_config.py --source envs/robots/urdf_f_link/URDF_F_link_named.xml --config configs/prefilled/foot_contact_prefilled_from_step_body_centered.json --out envs/robots/urdf_f_link/URDF_F_link_step_contact.xml
```

## 산출물

- 새 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_contact.xml`
- 사용 config: `/home/king0519/projects/Humanoid/configs/foot_contact_user_size_mjcf_body_centered.json`
- 리포트 JSON: `/home/king0519/projects/Humanoid/docs/hardware_validation/foot_contact_config_application_report.json`

## 적용한 contact

| side | body | geom | center xyz in body | halfsize xyz |
| --- | --- | --- | --- | --- |
| `left` | `foot_L_1` | `foot_L_1_sole_collision` | `[0.0, 0.0, -0.02]` | `[0.035, 0.06, 0.02]` |
| `right` | `foot_R_v1_1` | `foot_R_v1_1_sole_collision` | `[0.0, 0.0, -0.02]` | `[0.035, 0.06, 0.02]` |

## 컴파일 확인

- nq: `17`
- nv: `16`
- nu: `10`
- joint 수: `11`
- geom 수: `26`

## 판단

이 모델은 STEP 기반 초안 contact를 적용한 반복 검증용 variant다. 아직 최종 설계 모델은 아니며, Fusion 360에서 실제 발바닥 중심/크기와 standing pose가 확인되면 config 값을 교체해 다시 생성해야 한다.
