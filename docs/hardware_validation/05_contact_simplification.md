# Contact 단순화

## 목적

Neutral standing probe에서 mesh collision 접촉이 과도하게 불안정했다. 이 단계에서는 모든 mesh geom을 visual-only로 바꾸고, 좌우 foot body에 단순 box collision을 추가한다.

## 실행 명령

```bash
python3 scripts/build_contact_simplified_mjcf.py
python3 scripts/check_mujoco_model.py --model envs/robots/urdf_f/URDF_F_contact.xml
python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f/URDF_F_contact.xml --out-dir outputs/analysis/pd_standing_contact
```

## 산출물

- contact MJCF: `/home/king0519/projects/Humanoid/envs/robots/urdf_f/URDF_F_contact.xml`
- 리포트 JSON: `/home/king0519/projects/Humanoid/docs/hardware_validation/contact_simplification_report.json`

## Collision 변경 사항

- 비활성화한 mesh collision geom: `23`
- 추가한 단순 foot box collision:
  - `foot_L_1` pos=`[-0.04769117590785027, 0.0215, -0.04153567083179951]` halfsize=`[0.03302927315235138, 0.03500000014901161, 0.060153983533382416]`
  - `foot_R_v1_1` pos=`[0.10630882409214973, 0.0215, -0.04153567083179951]` halfsize=`[0.03302927315235138, 0.03500000014901161, 0.060153983533382416]`

## 해석

이 모델은 collision 안정성 검증용 중간 모델이다. 최종 구조 검증 전에는 발바닥 실제 접촉면, 고무 패드 크기, 마찰 계수, 충격 흡수 구조를 설계값으로 다시 반영해야 한다.
