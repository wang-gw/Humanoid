# Body-centered foot contact 진단 모델

## 목적

28번 감사에서 좌우 foot body는 떨어져 있지만 sole collision이 같은 world 위치로 겹치는 문제가 확인됐다. 이 모델은 실제 설계 확정 모델이 아니라, foot body 중심에 발바닥 box를 두면 support polygon과 standing probe가 어떻게 달라지는지 확인하기 위한 진단용 variant다.

## 실행 명령

```bash
python3 scripts/build_body_centered_foot_contact.py
```

## 산출물

- 진단 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_body_contact.xml`
- JSON: `/home/king0519/projects/Humanoid/docs/hardware_validation/body_centered_foot_contact_report.json`

## Contact 설정

- 기존 non-floor geom은 모두 visual-only/contact-off로 둔다.
- 기존 sole collision geom은 제거한다.
- 좌우 foot body에 같은 local contact box를 추가한다.

| body | geom | local pos | halfsize |
| --- | --- | --- | --- |
| `foot_L_1` | `foot_L_1_sole_collision` | `0 0 -0.035` | `0.035 0.06 0.005` |
| `foot_R_v1_1` | `foot_R_v1_1_sole_collision` | `0 0 -0.035` | `0.035 0.06 0.005` |

## 컴파일 확인

- nq: `17`
- nv: `16`
- nu: `10`
- joint 수: `11`
- geom 수: `26`

## 주의

이 contact 위치는 CAD 확정값이 아니다. 실제 발바닥 패드/바닥 접촉면 치수와 foot link origin이 확인되면 새 설계 기준 모델을 따로 만들어야 한다.
