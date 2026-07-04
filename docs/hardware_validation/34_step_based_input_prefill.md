# STEP 기반 입력 초안

## 목적

33번 STEP assembly transform 감사 결과를 바탕으로, 다음 설계 기준 모델을 만들 때 사용할 수 있는 입력 초안을 정리한다. 이 문서는 확정 설계값이 아니라 Fusion 360에서 확인해야 할 값을 줄이기 위한 중간 산출물이다.

## 생성한 파일

- foot contact 초안: `/home/king0519/projects/Humanoid/configs/prefilled/foot_contact_prefilled_from_step_body_centered.json`

## STEP에서 확인된 값

- `foot_L` target origin: `[76.750, -60.000, 25.000] mm`
- `foot_R` target origin: `[-77.250, -60.000, 25.000] mm`
- 좌우 foot target origin 거리: `154.000 mm`
- `footJ_L`와 `footJ_R` target origin 거리도 `154.000 mm`

이 값은 28번 감사에서 MuJoCo foot body가 약 `0.154 m` 떨어져 있던 결과와 일치한다.

## 현재 모델 문제 해석

현재 `URDF_F_link_named.xml`의 좌우 foot body는 분리되어 있다. 하지만 foot visual/collision local offset이 body separation을 상쇄해서 sole collision이 같은 world 위치로 겹친다.

따라서 문제는 `foot_L.stl`/`foot_R.stl` 파일 자체가 좌우 배치를 잃어버렸다는 쪽보다는, URDF/MJCF 안에서 foot mesh와 collision origin을 body-local 기준으로 변환하는 과정에 있을 가능성이 높다.

## 초안 contact 값

진단용 body-centered contact에서 사용한 값은 다음과 같다.

| side | body | center xyz in body m | halfsize xyz m |
| --- | --- | --- | --- |
| left | `foot_L_1` | `[0.0, 0.0, -0.035]` | `[0.035, 0.06, 0.005]` |
| right | `foot_R_v1_1` | `[0.0, 0.0, -0.035]` | `[0.035, 0.06, 0.005]` |

이 값은 최종 발바닥 패드 값이 아니다. 다만 기존처럼 좌우 contact가 한 위치로 겹치는 문제는 피한다.

## Fusion 360에서 확인해야 할 항목

1. `foot_L`/`foot_R` link origin이 ankle roll joint 기준 local frame인지 확인한다.
2. `foot_L.stl`/`foot_R.stl` vertex 좌표가 part-local인지 assembly-absolute인지 확인한다.
3. 실제 발바닥 패드 또는 지면 접촉면의 중심을 foot body local 좌표로 확인한다.
4. 실제 발바닥 패드 또는 지면 접촉면의 halfsize를 확인한다.
5. 좌우 발바닥 중심 사이 거리가 STEP target origin 기준 약 `154 mm`와 일치하는지 확인한다.

## 다음 판단

이제 자동으로 더 밀어붙일 수 있는 부분은 거의 끝났다. 다음 모델 variant를 의미 있게 만들려면 Fusion 360에서 위 항목을 확인하거나, CAD에서 foot link frame과 sole pad 치수를 명시적으로 export해야 한다.

확인값이 들어오면 `URDF_F_link_named.xml`을 덮어쓰지 않고 새 설계 기준 variant를 만들어 PD standing을 다시 실행한다.
