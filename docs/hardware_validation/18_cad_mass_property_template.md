# CAD Mass Property 확인 템플릿

## 목적

현재 MJCF의 mass/inertia 값이 CAD 설계값과 일치하는지 확인하기 위한 템플릿이다. standing 실패가 질량 배치나 좌우 비대칭에서 오는지 분리하려면 CAD 기준 mass property가 필요하다.

## 템플릿 파일

- `/home/king0519/projects/Humanoid/configs/cad_mass_properties_template.csv`

## 사용 방법

CAD에서 각 body/link의 질량, COM, inertia tensor를 확인한 뒤 템플릿 CSV에 채운다.

특히 아래 body는 현재 MJCF에서 좌우 질량 차이가 있으므로 우선 확인한다.

- `thigh_L_1` vs `thigh_R_1`
- `calf_L_1` vs `calf_R_1`

## 다음 단계

CAD mass property가 채워지면 MJCF mass/inertia와 비교하고, 차이가 의도된 설계인지 export/변환 오류인지 판단한다.

