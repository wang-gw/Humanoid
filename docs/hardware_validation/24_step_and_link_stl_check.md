# STEP와 link STL 확인

## 확인 대상

- STEP: `/home/king0519/projects/Humanoid/URDF_F_.step`
- link STL 폴더: `/home/king0519/projects/Humanoid/link`

## STEP 파일

- 크기: `240483` bytes
- schema/entity 기준: `{'ITEM_DEFINED_TRANSFORMATION': 23, 'SHAPE_REPRESENTATION': 116, 'PRODUCT': 24, 'NEXT_ASSEMBLY_USAGE_OCCURRENCE': 23, 'AXIS2_PLACEMENT_3D': 311, 'CARTESIAN_POINT': 743}`

STEP 헤더상 Autodesk Translation Framework에서 생성된 ASCII STEP 파일이다. 즉 Fusion 360에서 export한 중립 CAD 파일로 보인다.

## STL 파일

- 새 link STL 수: `23`
- 기존 URDF mesh STL 수: `24`
- 기존 mesh와 이름 매칭된 새 STL 수: `23`
- 기존 mesh와 이름 매칭되지 않은 새 STL 수: `0`

## 주의할 점

- 새 STL 이름은 기존 URDF mesh 이름보다 정리되어 있지만, 일부 이름은 그대로 매칭되지 않는다.
- `AK45-36_trL.stl`과 `AK45-36_trL (1).stl`이 동시에 존재한다. 사용자 확인 결과 `AK45-36_trL (1).stl`은 오른쪽 `trR`로 사용한다.
- 기존에는 오른쪽 ankle actuator가 `AK45-10_R_1.stl`였고, 새 파일은 `AK45-10_frR.stl`이다. 이름은 더 일관적이지만 URDF reference를 업데이트해야 한다.
- 기존 mesh 기준으로 `AK45-36_trR_1.stl`에 대응되는 파일은 `AK45-36_trL (1).stl`로 확정한다.
- 기존 mesh에는 `foot_R_1.stl`과 `foot_R_v1_1.stl`이 모두 있었지만, 새 link 폴더에는 `foot_R.stl` 하나만 있다. 사용자 확인 결과 오른쪽 foot mesh는 `foot_R.stl`을 사용한다.

## 중복/확인 필요 파일

- canonical `AK45-36_trL`: `['AK45-36_trL (1).stl', 'AK45-36_trL.stl']`

## 기존 mesh 기준으로 이름이 달라진 항목

- `AK45-36_trR_1.stl`: 새 link 폴더의 `AK45-36_trL (1).stl`을 사용한다.
- `foot_R_v1_1.stl`: 새 link 폴더의 `foot_R.stl`을 사용한다.

## 산출물

- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/new_cad_files/new_cad_files_audit.json`
- STL bounds CSV: `/home/king0519/projects/Humanoid/outputs/analysis/new_cad_files/link_stl_bounds.csv`
- STL 비교 CSV: `/home/king0519/projects/Humanoid/outputs/analysis/new_cad_files/stl_name_comparison.csv`

## 판단

새 STEP와 STL 파일들은 검증에 필요한 CAD export로 보인다. 확인된 파일명 매핑은 별도 모델 `envs/robots/urdf_f_link/URDF_F_link_named.xml`에 반영했다. 기존 모델은 덮어쓰지 않는다.
