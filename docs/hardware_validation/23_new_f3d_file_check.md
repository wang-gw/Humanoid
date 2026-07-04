# 새 F3D 파일 확인

## 확인 대상

- 파일: `/home/king0519/projects/Humanoid/URDF_F_test.f3d`
- 크기: 약 `632 KB`
- 수정 시각: `2026-07-02 01:16 KST`

## 파일 형식 확인

`file` 명령 기준으로 `URDF_F_test.f3d`는 ZIP archive 기반 파일이다. 내부 구조는 Fusion 360 설계 파일에서 흔히 보이는 형태다.

확인된 내부 항목:

- `FusionAssetName[Active]/`
- `FusionDesignSegmentType1/`
- `FusionBrowserSegmentType1/`
- `FusionACTSegmentType1/`
- `Breps.BlobParts/`
- `Previews/small.png`
- 다수의 `BREP.*.smb`, `BREP.*.smbh`

따라서 이 파일은 단순히 확장자만 바꾼 파일이 아니라, 실제 Fusion 360 CAD 설계 파일로 보인다.

## 현재 환경에서의 제한

이 `.f3d` 파일 내부 payload는 ZIP compression method `93`을 사용한다. 현재 로컬 환경의 Python 기본 `zipfile`과 설치된 기본 도구만으로는 해당 payload를 직접 해제할 수 없다.

따라서 이 환경에서는 `.f3d` 내부의 실제 BREP, preview image, CAD mass property를 직접 추출하지 못했다.

## 검증 흐름에서의 의미

이 파일은 우리가 필요로 하던 CAD 원본 후보로 볼 수 있다. 다만 MuJoCo/RL 검증에 바로 사용할 수 있는 파일은 아니다.

다음 정보는 Fusion 360에서 export하거나 직접 확인해야 한다.

- STEP 또는 SAT 같은 중립 CAD 파일
- link별 STL 또는 mesh
- joint axis와 positive direction
- joint zero pose
- joint limit
- link별 mass
- link별 COM
- link별 inertia tensor
- 실제 standing pose
- 실제 foot contact 치수

## 권장 export

Fusion 360에서 아래 파일을 추가로 export하면 현재 검증 파이프라인에 바로 반영하기 쉽다.

1. `STEP` 또는 `SAT`
   - CAD 구조 확인, link/assembly 검토용
2. link별 `STL`
   - 시각화 mesh와 collision 후보 생성용
3. mass property 표
   - `configs/cad_mass_properties_template.csv` 채우기용
4. joint 정보 표
   - `configs/cad_joint_confirmation_template.csv` 채우기용
5. standing pose 표
   - `configs/standing_pose_template.json` 채우기용
6. foot contact 치수
   - `configs/foot_contact_template.json` 채우기용

## 판단

`URDF_F_test.f3d`는 “맞는 종류의 파일”로 보인다. 하지만 이 파일 하나만으로는 현재 환경에서 자동으로 joint/mass/standing pose를 추출할 수 없다. Fusion 360에서 위 항목들을 export하거나 수치로 확인해 템플릿에 채우는 단계가 필요하다.

