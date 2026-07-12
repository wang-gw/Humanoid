# 04. 준정적 서있는 자세 탐색 + 안정 스탠딩 검증

## 목적

새 형상은 구모델 자세각이 맞지 않으므로(base_link 원점이 골반이 아니라 바닥 근처,
발 방향도 다름) RL 시드용 정적 균형 자세를 새로 탐색한다.

## 방법: CEM 준정적 탐색

구모델 `search_quasistatic_standing_pose.py`가 `_sole_collision` geom을 찾는데 신모델은
`_sole_pad_*` 4패드 구조라 호환되지 않는다. 발마다 패드를 집계해 지지면을 계산하도록
적응한 v2 전용 스크립트를 작성했다. CEM으로 관절 10각을 최적화해 전신 CoM이 양발
지지면 중심에 오고 양 발바닥이 평평·수평이 되게 한다.

## 실행 명령

```bash
python3 scripts/search_quasistatic_standing_pose_v2.py \
  --left-foot foot_L_1 --right-foot foot_R_1 \
  --out configs/urdf_f_v2/quasistatic_standing_pose.json
```

## 주요 수치 (탐색 결과)

| 항목 | 값 |
|---|---:|
| cost | 0.057 (수렴) |
| base_z | 0.0278 |
| 지지 여유 (x_min, x_max, y_min, y_max) | +0.139, +0.129, +0.068, +0.069 (전부 양수) |
| 관절 norm | 0.21 (작은 각 = 자연 자세) |
| 발 높이차 | 1.4 mm |

## 검증: PD 하에서 안정 스탠딩

| 조건 | 결과 |
|---|---|
| zero-action, 기본 설정, 1000스텝 | **1000/1000 생존 (50초)**, 양발 접촉 |
| 안정화기 부호 4조합 전부 | 400/400 생존 |
| 안정화기 없음 | 400/400 생존 |

→ `03`의 armature 수정 덕에 자세가 동역학적으로도 안정. 구모델과 달리 안정화기 튜닝
없이도 선다.

## 산출물

| 파일 | 설명 |
|---|---|
| `configs/urdf_f_v2/quasistatic_standing_pose.json` | 준정적 균형 자세 (base_z + 관절 10각) |
| `scripts/search_quasistatic_standing_pose_v2.py` | v2 패드 집계 방식 포즈 탐색기 |

## 판단

- 정적 균형 + PD 안정 스탠딩 확보 → RL 학습 시드로 사용 가능.

## 다음 조치

→ `05_standing_rl_and_walking_attempt.md`: 이 시드로 보행 정책 학습.
