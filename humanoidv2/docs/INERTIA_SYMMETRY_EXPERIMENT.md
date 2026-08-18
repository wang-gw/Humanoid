# Humanoidv2 좌우 관성 대칭화 비교 실험

- 실행일: 2026-08-06 (Asia/Seoul)
- 목표: 원본 CAD 관성 비대칭이 V4 좌우 단일지지 차이의 원인인지 분리 검증
- 결론: **관성 대칭화 후 동일 목표·동일 gain에서 좌우 단일지지가 거의 같은 결과로 성공**

## 1. 원본 관성 감사

미러 평면은 로봇 중앙의 `x=0`으로 정의했다. 전체 질량은 8.359301 kg이며 좌우 링크 질량은
거의 같았지만 COM과 전체 관성 텐서는 대칭이 아니었다.

| 항목 | 원본 | 대칭 모델 |
|---|---:|---:|
| 전체 질량 | 8.359301 kg | 8.359301 kg |
| 양발 중앙 대비 중립 COM lateral 편차 | 15.220 mm | 약 0 mm |
| 몸통 COM lateral 편차 | 46.525 mm | 0 mm |
| 최대 링크 COM 미러 오차 | 168.453 mm | 0 mm |
| 최대 링크 관성 미러 상대오차 | 89.63% | 약 0% |
| 몸통 관성 미러 상대오차 | 60.30% | 약 0% |

발 링크 자체와 발바닥 collision은 원래부터 대칭이었다. 관절축 부호는 미러 관절 좌표계의
표현일 수 있으므로 변경하지 않았다.

## 2. 파생 모델 생성 방법

원본 MJCF는 수정하지 않았다. 별도의 생성기가 각 좌우 링크 쌍에 대해 다음 연산을 수행한다.

```text
M = diag(-1, 1, 1)
mass = (mass_left + mass_right) / 2
com_left = (com_left + M * com_right) / 2
com_right = M * com_left
I_left = (I_left + M * I_right * Mᵀ) / 2
I_right = M * I_left * Mᵀ
```

몸통은 COM의 x 성분을 0으로 만들고 `Ixy`, `Ixz`가 0이 되도록 텐서를 미러 평균했다.
고유값 분해로 MuJoCo의 `diaginertia + quat` 표현으로 되돌렸다. 전체 질량과 pair의 전후·높이
방향 1차 모멘트는 보존했다. mesh, kinematics, joint limit, actuator, contact는 변경하지 않았다.

## 3. 정적 양발 지지 비교

동일한 중립 자세, Kp 80/Kd 0.32로 4초간 유지했다.

| 모델 | 왼발 평균하중 | 오른발 평균하중 | 차이 | 최종 COM lateral 편차 |
|---|---:|---:|---:|---:|
| 원본 | 48.659 N | 33.348 N | 15.311 N | 15.494 mm |
| 대칭 관성 | 41.004 N | 41.004 N | 0.000 N | 약 0 mm |

원본에서 관측된 초기 좌우 하중 차이가 관성 COM 편차와 직접 일치하며, 대칭 모델에서는
solver 수치오차 수준으로 사라졌다.

## 4. 동일 단일지지 명령 비교

대칭 모델에서 좌우에 완전히 같은 조건을 적용해 168개 조합을 탐색했다. 좌우 모두 성공한
공통 조합은 8개였고 가장 대칭적인 조건은 다음과 같다.

- lateral 이동: 양쪽 90 mm
- IK 발 높이: 양쪽 35 mm
- 전환 시간: 양쪽 4초
- PD gain: 양쪽 Kp 80/Kd 0.32

| 모델 | 과제 | 마지막 2초 스윙발 하중 | 5 N 미만 비율 | 지지발 하중 | 성공 |
|---|---|---:|---:|---:|---|
| 원본 | 왼발 스윙 | 6.384 N | 4% | 75.622 N | 아니오 |
| 원본 | 오른발 스윙 | 0.000 N | 100% | 82.005 N | 예 |
| 대칭 관성 | 왼발 스윙 | 1.060007 N | 100% | 80.944693 N | 예 |
| 대칭 관성 | 오른발 스윙 | 1.060024 N | 100% | 80.944677 N | 예 |

대칭 모델의 좌우 스윙 하중 차이는 0.000017 N, 최대 발 높이 차이는 0.000029 mm였다. 따라서
기존에 필요했던 60/120 mm lateral 목표와 서로 다른 gain은 주로 관성 비대칭을 보상한 것으로
판단할 수 있다.

## 5. 대칭 모델 PPO 결과

원본 V4와 같은 PPO 구조, seed 7, 병렬 환경 4개로 학습했다. 요청 50,000 steps는 PPO rollout
단위 때문에 실제 57,344 steps가 실행됐으며 양쪽 최저 성능 기준으로 50,000-step checkpoint를
선택했다.

| 과제 | 생존 | 마지막 2초 스윙발 하중 | 지지발 하중 | 최대 발바닥 높이 | 성공 |
|---|---:|---:|---:|---:|---|
| 왼발 스윙 | 8.0초 | 0.000 N | 82.001 N | 21.35 mm | 예 |
| 오른발 스윙 | 8.0초 | 0.000 N | 82.005 N | 18.02 mm | 예 |

정책 없는 기준운동은 거의 완전한 좌우대칭이지만, 일반 MLP PPO는 미러 equivariance를
강제하지 않으므로 학습 정책의 발 높이에는 약 3.3 mm 차이가 남았다. 다음 학습에서는
observation/action mirror augmentation 또는 shared mirrored policy를 검토할 가치가 있다.

## 6. 해석상 한계

이 결과는 원본 CAD 관성 값이 실제 로봇보다 정확하다는 뜻도, 대칭 모델이 실제 로봇을 더
정확히 나타낸다는 뜻도 아니다. 대칭 모델은 제어 알고리즘을 검증하기 위한 가설 모델이다.
실물로 이전하기 전에는 링크별 질량, 조립 상태 COM, 모터·배터리 배치와 전체 pendulum/CAD
관성을 측정해 원본 또는 대칭 모델 중 어느 쪽이 실제에 가까운지 결정해야 한다.

또한 이번 비교는 정적 단일지지까지다. 동적 착지, 좌우 전환 관성, 외란, 연속 보행은 아직
검증하지 않았다. 다만 다음 단일-step 학습의 기준 모델로는 대칭 관성 버전을 사용하는 것이
타당하다.

## 7. 산출물

- 대칭 MJCF: [`URDF_F_v2_footprint_contact_symmetric_inertia.xml`](../models/urdf_f_v2/URDF_F_v2_footprint_contact_symmetric_inertia.xml)
- 모델 생성기: [`build_symmetric_inertia_model.py`](../scripts/build_symmetric_inertia_model.py)
- 대칭 감사 도구: [`audit_model_symmetry.py`](../scripts/audit_model_symmetry.py)
- 물리 비교: [`dynamics_comparison.json`](../results/inertia_symmetry/dynamics_comparison.json)
- 원본 감사: [`original_audit.json`](../results/inertia_symmetry/original_audit.json)
- 대칭 감사: [`symmetric_audit.json`](../results/inertia_symmetry/symmetric_audit.json)
- 원본/대칭 왼발 비교 영상: [`left_task_original_vs_symmetric.mp4`](../results/inertia_symmetry/left_task_original_vs_symmetric.mp4)
- 대칭 PPO 왼발 영상: [`single_support_left_trained.mp4`](../results/khr3hv_v4_symmetric/single_support_left_trained.mp4)
- 대칭 PPO 오른발 영상: [`single_support_right_trained.mp4`](../results/khr3hv_v4_symmetric/single_support_right_trained.mp4)
- 대칭 최종 정책: [`ppo_single_support_final.zip`](../results/khr3hv_v4_symmetric/ppo_single_support_final.zip)

## 8. 재실행

```bash
cd /home/king0519/projects/Humanoidv2
python3 scripts/build_symmetric_inertia_model.py
python3 scripts/audit_model_symmetry.py \
  --model models/urdf_f_v2/URDF_F_v2_footprint_contact_symmetric_inertia.xml
python3 scripts/compare_inertia_symmetry.py
python3 scripts/train_single_support_v4.py --timesteps 50000 --n-envs 4 \
  --task-profile symmetric \
  --robot-model models/urdf_f_v2/URDF_F_v2_footprint_contact_symmetric_inertia.xml
python3 scripts/evaluate_single_support_v4.py --side left --task-profile symmetric \
  --robot-model models/urdf_f_v2/URDF_F_v2_footprint_contact_symmetric_inertia.xml
python3 scripts/evaluate_single_support_v4.py --side right --task-profile symmetric \
  --robot-model models/urdf_f_v2/URDF_F_v2_footprint_contact_symmetric_inertia.xml
```
