# KHR-3HV 방식 단일지지 V4 실험 기록

- 실행일: 2026-08-05 (Asia/Seoul)
- 목표: 연속 보행 전에 좌우 정적 단일지지 전환과 4초 유지를 각각 성공
- 최종 판정: **좌우 모두 8초 생존 및 단일지지 성공**

## 1. 이 단계를 따로 만든 이유

V3.4 정책은 20초 동안 넘어지지 않았지만 양발 접촉률이 100%였다. 순전진도 7.40 cm에
그쳐, 전진 보상을 더 학습하기 전에 보행의 필수 전제인 체중 이동과 스윙발 이탈을 독립된
과제로 검증했다. V4는 연속 보행 성공을 주장하는 실험이 아니라, 다음 보행 학습의 초기
자세와 teacher를 만드는 하위 과제다.

## 2. 기준 자세 탐색

수정된 collision 물리에서 IK 목표와 PD gain을 직접 탐색했다. 공통 탐색 324개와 왼발
확장 탐색 36개, 총 360개 조합을 평가했다. 각 후보는 2–4초 동안 smoothstep으로 목표
자세에 전환한 뒤 4초 동안 유지했다.

| 스윙발 | lateral 이동 | IK 발 높이 | 전환 | PD Kp/Kd | 마지막 2초 스윙발 하중 | 지지발 하중 |
|---|---:|---:|---:|---:|---:|---:|
| 오른발 | 60 mm | 35 mm | 4초 | 50 / 0.20 | 0.765 N | 81.239 N |
| 왼발 | 120 mm | 55 mm | 4초 | 80 / 0.32 | 2.477 N | 79.528 N |

좌우 수치가 대칭이 아닌 것은 현재 CAD 관절축·질량·foot frame에서 같은 부호와 gain이
동일한 동역학을 만들지 않기 때문이다. 이를 억지로 대칭화하지 않고 실제 시뮬레이션에서
검증된 두 목표를 각각 사용했다.

## 3. V4 환경과 성공 조건

- 에피소드: 200 control steps, 8초
- 전환: 처음 100 steps(4초) smoothstep
- 유지: 이후 100 steps(4초)
- reset마다 왼발/오른발 스윙 과제를 무작위 선택
- observation: 기존 5-frame 센서 history 80차원 + 요청 측 부호 + 전환 진행률 = 82차원
- action: 10개 관절 기준 자세에 더하는 저폭 residual
- 제어: 오른발/왼발에 탐색된 side-specific PD gain 적용
- 보상: 스윙발 unload, 지지발 하중, 몸통 자세, 목표 자세 추종, 생존, torque, 낙상

성공은 8초 동안 낙상하지 않고 마지막 2초에 아래 조건을 모두 만족하는 것으로 정의했다.

1. 스윙발 평균 수직 접촉력 5 N 미만
2. 샘플의 90% 이상에서 스윙발 접촉력 5 N 미만
3. 마지막 지지발 접촉력 30 N 초과

## 4. PPO 학습과 체크포인트 선택

- PPO MLP: policy/value 각각 64×64, Tanh
- seed: 7
- 병렬 환경: 4개
- 최초 계획: 300,000 steps
- 실제 실행: 100,000 steps까지 검증 후 조기 종료

무작위 좌우 에피소드의 최근 평균 보상은 25k/50k/75k/100k에서 각각
123.245/128.812/137.454/140.248로 증가했다. 그러나 좌우를 분리해 평가하자 평균 보상이
정책의 한쪽 붕괴를 가린다는 사실이 확인됐다.

| checkpoint | 오른발 과제 | 왼발 과제 | 판정 |
|---|---:|---:|---|
| 50,000 steps | 200/200, 성공 | 200/200, 성공 | **최종 선택** |
| 100,000 steps | 200/200, 성공 | 116/200, 낙상 | 제외 |

따라서 최종 모델은 50,000-step checkpoint다. 재실행용 학습 스크립트도 양쪽 성공 수를
우선하고, 동률이면 두 쪽 중 낮은 보상을 최대화하는 규칙으로 최종 모델을 자동 선택한다.

## 5. 최종 결과

| 모드 | 스윙발 | 생존 | 마지막 2초 스윙발 하중 | 5 N 미만 비율 | 지지발 하중 | 최대 발바닥 높이 | 성공 |
|---|---|---:|---:|---:|---:|---:|---|
| 기준 자세 | 오른발 | 8.0초 | 0.765 N | 100% | 81.239 N | 5.1 mm | 예 |
| 기준 자세 | 왼발 | 8.0초 | 2.477 N | 100% | 79.528 N | 4.7 mm | 예 |
| PPO 50k | 오른발 | 8.0초 | **0.000 N** | 100% | 82.005 N | 10.8 mm | 예 |
| PPO 50k | 왼발 | 8.0초 | **0.000 N** | 100% | 82.005 N | 13.3 mm | 예 |

PPO 정책은 기준 자세보다 양쪽 스윙발을 더 높이 들고 마지막 2초 동안 접촉력을 완전히
제거했다. 생성한 최종 프레임도 한 발이 바닥에서 떨어진 상태임을 확인했다.

## 6. 한계와 다음 단계

이번 성공은 고정 바닥·고정 마찰·고정 초기 자세에서의 정적 단일지지다. 외란 강건성,
좌우 전환 순간의 관성, 발 착지 충격, 전진 스텝, 연속 보행은 아직 검증하지 않았다.
실물 적용 성공을 뜻하지도 않는다.

다음 버전에서는 다음 순서가 적절하다.

1. V4 오른발 지지와 왼발 지지 상태를 reset/teacher 상태로 사용
2. 양발 지지 → 오른발 스윙 → 양발 착지의 단일 step 전환 학습
3. 반대쪽 단일 step을 별도 평가해 한쪽 붕괴 방지
4. 좌우 두 step 성공 후 짧은 2-step episode로 연결
5. 그 다음에만 KHR 전진 속도 보상과 주기적 연속 보행을 다시 적용

## 7. 산출물

- 오른발 PPO 영상: [`single_support_right_trained.mp4`](../results/khr3hv_v4/single_support_right_trained.mp4)
- 왼발 PPO 영상: [`single_support_left_trained.mp4`](../results/khr3hv_v4/single_support_left_trained.mp4)
- 오른발 기준 영상: [`single_support_right_reference.mp4`](../results/khr3hv_v4/single_support_right_reference.mp4)
- 왼발 기준 영상: [`single_support_left_reference.mp4`](../results/khr3hv_v4/single_support_left_reference.mp4)
- 선택 정책: [`ppo_single_support_final.zip`](../results/khr3hv_v4/ppo_single_support_final.zip)
- 체크포인트 비교: [`checkpoint_comparison.json`](../results/khr3hv_v4/checkpoint_comparison.json)
- 탐색 전체 결과: [`single_support_search.json`](../results/khr3hv_v4/single_support_search.json)
- 학습 요약: [`training_summary.json`](../results/khr3hv_v4/training_summary.json)

## 8. 재실행

```bash
cd /home/king0519/projects/Humanoidv2
python3 -m pytest -q
python3 scripts/search_single_support_reference.py
python3 scripts/train_single_support_v4.py --timesteps 100000 --n-envs 4
python3 scripts/evaluate_single_support_v4.py --side right
python3 scripts/evaluate_single_support_v4.py --side left
```
