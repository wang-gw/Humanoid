# KHR-3HV 방식 V5 단일 전진 스텝 실험

- 실행일: 2026-08-06 (Asia/Seoul)
- 모델: 좌우 대칭 관성 Humanoidv2
- 목표: 양발 지지에서 한 발을 들어 앞으로 옮기고 다시 양발로 착지
- 최종 판정: **왼발·오른발 단일 전진 스텝 모두 성공**

## 1. 과제 구성

V4는 한 발을 든 정적 단일지지까지 성공했다. V5는 이를 다음 10초 접촉 순서로 확장했다.

| 단계 | 시간 | 동작 |
|---|---:|---|
| lift | 0–4초 | 지지발 쪽으로 COM 이동, 스윙발 들기 |
| advance | 4–6초 | 단일지지를 유지하며 스윙발 전진 |
| land | 6–8초 | lateral 이동을 되돌리며 앞쪽에 착지 |
| settle | 8–10초 | 앞뒤로 벌어진 양발 지지 자세 유지 |

좌우 모두 같은 90 mm lateral 이동, 30 mm IK lift, Kp 80/Kd 0.32를 사용한다. 전진 IK
목표는 30 mm이며 몸통을 두 발 사이에 두기 위해 지지발/스윙발 목표를 각각 보폭 절반씩
뒤/앞으로 이동시켰다.

성공 조건은 다음을 모두 만족해야 한다.

1. 10초 동안 낙상하지 않음
2. advance 50 steps의 평균 스윙발 하중 5 N 미만
3. advance 샘플의 90% 이상이 5 N 미만
4. settle에서 양발 각각 10 N 초과인 비율 90% 이상
5. 최종 스윙발이 지지발보다 20 mm 이상 앞에 있음

## 2. 기준궤적 탐색

보폭 10–50 mm, Kp 50/65/80, 착지 50/75/100 steps와 좌우를 조합한 90개 평가 중
12개가 성공했고, 동일 parameter로 좌우가 모두 성공한 조합은 6개였다. 의미 있는 보폭과
짧은 착지 시간을 우선해 `30 mm / Kp 80 / 50 steps`를 선택했다.

| 기준운동 | 스윙 구간 하중 | 5 N 미만 | 최종 발 간격 | 몸통 전진 | 착지 peak | 양발 접촉 |
|---|---:|---:|---:|---:|---:|---:|
| 왼발 step | 2.443 N | 100% | 27.06 mm | 18.03 mm | 42.11 N | 100% |
| 오른발 step | 1.635 N | 100% | 28.40 mm | 18.98 mm | 41.81 N | 100% |

정책이 없어도 접촉 순서를 완주하므로 PPO가 실패할 경우 보상이나 탐색 문제와 기준궤적
문제를 분리할 수 있다.

## 3. V5 PPO

- MLP policy/value: 각각 64×64, Tanh
- observation: 센서 history 80 + 스윙 측 부호 + 전체 진행률
- action: 10개 관절 기준궤적에 추가하는 작은 residual
- seed: 7
- 병렬 환경: 4개
- 요청 학습량: 50,000 steps
- 실제 학습량: PPO rollout 단위로 57,344 steps
- 실행 시간: 134.04초

25k, 50k, 최종 57,344-step 후보가 모두 좌우 성공했다. 양쪽 중 낮은 보상을 최대화하는
규칙으로 최종 rollout을 선택했다.

| checkpoint | 왼발 | 오른발 | 낮은 쪽 보상 |
|---|---:|---:|---:|
| 25k | 성공, 149.784 | 성공, 148.562 | 148.562 |
| 50k | 성공, 150.654 | 성공, 149.406 | 149.406 |
| 57,344 | 성공, 150.229 | 성공, 149.920 | **149.920** |

## 4. 최종 정책 결과

| 정책 | 생존 | 스윙 구간 하중 | 5 N 미만 | 최종 발 간격 | 최대 높이 | 최종 양발 하중 | 착지 peak | 성공 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 왼발 step | 10초 | 0.423 N | 96% | 31.72 mm | 23.16 mm | 39.67/42.33 N | 42.46 N | 예 |
| 오른발 step | 10초 | 0.332 N | 98% | 31.97 mm | 19.55 mm | 40.48/41.52 N | 42.66 N | 예 |

PPO는 기준운동보다 발을 약 13–16 mm 더 높이 들어 스윙 중 접촉을 줄였고, 최종 발 간격을
약 32 mm로 만들었다. 착지 후 양발 접촉은 settle 구간 전체에서 유지됐다. 몸통도 좌우 각각
22.73/23.08 mm 전진했다.

## 5. 한계와 다음 연결 조건

V5는 실제 한 번의 전진 스텝이지만 아직 연속 보행은 아니다. 매 episode가 같은 양발 평행
자세에서 시작하고, 첫 발을 내디딘 뒤 두 번째 발을 이어서 옮기지 않는다. 외란, 마찰 변화,
초기 자세 변화와 실물 torque-speed 곡선도 적용하지 않았다.

다음 V6는 V5의 최종 split stance를 그대로 초기 상태로 사용해야 한다.

1. 첫발 착지 상태에서 반대발로 하중 이동
2. 뒤쪽 발을 들어 앞발 앞으로 이동
3. 두 번째 착지 후 최소 1초 양발 안정화
4. 좌→우와 우→좌 두 순서를 각각 평가
5. 2-step 성공 후에만 episode를 반복 주기로 연결

연속 보행 보상은 이 2-step 전환을 성공시킨 뒤 추가하는 것이 적절하다.

## 6. 산출물

- 좌우 비교 영상: [`single_step_left_vs_right_trained.mp4`](../results/khr3hv_v5_symmetric/single_step_left_vs_right_trained.mp4)
- 왼발 PPO 영상: [`single_step_left_trained.mp4`](../results/khr3hv_v5_symmetric/single_step_left_trained.mp4)
- 오른발 PPO 영상: [`single_step_right_trained.mp4`](../results/khr3hv_v5_symmetric/single_step_right_trained.mp4)
- 왼발 기준 영상: [`single_step_left_reference.mp4`](../results/khr3hv_v5_symmetric/single_step_left_reference.mp4)
- 오른발 기준 영상: [`single_step_right_reference.mp4`](../results/khr3hv_v5_symmetric/single_step_right_reference.mp4)
- 최종 정책: [`ppo_single_step_final.zip`](../results/khr3hv_v5_symmetric/ppo_single_step_final.zip)
- 탐색 결과: [`reference_search_summary.json`](../results/khr3hv_v5_symmetric/reference_search_summary.json)
- 체크포인트 비교: [`checkpoint_comparison.json`](../results/khr3hv_v5_symmetric/checkpoint_comparison.json)
- 환경 코드: [`single_step_v5_env.py`](../humanoidv2/single_step_v5_env.py)

## 7. 재실행

```bash
cd /home/king0519/projects/Humanoidv2
python3 -m pytest -q
python3 scripts/search_single_step_v5_reference.py
python3 scripts/train_single_step_v5.py --timesteps 50000 --n-envs 4
python3 scripts/evaluate_single_step_v5.py --side left
python3 scripts/evaluate_single_step_v5.py --side right
```
