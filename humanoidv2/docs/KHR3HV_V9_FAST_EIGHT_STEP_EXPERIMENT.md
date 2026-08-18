# KHR-3HV 방식 V9 고속 전방 8스텝 실험

- 실행일: 2026-08-06 (Asia/Seoul)
- 모델: 좌우 대칭 관성 Humanoidv2
- 목표: V8의 전방 8스텝을 유지하며 step 시간을 10초에서 6초로 단축
- 최종 판정: **좌측·우측 시작 모두 6초/step 엄격 성공**
- 최종 정책: 57,344-step PPO

## 1. 시간 압축

V8은 한 step에 10초가 걸렸다. V9는 같은 접촉 순서와 하이브리드 관절 재기준화를
유지하면서 phase 시간을 다음과 같이 줄였다.

| phase | V8 | V9 |
|---|---:|---:|
| lift | 4.0초 / 100 steps | 2.4초 / 60 steps |
| advance | 2.0초 / 50 steps | 1.2초 / 30 steps |
| land | 2.0초 / 50 steps | 1.2초 / 30 steps |
| settle | 2.0초 / 50 steps | 1.2초 / 30 steps |
| step 합계 | 10.0초 / 250 steps | **6.0초 / 150 steps** |
| 8스텝 합계 | 80초 / 2,000 steps | **48초 / 1,200 steps** |

## 2. 기준운동 속도 탐색

같은 30 mm 보폭, 90 mm sway, Kp 80에서 8초·6초·4초/step을 양쪽 시작으로
평가했다.

| 시간 | 양쪽 중 엄격 성공 | 결과 |
|---|---:|---|
| 8초/step | 2/2 | 기준궤적만으로 성공 |
| 6초/step | 0/2 | 낙상 없음, 접촉 표본 조건만 미달 |
| 4초/step | 0/2 | 스윙 접촉 전환이 더 거침 |

6초 조건에서 lift 30/35 mm, sway 90/100 mm, Kp 80/85와 양쪽을 조합한 16개
공간 탐색도 동일 파라미터 양쪽 성공은 없었다. 한쪽 접촉을 개선하면 반대쪽 보폭이 20 mm
아래로 내려가는 경향이 있었다.

선택한 기본 6초 기준은 엄격 성공은 아니지만 학습 가능한 상태였다.

| 시작 | 생존 | 전진 | 최대 평균 swing 하중 | 최소 5 N 미만 표본 | 최소 보폭 | 착지 접촉 |
|---|---:|---:|---:|---:|---:|---:|
| 왼발 | 48초 | 238.61 mm | 2.685 N | 83.3% | 28.93 mm | 모두 100% |
| 오른발 | 48초 | 235.90 mm | 2.418 N | 86.7% | 28.56 mm | 모두 100% |

평균 접촉력은 5 N보다 낮지만 advance 시작의 몇 개 샘플이 5 N을 넘어 기존 90% 성공
조건을 충족하지 못했다. 이 조건은 완화하지 않고 PPO 목표로 유지했다.

## 3. 성공 조건

V8과 동일한 물리 성공 조건을 사용했다.

1. 1,200 steps 동안 낙상 없이 완주
2. 여덟 advance 각각 평균 스윙발 접촉력 5 N 미만
3. 각 advance에서 5 N 미만 표본 비율 90% 이상
4. 여덟 settle 모두 양발 10 N 초과 비율 90% 이상
5. 여덟 보폭이 각각 20 mm 이상
6. 몸통 총 전진량 180 mm 이상

시간만 줄였으며 성공 문턱은 낮추지 않았다.

## 4. PPO 학습과 checkpoint

- policy/value network: 각각 64×64, Tanh
- observation/action/reward: V8과 동일
- seed: 7
- 병렬 환경: 4개
- 요청 학습량: 50,000 steps
- 실제 rollout 종료: 57,344 steps
- 실행 시간: 87.31초
- 완료 episode: 44
- 마지막 20 episode 평균 reward/length: 689.488 / 1,200

| checkpoint | 좌측 시작 | 우측 시작 | 판정 |
|---|---:|---:|---|
| 25k | 실패, 최소 접촉 표본 86.7% | 성공 | 제외 |
| 50k | 성공, reward 724.374 | 성공, reward 724.148 | 후보 |
| 57,344 | 성공, reward 726.813 | 성공, reward 724.172 | **최종 선택** |

최종 정책은 양쪽 모든 advance에서 5 N 미만 표본 100%를 달성했다. 기준운동이 놓친
초기 발 분리를 residual policy가 보정한 결과다.

## 5. 최종 정책 결과

| 순서 | 전진 | 평균 속도 | swing 평균 하중 범위 | 보폭 범위 | 최대 swing 높이 | 최종 좌/우 하중 | 성공 |
|---|---:|---:|---:|---:|---:|---:|---|
| 좌→우 반복 | 248.17 mm | 0.00517 m/s | 0.000–0.314 N | 27.99–35.75 mm | 24.98 mm | 40.89/41.09 N | 예 |
| 우→좌 반복 | 248.64 mm | 0.00518 m/s | 0.000–0.287 N | 24.90–40.25 mm | 27.01 mm | 40.55/41.44 N | 예 |

두 순서 모두 여덟 settle의 양발 접촉을 100% 유지했다. V8의 약 0.0031 m/s에서
약 0.00518 m/s로 평균 속도가 약 66% 증가했다. 이동 거리는 거의 같지만 수행 시간이
80초에서 48초로 줄었다.

## 6. 착지 충격과 한계

속도 향상에는 비용이 있었다.

- V8 최종 정책의 최대 착지 peak는 약 43.5 N이었다.
- V9는 왼쪽 시작에서 51.02 N, 오른쪽 시작에서 65.82 N까지 증가했다.
- 오른쪽 시작의 최소 보폭은 24.90 mm로 성공 문턱과의 여유가 줄었다.
- 평균 속도 0.00518 m/s는 여전히 실제 보행에 비해 매우 느리다.
- 8스텝 finite-horizon, 명목 도착 자세, 고정 마찰·질량·지연 조건이라는 제약은 남아 있다.
- 실물 actuator의 torque-speed, backlash, 전류 제한과 충격 허용치는 반영하지 않았다.

따라서 바로 4초/step으로 줄이는 것은 적절하지 않다. 다음 V10은 6초 총시간을 유지하면서
land phase를 더 부드럽게 배분하고 landing peak penalty를 보상에 추가해야 한다. 양쪽 최대
착지력을 50 N 이하로 낮춘 뒤에만 5초 또는 4초 시간 압축을 시도하는 것이 안전하다.

## 7. 산출물

- 최종 좌우 비교 영상: [`fast_eight_step_trained_side_by_side.mp4`](../results/khr3hv_v9_symmetric/fast_eight_step_trained_side_by_side.mp4)
- 좌측 시작 PPO 영상: [`fast_eight_step_left_first_trained.mp4`](../results/khr3hv_v9_symmetric/fast_eight_step_left_first_trained.mp4)
- 우측 시작 PPO 영상: [`fast_eight_step_right_first_trained.mp4`](../results/khr3hv_v9_symmetric/fast_eight_step_right_first_trained.mp4)
- 좌측 시작 기준 영상: [`fast_eight_step_left_first_reference.mp4`](../results/khr3hv_v9_symmetric/fast_eight_step_left_first_reference.mp4)
- 우측 시작 기준 영상: [`fast_eight_step_right_first_reference.mp4`](../results/khr3hv_v9_symmetric/fast_eight_step_right_first_reference.mp4)
- 최종 정책: [`ppo_fast_eight_step_final.zip`](../results/khr3hv_v9_symmetric/ppo_fast_eight_step_final.zip)
- 시간·공간 탐색: [`reference_search_summary.json`](../results/khr3hv_v9_symmetric/reference_search_summary.json)
- 학습 요약: [`training_summary.json`](../results/khr3hv_v9_symmetric/training_summary.json)
- checkpoint 비교: [`checkpoint_comparison.json`](../results/khr3hv_v9_symmetric/checkpoint_comparison.json)
- 좌측 시작 결과: [`fast_eight_step_left_first_trained_summary.json`](../results/khr3hv_v9_symmetric/fast_eight_step_left_first_trained_summary.json)
- 우측 시작 결과: [`fast_eight_step_right_first_trained_summary.json`](../results/khr3hv_v9_symmetric/fast_eight_step_right_first_trained_summary.json)
- 환경 코드: [`fast_eight_step_v9_env.py`](../humanoidv2/fast_eight_step_v9_env.py)

## 8. 재실행

```bash
cd /home/king0519/projects/Humanoidv2
python3 -m pytest -q
python3 scripts/search_fast_eight_step_v9_reference.py
python3 scripts/train_fast_eight_step_v9.py --timesteps 50000 --n-envs 4
python3 scripts/evaluate_fast_eight_step_v9.py --first-side left
python3 scripts/evaluate_fast_eight_step_v9.py --first-side right
python3 scripts/compose_side_by_side_video.py \
  --left-video results/khr3hv_v9_symmetric/fast_eight_step_left_first_trained.mp4 \
  --right-video results/khr3hv_v9_symmetric/fast_eight_step_right_first_trained.mp4 \
  --left-label "Left first: 6 s/step" --right-label "Right first: 6 s/step" \
  --output results/khr3hv_v9_symmetric/fast_eight_step_trained_side_by_side.mp4
```
