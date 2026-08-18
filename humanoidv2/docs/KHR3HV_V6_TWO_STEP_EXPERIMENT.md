# KHR-3HV 방식 V6 교대 2스텝 실험

- 실행일: 2026-08-06 (Asia/Seoul)
- 모델: 좌우 대칭 관성 Humanoidv2
- 목표: 한 episode 안에서 리셋 없이 왼발과 오른발을 차례로 전진
- 최종 판정: **좌→우·우→좌 교대 2스텝 모두 성공**

## 1. 이번 단계의 의미

V5는 평행한 양발 자세에서 한 발을 앞으로 놓는 데 성공했지만, 그 직후 episode가
끝났다. V6는 첫 착지의 split stance를 그대로 유지한 채 하중을 반대쪽으로 옮기고,
뒤쪽 발을 들어 앞발 앞으로 보내도록 확장했다. 첫발과 둘째 발 사이에 reset이나 상태
복사는 없다.

한 episode는 20초, 500 control steps이며 다음 8개 phase로 구성한다.

| phase | step | 시간 | 동작 |
|---|---:|---:|---|
| first_lift | 0–99 | 0–4초 | 첫 스윙발 들기 |
| first_advance | 100–149 | 4–6초 | 첫 스윙발 전진 |
| first_land | 150–199 | 6–8초 | 첫발 착지 |
| first_settle | 200–249 | 8–10초 | split stance 안정화 |
| second_lift | 250–349 | 10–14초 | 반대쪽 뒤 발 들기 |
| second_advance | 350–399 | 14–16초 | 둘째 발 전진 |
| second_land | 400–449 | 16–18초 | 둘째 발 착지 |
| second_settle | 450–499 | 18–20초 | 최종 양발 안정화 |

기준운동은 30 mm 보폭, 90 mm lateral sway, 30 mm IK lift, Kp 80/Kd 0.32를
사용한다. 매 reset에서 첫 스윙발을 무작위로 골라 정책 하나가 양쪽 순서를 모두 학습하게
했다.

## 2. 성공 조건

다음을 모두 만족해야 2스텝 성공으로 판정한다.

1. 20초 동안 낙상하지 않음
2. 첫째·둘째 advance의 평균 스윙발 접촉력이 각각 5 N 미만
3. 두 advance 모두 샘플의 90% 이상이 5 N 미만
4. 첫 settle과 최종 settle에서 양발 각각 10 N 초과인 비율이 90% 이상
5. 첫째·둘째 step length가 각각 20 mm 이상
6. 몸통이 시작점보다 35 mm 이상 전진

발을 앞으로 미끄러뜨리는 동작을 성공으로 잘못 세지 않도록 두 번의 swing contact와
두 번의 양발 착지를 독립적으로 검사한다.

## 3. 기준궤적 탐색

보폭 20/30/40 mm, lateral sway 70/80/90 mm, Kp 65/80과 두 시작 순서를 조합해
36회 평가했다. 개별 성공은 4회였고, 동일 parameter로 두 순서가 모두 성공한 조합은
2개였다. 더 큰 유효 보폭을 우선해 `30 mm / 90 mm / Kp 80`을 선택했다.

| 순서 | 첫 swing 하중 | 둘째 swing 하중 | 첫 step | 둘째 step | 몸통 전진 | 최종 양발 접촉 | 성공 |
|---|---:|---:|---:|---:|---:|---:|---|
| 좌→우 | 2.443 N | 1.857 N | 27.06 mm | 31.63 mm | 42.51 mm | 100% | 예 |
| 우→좌 | 1.635 N | 1.760 N | 28.40 mm | 30.29 mm | 43.58 mm | 100% | 예 |

기준궤적만으로도 두 순서가 성공하므로, PPO 실패 시 기준동작 자체와 학습 문제를 분리해
진단할 수 있다.

## 4. V6 PPO 학습과 checkpoint 선택

- MLP policy/value: 각각 64×64, Tanh
- observation: 센서 history 80 + 현재 스윙 측 부호 + 전체 진행률
- action: 10개 관절 기준궤적에 추가하는 residual, 기본 action scale의 25%
- seed: 7
- 병렬 환경: 4개
- 요청 학습량: 50,000 steps
- 실제 학습량: PPO rollout 단위로 57,344 steps
- 실행 시간: 153.96초
- episode 수: 112
- 마지막 20 episode 평균 reward/length: 290.175 / 500

25k, 50k, 57,344-step checkpoint를 좌→우와 우→좌로 각각 결정론 평가했다. 모든
checkpoint가 양쪽 순서에서 성공했다. 두 순서 중 낮은 reward를 최대화하는 규칙으로 최종
rollout을 선택했다.

| checkpoint | 좌→우 | 우→좌 | 낮은 쪽 reward |
|---|---:|---:|---:|
| 25k | 성공, 298.588 | 성공, 297.801 | 297.801 |
| 50k | 성공, 299.931 | 성공, 300.495 | 299.931 |
| 57,344 | 성공, 300.631 | 성공, 300.932 | **300.631** |

## 5. 최종 정책 결과

| 순서 | 첫 swing 하중 | 둘째 swing 하중 | 첫 step | 둘째 step | 최대 swing 높이 | 몸통 전진 | 최종 좌/우 하중 | 성공 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 좌→우 | 0.351 N | 0.000 N | 32.54 mm | 26.75 mm | 22.13 mm | 48.61 mm | 40.15/41.85 N | 예 |
| 우→좌 | 0.648 N | 0.000 N | 31.29 mm | 27.57 mm | 17.91 mm | 47.52 mm | 40.85/41.15 N | 예 |

두 평가 모두 500 steps를 완주했고 최종 settle 전체에서 양발 접촉을 유지했다. 기준궤적보다
스윙발을 높게 들어 접촉을 줄였고, 몸통 전진량도 약 4–6 mm 증가했다. 렌더 프레임에서도
두 번째 발 분리와 최종 양발 착지를 확인했다.

## 6. 해석과 한계

이번 결과는 `정적 단일지지 → 한 걸음 → 교대 두 걸음`으로 난도를 점진적으로 높였을 때
대칭 관성 모델에서 접촉 순서가 이어진다는 증거다. 첫 착지 이후 상태를 버리지 않고 반대발
스윙까지 연결했다는 점이 V5와의 핵심 차이다.

그러나 아직 연속 보행 성공으로 부르지는 않는다.

- 두 걸음 뒤 episode가 끝나며, 최종 상태에서 다음 주기를 재계획하지 않는다.
- 4.8 cm 전진에 20초가 걸려 평균 속도는 약 0.0024 m/s이다.
- 기준궤적의 phase 시간과 초기 자세가 고정되어 있다.
- 바닥 마찰, 질량, 센서 잡음, 지연과 외란에 대한 randomization이 없다.
- 액추에이터의 실물 torque-speed, backlash, 전류 제한은 아직 반영하지 않았다.
- 성공한 정책은 residual controller이며 기준궤적 없이 자율적으로 보행을 생성한 것이 아니다.

따라서 다음 V7은 같은 성공 판정을 유지하면서 4스텝 이상으로 반복해야 한다. 각 착지 시점의
실제 발 위치와 몸통 위치를 다음 주기 목표의 원점으로 갱신하고, 좌우 phase를 주기적으로
연결해야 한다. 4스텝 양방향 성공 후에 phase 시간을 줄이고 목표 속도를 점진적으로 높인 뒤,
마찰·질량·초기 자세 randomization을 추가하는 순서가 적절하다.

## 7. 산출물

- 최종 좌우 비교 영상: [`two_step_trained_side_by_side.mp4`](../results/khr3hv_v6_symmetric/two_step_trained_side_by_side.mp4)
- 좌→우 PPO 영상: [`two_step_left_right_trained.mp4`](../results/khr3hv_v6_symmetric/two_step_left_right_trained.mp4)
- 우→좌 PPO 영상: [`two_step_right_left_trained.mp4`](../results/khr3hv_v6_symmetric/two_step_right_left_trained.mp4)
- 좌→우 기준 영상: [`two_step_left_right_reference.mp4`](../results/khr3hv_v6_symmetric/two_step_left_right_reference.mp4)
- 우→좌 기준 영상: [`two_step_right_left_reference.mp4`](../results/khr3hv_v6_symmetric/two_step_right_left_reference.mp4)
- 최종 정책: [`ppo_two_step_final.zip`](../results/khr3hv_v6_symmetric/ppo_two_step_final.zip)
- 기준궤적 탐색: [`reference_search_summary.json`](../results/khr3hv_v6_symmetric/reference_search_summary.json)
- 학습 요약: [`training_summary.json`](../results/khr3hv_v6_symmetric/training_summary.json)
- checkpoint 비교: [`checkpoint_comparison.json`](../results/khr3hv_v6_symmetric/checkpoint_comparison.json)
- 좌→우 최종 수치: [`two_step_left_right_trained_summary.json`](../results/khr3hv_v6_symmetric/two_step_left_right_trained_summary.json)
- 우→좌 최종 수치: [`two_step_right_left_trained_summary.json`](../results/khr3hv_v6_symmetric/two_step_right_left_trained_summary.json)
- 환경 코드: [`two_step_v6_env.py`](../humanoidv2/two_step_v6_env.py)

## 8. 재실행

```bash
cd /home/king0519/projects/Humanoidv2
python3 -m pytest -q
python3 scripts/search_two_step_v6_reference.py
python3 scripts/train_two_step_v6.py --timesteps 50000 --n-envs 4
python3 scripts/evaluate_two_step_v6.py --first-side left
python3 scripts/evaluate_two_step_v6.py --first-side right
python3 scripts/compose_side_by_side_video.py \
  --left-video results/khr3hv_v6_symmetric/two_step_left_right_trained.mp4 \
  --right-video results/khr3hv_v6_symmetric/two_step_right_left_trained.mp4 \
  --left-label "Left then right" --right-label "Right then left" \
  --output results/khr3hv_v6_symmetric/two_step_trained_side_by_side.mp4
```
