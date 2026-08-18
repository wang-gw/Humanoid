# KHR-3HV 방식 V7 교대 4스텝 실험

- 실행일: 2026-08-06 (Asia/Seoul)
- 모델: 좌우 대칭 관성 Humanoidv2
- 목표: 한 episode 안에서 reset 없이 네 발걸음을 교대로 연결
- 최종 판정: **좌→우→좌→우·우→좌→우→좌 모두 성공**
- 최종 정책: 50,000-step PPO checkpoint

## 1. V6에서 V7으로 확장한 부분

V6는 두 번째 착지에서 끝났다. 그 관절 목표를 단순히 첫 phase로 되돌리면 현재 split
stance와 다음 lift 시작 자세가 불연속이 된다. V7은 각 step의 `landed` 관절 상태를 다음
step IK의 seed와 보간 시작점으로 사용한다. 따라서 네 step 사이에 simulator reset, 상태
복사 또는 초기 자세 복귀가 없다.

각 step은 V6와 같은 10초이며 전체 episode는 40초, 1,000 control steps다.

| step 내부 phase | 길이 | 동작 |
|---|---:|---|
| lift | 100 steps / 4초 | 지지발 쪽으로 하중 이동 후 뒤쪽 발 들기 |
| advance | 50 steps / 2초 | 스윙발을 반대 split stance까지 전진 |
| land | 50 steps / 2초 | 스윙발 착지와 lateral 복귀 |
| settle | 50 steps / 2초 | 양발 접촉 안정화 및 다음 step 상태 전달 |

기준 파라미터는 보폭 30 mm, lateral sway 90 mm, IK lift 30 mm, Kp 80/Kd 0.32다.
첫 스윙발은 episode마다 무작위로 선택되며 정책 하나가 두 순서를 모두 처리한다.

## 2. 성공 조건

다음을 모두 만족해야 성공이다.

1. 1,000 steps 동안 낙상 없이 완주
2. 네 advance 구간의 스윙발 평균 접촉력이 각각 5 N 미만
3. 네 advance 모두 샘플의 90% 이상이 5 N 미만
4. 네 settle 구간 모두 양발 각각 10 N 초과인 비율이 90% 이상
5. 네 step length가 각각 20 mm 이상
6. 몸통 총 전진량이 70 mm 이상

매 step의 발 분리, 착지, 보폭을 따로 검사하므로 발을 끌거나 마지막 한 번만 크게 이동하는
동작은 성공으로 판정되지 않는다.

## 3. 기준궤적 탐색

보폭 20/30/40 mm, sway 80/90/100 mm, Kp 65/80과 양쪽 시작 순서를 조합해 36회
평가했다. 개별 성공은 3회였고 동일 파라미터로 양쪽 순서가 함께 성공한 조합은
`30 mm / 90 mm / Kp 80` 하나뿐이었다. V6보다 허용 파라미터 영역이 좁아졌으므로
현재 결과에는 동역학 여유가 크지 않다.

| 기준 순서 | step별 swing 평균 하중 | step별 보폭 | 몸통 전진 | 네 settle 양발 접촉 | 성공 |
|---|---|---|---:|---:|---|
| 좌→우→좌→우 | 2.443 / 1.857 / 1.880 / 1.686 N | 27.06 / 31.63 / 29.21 / 35.09 mm | 102.64 mm | 모두 100% | 예 |
| 우→좌→우→좌 | 1.635 / 1.760 / 2.124 / 1.849 N | 28.40 / 30.29 / 30.54 / 32.94 mm | 102.99 mm | 모두 100% | 예 |

## 4. PPO 학습과 checkpoint 선택

- policy/value network: 각각 64×64, Tanh
- observation: 센서 history 80 + 현재 스윙 측 부호 + 전체 진행률
- action: 10개 관절 기준궤적에 추가하는 residual, 기본 action scale의 25%
- seed: 7
- 병렬 환경: 4개
- 요청 학습량: 50,000 steps
- 실제 rollout 종료: 57,344 steps
- 실행 시간: 151.86초
- 완료 episode: 57
- 마지막 20 episode 평균 reward/length: 575.465 / 1,000

checkpoint는 반드시 두 시작 순서를 따로 평가했다. 25k와 50k는 양쪽 모두 성공했지만
57,344-step 최종 rollout은 좌측 시작에서 131 steps 만에 넘어졌다. 우측 시작 reward만
보면 최종 rollout이 가장 높지만, 좌우 중 낮은 성능을 우선하는 규칙에 따라 50k를 선택했다.

| checkpoint | 좌측 시작 | 우측 시작 | 선택 판단 |
|---|---:|---:|---|
| 25k | 성공, 590.109 | 성공, 592.126 | 후보 |
| 50k | 성공, 599.660 | 성공, 602.612 | **최종 선택** |
| 57,344 | 실패, 131 steps | 성공, 608.911 | 좌우 회귀로 제외 |

이 결과는 평균 episode length가 1,000이라고 해도 양쪽 결정론 평가를 생략하면 회귀 정책을
선택할 수 있음을 보여준다.

## 5. 최종 50k 정책 결과

| 순서 | step별 swing 평균 하중 | step별 보폭 | 최대 swing 높이 | 몸통 전진 | 최종 좌/우 하중 | 성공 |
|---|---|---|---:|---:|---:|---|
| 좌→우→좌→우 | 0.900 / 0.037 / 0.565 / 0.172 N | 33.49 / 25.95 / 36.63 / 29.53 mm | 17.94 mm | 113.48 mm | 40.58 / 41.42 N | 예 |
| 우→좌→우→좌 | 0.046 / 0.079 / 0.266 / 0.206 N | 31.64 / 28.63 / 33.99 / 32.52 mm | 13.15 mm | 111.87 mm | 40.27 / 41.73 N | 예 |

두 순서 모두 40초 전체를 완주했고 네 settle에서 양발 접촉을 100% 유지했다. PPO는
기준궤적보다 스윙발을 더 높이 들어 접촉력을 낮췄으며 몸통 전진도 약 9–11 mm 늘렸다.
네 번째 스윙과 최종 착지는 저장 프레임으로도 확인했다.

## 6. 해석과 한계

V7은 두 걸음 뒤에도 상태를 초기화하지 않고 같은 접촉 전환을 한 주기 더 반복할 수 있음을
보였다. 특히 세 번째와 네 번째 step 성공은 V6 최종 자세가 다음 주기의 동역학적 시작점으로
사용될 수 있다는 증거다.

아직 일반적인 연속 보행은 아니다.

- 네 걸음 뒤 episode가 종료되는 finite-horizon 기준동작이다.
- 11.2 cm 전진에 40초가 걸려 평균 속도는 약 0.0028 m/s다.
- 이전 `landed` 목표를 다음 IK seed로 전달하지만, 실제 측정 착지 위치를 매번 온라인 원점으로
  다시 잡는 receding-horizon 제어는 아니다.
- 36개 탐색 중 양쪽 성공 파라미터가 하나뿐이어서 sway/gain 여유가 좁다.
- 바닥 마찰, 질량, 초기 자세, 센서 잡음, 제어 지연 randomization이 없다.
- 기준궤적에 residual을 더하는 정책이며 기준궤적 없이 gait를 생성하지 않는다.
- 실물 actuator의 torque-speed, backlash와 전류 제한은 반영하지 않았다.

다음 V8은 측정된 착지 위치와 몸통 상태를 매 step의 로컬 원점으로 갱신하는 온라인
re-anchoring을 먼저 구현하는 것이 적절하다. 그 상태에서 8스텝 이상을 성공시킨 뒤 lift와
settle 시간을 단계적으로 줄이고, 파라미터 randomization과 외란 회복을 추가해야 한다.

## 7. 산출물

- 최종 좌우 비교 영상: [`four_step_trained_side_by_side.mp4`](../results/khr3hv_v7_symmetric/four_step_trained_side_by_side.mp4)
- 좌측 시작 PPO 영상: [`four_step_left_right_left_right_trained.mp4`](../results/khr3hv_v7_symmetric/four_step_left_right_left_right_trained.mp4)
- 우측 시작 PPO 영상: [`four_step_right_left_right_left_trained.mp4`](../results/khr3hv_v7_symmetric/four_step_right_left_right_left_trained.mp4)
- 좌측 시작 기준 영상: [`four_step_left_right_left_right_reference.mp4`](../results/khr3hv_v7_symmetric/four_step_left_right_left_right_reference.mp4)
- 우측 시작 기준 영상: [`four_step_right_left_right_left_reference.mp4`](../results/khr3hv_v7_symmetric/four_step_right_left_right_left_reference.mp4)
- 최종 50k 정책: [`ppo_four_step_final.zip`](../results/khr3hv_v7_symmetric/ppo_four_step_final.zip)
- 기준궤적 탐색: [`reference_search_summary.json`](../results/khr3hv_v7_symmetric/reference_search_summary.json)
- 학습 요약: [`training_summary.json`](../results/khr3hv_v7_symmetric/training_summary.json)
- checkpoint 비교: [`checkpoint_comparison.json`](../results/khr3hv_v7_symmetric/checkpoint_comparison.json)
- 좌측 시작 결과: [`four_step_left_right_left_right_trained_summary.json`](../results/khr3hv_v7_symmetric/four_step_left_right_left_right_trained_summary.json)
- 우측 시작 결과: [`four_step_right_left_right_left_trained_summary.json`](../results/khr3hv_v7_symmetric/four_step_right_left_right_left_trained_summary.json)
- 환경 코드: [`four_step_v7_env.py`](../humanoidv2/four_step_v7_env.py)

## 8. 재실행

```bash
cd /home/king0519/projects/Humanoidv2
python3 -m pytest -q
python3 scripts/search_four_step_v7_reference.py
python3 scripts/train_four_step_v7.py --timesteps 50000 --n-envs 4
python3 scripts/evaluate_four_step_v7.py --first-side left
python3 scripts/evaluate_four_step_v7.py --first-side right
python3 scripts/compose_side_by_side_video.py \
  --left-video results/khr3hv_v7_symmetric/four_step_left_right_left_right_trained.mp4 \
  --right-video results/khr3hv_v7_symmetric/four_step_right_left_right_left_trained.mp4 \
  --left-label "Left-right-left-right" --right-label "Right-left-right-left" \
  --output results/khr3hv_v7_symmetric/four_step_trained_side_by_side.mp4
```
