# KHR-3HV 방식 V8 전방 교대 8스텝 실험

- 실행일: 2026-08-06 (Asia/Seoul)
- 모델: 좌우 대칭 관성 Humanoidv2
- 목표: 전방으로 향하는 교대 보행을 reset 없이 8스텝 연결
- 최종 판정: **좌측·우측 시작 모두 8스텝 성공**
- 최종 정책: 57,344-step PPO

## 1. 목표와 재기준화 설계

V8의 진행 방향은 명확히 전방이다. 각 스윙발을 현재 뒤쪽 split stance에서 반대쪽 앞
split stance로 옮기며 8번 반복한다. 한 step은 V7과 같은 10초, 전체 episode는 80초와
2,000 control steps다.

V7 이후 두 재기준화 방식을 비교했다.

### 순수 월드 위치 재기준화

각 착지 후 실제 베이스 자세와 실제 양발 월드 위치에서 다음 IK를 새로 계산했다. 이 방식은
작은 몸통 높이 저하와 기울기까지 다음 step의 기준으로 확정해 오차가 누적됐다.

| 시작 | 종료 | 전진량 | 최대 step swing 하중 | 판정 |
|---|---:|---:|---:|---|
| 왼발 | 1,927 steps | 164.35 mm | 11.04 N | 낙상, 실패 |
| 오른발 | 1,817 steps | 139.08 mm | 7.52 N | 낙상, 실패 |

높이를 즉시 명목값으로 복구하거나 오차 일부만 보정하는 방식도 양쪽 성공 영역을 만들지
못했다. 따라서 V8 첫 성공판에는 하이브리드 재기준화를 사용했다.

### 하이브리드 관절 재기준화

매 step의 보간 시작점은 실제 착지 관절 상태로 갱신한다. 도착 목표는 V7에서 검증한
명목 좌우 split-stance 주기를 사용한다. 즉 측정된 시작 불연속은 제거하되, 자세·높이
오차를 다음 착지 목표로 계속 누적하지 않는다. simulator state reset이나 초기 자세 복귀는
없다.

## 2. 성공 조건

1. 2,000 steps 동안 낙상 없이 완주
2. 여덟 advance 구간 각각의 평균 스윙발 접촉력이 5 N 미만
3. 각 advance 샘플의 90% 이상이 5 N 미만
4. 여덟 settle 모두 양발 각각 10 N 초과인 비율이 90% 이상
5. 여덟 step length가 각각 20 mm 이상
6. 몸통 총 전진량이 180 mm 이상

발 분리·보폭·착지를 매 step 독립적으로 판정하므로 미끄러짐이나 일부 step 생략은 성공으로
세지 않는다.

## 3. 기준궤적 탐색

하이브리드 방식에서 보폭 25/30/35 mm, sway 85/90 mm, Kp 70/80, 양쪽 시작을
조합해 24회 평가했다. 개별 성공은 4회, 동일 파라미터 양쪽 성공은 2개였다.

- `25 mm / 90 mm / Kp 80`
- `30 mm / 90 mm / Kp 80` — 최종 선택

더 큰 전진량을 위해 30 mm 보폭을 선택했다.

| 기준 순서 | step별 swing 평균 하중 범위 | step별 보폭 범위 | 몸통 전진 | 여덟 settle 양발 접촉 | 성공 |
|---|---:|---:|---:|---:|---|
| 좌→우 반복 | 1.801–2.443 N | 27.06–36.66 mm | 231.13 mm | 모두 100% | 예 |
| 우→좌 반복 | 1.635–2.306 N | 28.40–33.66 mm | 230.12 mm | 모두 100% | 예 |

## 4. PPO 학습과 checkpoint 선택

- policy/value network: 각각 64×64, Tanh
- observation: 센서 history 80 + 현재 스윙 측 부호 + 전체 진행률
- action: 기준관절 궤적에 추가하는 residual, 기본 action scale의 25%
- seed: 7
- 병렬 환경: 4개
- 요청 학습량: 50,000 steps
- 실제 rollout 종료: 57,344 steps
- 실행 시간: 175.82초
- 완료 episode: 28
- 마지막 20 episode 평균 reward/length: 1098.086 / 1918.55

최근 stochastic episode 평균에는 일부 조기 종료가 포함됐지만 checkpoint를 양쪽 시작으로
결정론 평가했을 때 세 후보는 모두 성공했다. 양쪽 중 낮은 reward가 가장 높은 최종 rollout을
선택했다.

| checkpoint | 좌측 시작 | 우측 시작 | 낮은 쪽 reward |
|---|---:|---:|---:|
| 25k | 성공, 1185.557 | 성공, 1187.070 | 1185.557 |
| 50k | 성공, 1214.541 | 성공, 1213.592 | 1213.592 |
| 57,344 | 성공, 1218.940 | 성공, 1218.923 | **1218.923** |

## 5. 최종 정책 결과

| 순서 | step별 swing 평균 하중 범위 | step별 보폭 범위 | 최대 swing 높이 | 몸통 전진 | 최종 좌/우 하중 | 성공 |
|---|---:|---:|---:|---:|---:|---|
| 좌→우 반복 | 0.000–0.331 N | 28.88–34.49 mm | 14.90 mm | 249.16 mm | 41.32/40.68 N | 예 |
| 우→좌 반복 | 0.000–0.121 N | 28.12–35.63 mm | 13.81 mm | 248.51 mm | 41.03/40.97 N | 예 |

두 평가 모두 80초와 2,000 steps를 완주했다. 여덟 settle의 양발 접촉은 모두 100%이며,
PPO는 기준운동보다 스윙발을 높이 들어 접촉을 줄이고 총 전진량을 약 18–19 mm 늘렸다.
8번째 발 분리와 최종 양발 착지는 저장 프레임에서도 확인했다.

## 6. 해석과 남은 한계

이번 결과는 전방 이동이 일시적인 한두 step 현상이 아니라 동일 정책으로 8번 반복될 수
있음을 보여준다. 4스텝 V7 대비 이동 거리는 약 2.2배로 증가했고, 후반 step에서도 보폭과
접촉 품질이 유지됐다.

그러나 아직 실용적인 연속 보행은 아니다.

- 24.9 cm 전진에 80초가 걸려 평균 속도는 약 0.0031 m/s다.
- 8스텝 뒤 종료되는 finite-horizon 동작이다.
- 하이브리드 방식은 실제 시작 관절을 반영하지만 도착점은 명목 주기이므로 완전한 월드 좌표
  receding-horizon 발걸음 계획은 아니다.
- 순수 월드 재기준화는 높이·자세 drift 때문에 실패했다.
- 성공 파라미터가 sway 90 mm/Kp 80 부근에 집중돼 동역학 여유가 좁다.
- 마찰, 질량, 지연, 센서 잡음, 외란과 actuator 실물 제한을 randomization하지 않았다.
- 정책은 기준궤적 residual controller이며 기준궤적 없이 gait를 생성하지 않는다.

다음 V9의 우선순위는 step 수를 더 늘리는 것보다 시간을 줄이는 것이다. 먼저 8스텝 성공
판정을 유지하며 lift/settle 시간을 단계적으로 압축해 10초/step을 6초, 이후 4초 수준으로
낮춰야 한다. 각 속도 단계에서 양쪽 checkpoint 검증을 통과한 뒤 마찰·질량·초기 자세
randomization과 외란 회복을 추가하는 것이 적절하다.

## 7. 산출물

- 최종 좌우 비교 영상: [`eight_step_trained_side_by_side.mp4`](../results/khr3hv_v8_symmetric/eight_step_trained_side_by_side.mp4)
- 좌측 시작 PPO 영상: [`eight_step_left_first_trained.mp4`](../results/khr3hv_v8_symmetric/eight_step_left_first_trained.mp4)
- 우측 시작 PPO 영상: [`eight_step_right_first_trained.mp4`](../results/khr3hv_v8_symmetric/eight_step_right_first_trained.mp4)
- 좌측 시작 기준 영상: [`eight_step_left_first_reference.mp4`](../results/khr3hv_v8_symmetric/eight_step_left_first_reference.mp4)
- 우측 시작 기준 영상: [`eight_step_right_first_reference.mp4`](../results/khr3hv_v8_symmetric/eight_step_right_first_reference.mp4)
- 최종 정책: [`ppo_eight_step_final.zip`](../results/khr3hv_v8_symmetric/ppo_eight_step_final.zip)
- 기준 및 월드 앵커 비교: [`reference_search_summary.json`](../results/khr3hv_v8_symmetric/reference_search_summary.json)
- 학습 요약: [`training_summary.json`](../results/khr3hv_v8_symmetric/training_summary.json)
- checkpoint 비교: [`checkpoint_comparison.json`](../results/khr3hv_v8_symmetric/checkpoint_comparison.json)
- 좌측 시작 최종 수치: [`eight_step_left_first_trained_summary.json`](../results/khr3hv_v8_symmetric/eight_step_left_first_trained_summary.json)
- 우측 시작 최종 수치: [`eight_step_right_first_trained_summary.json`](../results/khr3hv_v8_symmetric/eight_step_right_first_trained_summary.json)
- 환경 코드: [`eight_step_v8_env.py`](../humanoidv2/eight_step_v8_env.py)

## 8. 재실행

```bash
cd /home/king0519/projects/Humanoidv2
python3 -m pytest -q
python3 scripts/search_eight_step_v8_reference.py
python3 scripts/train_eight_step_v8.py --timesteps 50000 --n-envs 4
python3 scripts/evaluate_eight_step_v8.py --first-side left
python3 scripts/evaluate_eight_step_v8.py --first-side right
python3 scripts/compose_side_by_side_video.py \
  --left-video results/khr3hv_v8_symmetric/eight_step_left_first_trained.mp4 \
  --right-video results/khr3hv_v8_symmetric/eight_step_right_first_trained.mp4 \
  --left-label "Left first: 8 forward steps" \
  --right-label "Right first: 8 forward steps" \
  --output results/khr3hv_v8_symmetric/eight_step_trained_side_by_side.mp4
```
