# KHR-3HV 방식 V10 저충격 8스텝 실험 — 전후 방향 정정

> **2026-08-08 정정:** V10은 월드 `+Y`를 전방으로 정의했지만 정측면 형상과 강제 평행이동을
> 대조한 결과 이 로봇의 해부학적 전방은 월드 `-Y`다. 따라서 아래에서 역사적으로
> `전방/전진`이라 부른 V10 동작은 실제로는 **후진 준정적 8스텝**이다. 기존 수치와 파일은
> 재현을 위해 보존하지만 전방 보행 성공 근거로 사용하지 않는다. 방향을 바로잡은 결과는
> [`KHR3HV_V13_CORRECTED_FORWARD_EXPERIMENT.md`](KHR3HV_V13_CORRECTED_FORWARD_EXPERIMENT.md)에
> 기록했다.

## 결론

V10은 V9의 6초/step, 당시 `+Y` 진행 8스텝, 대칭 관성 모델과 PPO 정책을 유지하면서 phase 시간만
재배분했다. 최종 정책은 좌측 시작과 우측 시작 모두 48초 동안 8스텝을 완주했고, 착지와
안정화 구간 전체의 최대 접촉력이 각각 **42.61 N / 47.45 N**으로 50 N 제한을 통과했다.

V9 대비 최대 착지력은 좌측 시작에서 16.48%, 우측 시작에서 27.91% 감소했다. `+Y` 이동거리는
각각 1.80%, 1.14% 줄어 충격 감소에 비해 속도 손실은 작았다.

## 변경 범위

외부 로봇의 물리값이나 환경 코드를 새로 섞지 않았다. V9와 같은 파일을 그대로 사용했다.

- 로봇: `URDF_F_v2_footprint_contact_symmetric_inertia.xml`
- 제어: 25 Hz 상위 제어, PD 관절 추종
- 보행: 실제 착지 관절 상태 + 명목 도착 자세의 V8 하이브리드 재기준화
- 정책: V9에서 학습한 PPO 잔차 정책
- 변경점: 6초 cycle 내부의 `land/settle` 시간 배분과 충격 판정/보상

| phase | V9 steps (s) | V10 steps (s) |
|---|---:|---:|
| lift | 60 (2.4) | 60 (2.4) |
| advance | 30 (1.2) | 30 (1.2) |
| land | 30 (1.2) | 50 (2.0) |
| settle | 30 (1.2) | 10 (0.4) |
| 합계 | 150 (6.0) | 150 (6.0) |

충격 판정 창은 `land`만이 아니라 `land + settle` 전체다. phase 경계 직후의 접촉력 spike가
누락되지 않도록 구현했다. `lift` 초반의 접촉력은 발이 아직 체중을 지지하며 하중을 빼는
구간이므로 착지 충격에서는 제외한다.

## 성공 조건

기존 V9 조건에 마지막 조건을 추가했다.

1. 1,200 control-step 동안 낙상 없이 8스텝 완주
2. 매 step advance 평균 swing force < 5 N
3. 매 step advance 표본 중 force < 5 N 비율 >= 90%
4. 매 step settle의 양발 접촉 표본 비율 >= 90%
5. 매 step 보폭 >= 20 mm
6. 몸통 전진거리 >= 180 mm
7. 매 step `land + settle` 최대 swing-foot force <= 50 N

## 시간 배분 탐색

V9 정책을 그대로 사용해 총 150-step을 유지한 후보를 양쪽 시작 순서로 평가했다.

| lift/advance/land/settle | 양방향 보행 | 양방향 50 N 통과 | 최악 착지력 |
|---|---:|---:|---:|
| 60/30/30/30 (V9) | 성공 | 실패 | 65.82 N |
| 60/30/35/25 | 성공 | 실패 | 58.31 N |
| 60/30/40/20 | 성공 | 실패 | 53.36 N |
| 60/30/45/15 | 성공 | 실패 | 50.95 N |
| 60/30/46/14 | 성공 | 실패 | 50.02 N |
| 60/30/47/13 | 성공 | 성공 | 49.17 N |
| 60/30/48/12 | 성공 | 성공 | 48.31 N |
| 60/30/49/11 | 성공 | 성공 | 47.35 N |
| **60/30/50/10** | **성공** | **성공** | **47.45 N** |

`lift`를 55 또는 50 step으로 줄인 후보는 좌측 시작 충격은 낮았지만 우측 시작 첫 step에서
낙상해 제외했다. 49-step 후보가 단일 결정론적 실행에서는 0.10 N 낮았지만 차이는 작았다.
최종 설정은 착지 보간 시간을 정확히 2.0초로 두고 50 N 경계에 약 2.55 N 여유가 있는
60/30/50/10으로 정했다.

## PPO 전이학습 시도와 실패

초기 60/30/45/15 설정에서 40 N 초과 접촉력에 제곱 페널티를 주고 V9 정책을 50,000-step
전이학습했다. PPO rollout 단위 때문에 실제 학습량은 57,344 step이었다.

- 25k checkpoint: 좌/우 모두 약 100 step에서 낙상
- 50k checkpoint: 좌 102, 우 107 step에서 낙상
- 마지막 checkpoint: 좌 104, 우 105 step에서 낙상
- 결과: 전이학습 checkpoint를 모두 배제하고 원래 V9 정책을 자동 선택

따라서 V10의 성공은 새 PPO 학습이 만든 것이 아니다. **실패한 전이학습을 채택하지 않고,
V9 강화학습 정책과 더 부드러운 기준궤적 시간 배분을 결합해 얻은 결과**다. 실패 기록과
checkpoint는 재현을 위해 보존했다.

## 최종 정량 결과

| 항목 | V9 좌측 시작 | V10 좌측 시작 | V9 우측 시작 | V10 우측 시작 |
|---|---:|---:|---:|---:|
| 엄격 성공 | 성공 | 성공 | 성공 | 성공 |
| 시간 | 48.0 s | 48.0 s | 48.0 s | 48.0 s |
| 몸통 전진 | 248.17 mm | 243.69 mm | 248.64 mm | 245.79 mm |
| 평균 전진속도 | 5.170 mm/s | 5.077 mm/s | 5.180 mm/s | 5.121 mm/s |
| 최대 착지+안정화 힘 | 51.02 N | **42.61 N** | 65.82 N | **47.45 N** |
| 50 N 제한 | 실패 | **통과** | 실패 | **통과** |

V10 PPO 정책은 모든 step에서 advance force < 5 N 표본 비율 100%, settle 양발 접촉 비율
100%, 보폭 24.35 mm 이상을 만족했다. 기준궤적만 사용한 경우 충격 제한은 통과했지만 첫
발 분리 표본 비율이 좌/우 83.3%/86.7%라 엄격 보행 성공에는 실패했다. 즉 V9의 PPO 잔차
보정은 여전히 필요하다.

## 영상과 결과 파일

- 기존 추적/새 고정 카메라 비교: [`tracking_vs_world_fixed_left_first.mp4`](../results/khr3hv_v10_symmetric/tracking_vs_world_fixed_left_first.mp4)
- 월드 고정 좌우 시작 비교: [`soft_landing_world_fixed_left_wide_side_by_side.mp4`](../results/khr3hv_v10_symmetric/soft_landing_world_fixed_left_wide_side_by_side.mp4)
- 월드 고정 좌측 시작: [`soft_landing_left_first_trained_world_fixed_left_wide.mp4`](../results/khr3hv_v10_symmetric/soft_landing_left_first_trained_world_fixed_left_wide.mp4)
- 월드 고정 우측 시작: [`soft_landing_right_first_trained_world_fixed_left_wide.mp4`](../results/khr3hv_v10_symmetric/soft_landing_right_first_trained_world_fixed_left_wide.mp4)
- V9/V10 우측 시작 비교: [`v9_v10_right_first_comparison.mp4`](../results/khr3hv_v10_symmetric/v9_v10_right_first_comparison.mp4)
- V10 좌우 시작 비교: [`soft_landing_trained_side_by_side.mp4`](../results/khr3hv_v10_symmetric/soft_landing_trained_side_by_side.mp4)
- V10 좌측 시작: [`soft_landing_left_first_trained.mp4`](../results/khr3hv_v10_symmetric/soft_landing_left_first_trained.mp4)
- V10 우측 시작: [`soft_landing_right_first_trained.mp4`](../results/khr3hv_v10_symmetric/soft_landing_right_first_trained.mp4)
- 최종 비교값: [`final_comparison.json`](../results/khr3hv_v10_symmetric/final_comparison.json)
- 시간 탐색: [`timing_search_summary.json`](../results/khr3hv_v10_symmetric/timing_search_summary.json)
- 전이학습 실패 비교: [`checkpoint_comparison.json`](../results/khr3hv_v10_symmetric/checkpoint_comparison.json)
- 좌측 정량 결과: [`soft_landing_left_first_trained_summary.json`](../results/khr3hv_v10_symmetric/soft_landing_left_first_trained_summary.json)
- 우측 정량 결과: [`soft_landing_right_first_trained_summary.json`](../results/khr3hv_v10_symmetric/soft_landing_right_first_trained_summary.json)
- 최종 정책: [`ppo_soft_landing_final.zip`](../results/khr3hv_v10_symmetric/ppo_soft_landing_final.zip)
- 환경 코드: [`soft_landing_v10_env.py`](../humanoidv2/soft_landing_v10_env.py)

## 재현 명령

```bash
python3 scripts/search_soft_landing_v10_reference.py
python3 scripts/train_soft_landing_v10.py --timesteps 50000 --n-envs 4
python3 scripts/evaluate_soft_landing_v10.py --first-side left
python3 scripts/evaluate_soft_landing_v10.py --first-side right
python3 scripts/evaluate_soft_landing_v10.py --first-side left \
  --camera-mode world_fixed --camera-azimuth 160 --camera-elevation -10 \
  --camera-distance 1.8 --camera-lookat 0 0.12 0.18 \
  --show-progress --label-suffix world_fixed_left_wide
python3 scripts/compose_side_by_side_video.py \
  --left-video results/khr3hv_v10_symmetric/soft_landing_left_first_trained.mp4 \
  --right-video results/khr3hv_v10_symmetric/soft_landing_right_first_trained.mp4 \
  --left-label "V10 left-first" --right-label "V10 right-first" \
  --output results/khr3hv_v10_symmetric/soft_landing_trained_side_by_side.mp4
```

새 고정 카메라는 로봇 base를 추적하지 않는다. 월드 좌표 `(0, 0.12, 0.18)`을 계속 바라보며
기존 135°/1.25 m에서 더 왼쪽인 160°/1.8 m로 변경했다. 화면 상단에는 시간과 월드 기준
몸통 전진거리를 표시하므로 카메라 추적으로 이동이 가려지는 문제를 피한다.

## 한계와 다음 단계

- 평지, 결정론적 정책, seed 7의 단일 물리 파라미터 검증이다.
- V10은 settle을 0.4초로 줄였지만 그 구간까지 충격과 양발 접촉을 검사한다.
- 새 PPO 전이학습은 실패했으므로 V10 정책 archive의 가중치는 V9 선택 정책과 같다.
- 다음 단계는 추가 속도 증가보다 마찰·질량·관성·지면 높이·센서 잡음을 바꾼 다중 seed
  robustness 평가가 적절하다.
