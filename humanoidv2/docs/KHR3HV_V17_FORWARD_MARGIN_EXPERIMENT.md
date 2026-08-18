# KHR-3HV 방식 V17 전진 여유 회복 실험

## 결론

V17은 V16의 2.56초/step과 gain scheduling을 유지하고 명목 보폭을 35 mm에서 **42 mm**로
늘렸다. 성공 판정의 최소 전진량도 180 mm에서 200 mm로 강화했다. V16 PPO 정책은 좌측·
우측 선행 모두 20.48초 동안 8스텝을 완주하고 239.07/242.67 mm 전진했다.

200 mm 기준 여유는 39.07/42.67 mm다. V16의 180 mm 기준 여유 6.88/4.55 mm보다 크게
회복됐다. 같은 시간에 이동 거리가 늘어 평균 속도도 11.67/11.85 mm/s로 상승했다.

다만 최대 착지력이 48.53/49.05 N으로 50 N 제한에 가까워졌다. 전진 여유는 회복했지만
착지 여유는 약 0.95 N까지 감소했으므로 명목 물성 밖의 강건성은 아직 확인되지 않았다.

## 고정한 V16 조건

- 한 스텝 주기: 64 control-step, 2.56초
- phase: `20/8/10/21/5`
- `lift / hold / advance`: `kp=130`, `kd=0.52`
- `land / settle`: `kp=80`, `kd=0.32`
- advance 마지막 12% gain 보간
- sway / lift: 90 / 30 mm
- PPO 잔차 제한: 관절 가동범위의 10%

V17은 주기를 더 줄이지 않았다.

## 보폭 탐색

35/38/40/42 mm를 참조와 V16 정책, 좌·우 선행으로 총 16회 평가했다.

| 보폭 | 결과 요약 |
|---:|---|
| 35 mm | 모두 기존 엄격 성공, 정책 전진 184.55–186.88 mm로 200 mm 미달 |
| 38 mm | 전진 210.78–225.97 mm, 착지 52.90–58.59 N으로 실패 |
| 40 mm | 전진 231.55–238.53 mm, 착지 51.46–53.23 N으로 실패 |
| 42 mm | 참조와 정책 모두 성공, 정책 전진 239.07–242.67 mm |

결과는 단조롭지 않았다. 38/40 mm에서 착지 타이밍과 접촉 과도응답이 악화됐지만 42 mm에서
다시 phase와 물리 응답이 맞았다. 따라서 단순 보간으로 선택하지 않고 양쪽 실측 성공을
근거로 42 mm를 채택했다.

## 몸통 진행 목표

몸통을 물리적으로 이동시키거나 base에 외력을 가하지 않았다. 보상에만 phase별 누적 목표를
추가했다.

- 한 스텝 목표: 27.5 mm
- 8스텝 목표: 220 mm
- 최소 성공 조건: 200 mm
- lift/hold: 새 전진량 요구 없음
- advance: 해당 스텝 목표의 70%
- land: 나머지 30%
- settle: 목표 유지

목표보다 앞선 동작에는 페널티를 주지 않고 뒤처진 shortfall만 보상에 반영했다. 이는 이미
239 mm 전진하는 정책을 220 mm 쪽으로 뒤로 끌지 않기 위한 비대칭 설계다. advance/land에서
몸통이 실제로 후퇴하는 경우에만 작은 stall penalty를 적용했다.

## PPO 미세조정 결과

몸통 목표 보상을 포함해 learning rate `1e-5`로 57,344-step 미세조정했다. 모든 최근
rollout은 512/512 스텝을 완주했지만 최종 정책으로는 채택하지 않았다.

| 정책 | 최악 전진 여유 | 최악 착지 | 양쪽 성공 |
|---|---:|---:|---|
| 초기 V16 정책 | 39.07 mm | 49.05 N | 성공 |
| 25,000-step | 35.21 mm | 48.77 N | 성공 |
| 50,000-step | 34.10 mm | 54.57 N | 실패 |
| last | 33.64 mm | 58.88 N | 실패 |

몸통 보상은 학습 목표로 구현됐지만, 채택된 결과의 전진 여유 회복은 PPO가 아니라 42 mm
참조 보폭에서 나왔다. 최종 정책은 초기 V16 정책과 바이트 단위로 동일하게 보존했다.

## 최종 결과

| 항목 | 좌측 선행 | 우측 선행 |
|---|---:|---:|
| 엄격 성공 | 성공 | 성공 |
| 낙상 | 없음 | 없음 |
| 시간 / 스텝 수 | 20.48 s / 8 | 20.48 s / 8 |
| 전방 진행량 | 239.07 mm | 242.67 mm |
| 200 mm 기준 여유 | 39.07 mm | 42.67 mm |
| 평균 전방속도 | 11.673 mm/s | 11.849 mm/s |
| 최소 보폭 | 31.35 mm | 29.85 mm |
| 접촉 해제율 | 100% | 100% |
| 최대 착지력 | 48.53 N | 49.05 N |
| 최대 관절 토크 | 9.17 N·m | 9.15 N·m |
| 토크 포화율 | 0% | 0% |

## 영상과 산출물

- [정면 좌우 비교 영상](../results/khr3hv_v17_forward_margin/forward_margin_v17_world_fixed_front_side_by_side.mp4)
- [사선 좌우 비교 영상](../results/khr3hv_v17_forward_margin/forward_margin_v17_world_fixed_side_by_side.mp4)
- [정면 좌측 선행 영상](../results/khr3hv_v17_forward_margin/forward_margin_v17_left_first_trained_world_fixed_front.mp4)
- [정면 우측 선행 영상](../results/khr3hv_v17_forward_margin/forward_margin_v17_right_first_trained_world_fixed_front.mp4)
- [좌측 정량 결과](../results/khr3hv_v17_forward_margin/forward_margin_v17_left_first_trained_world_fixed_summary.json)
- [우측 정량 결과](../results/khr3hv_v17_forward_margin/forward_margin_v17_right_first_trained_world_fixed_summary.json)
- [보폭 탐색](../results/khr3hv_v17_forward_margin/stride_search.json)
- [PPO checkpoint 비교](../results/khr3hv_v17_forward_margin/checkpoint_comparison.json)
- [학습 요약](../results/khr3hv_v17_forward_margin/training_summary.json)
- [최종 정책](../results/khr3hv_v17_forward_margin/ppo_forward_margin_v17_final.zip)
- [환경 코드](../humanoidv2/forward_margin_v17_env.py)

## 재현 명령

```bash
python3 scripts/search_dynamic_forward_v17_stride.py
python3 scripts/train_forward_margin_v17.py --timesteps 50000 --learning-rate 1e-5
python3 scripts/evaluate_forward_margin_v17.py --first-side left
python3 scripts/evaluate_forward_margin_v17.py --first-side right
```

## 다음 단계

V17은 전진 여유를 회복했지만 착지 여유가 줄었다. 다음은 추가 속도 압축 전에 다음 변화를
평가해야 한다.

- 여러 seed의 초기 관절·속도 오차
- 바닥 마찰 변화
- 전체 질량과 좌우 질량 비대칭
- 모터 gain과 제어 지연 변화

최악 조건에서 착지 50 N과 전진 200 mm를 동시에 통과하지 못하면 V17을 다음 속도 단계의
초기값으로 사용하지 않고, 보폭 42 mm에서 착지 phase 또는 landing gain의 강건성을 먼저
확보한다.
