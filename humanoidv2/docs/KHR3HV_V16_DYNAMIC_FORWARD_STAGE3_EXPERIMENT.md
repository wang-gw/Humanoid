# KHR-3HV 방식 V16 동적 보행 커리큘럼 3단계

## 결론

V16은 V15의 3.40초/step을 **2.56초/step**으로 압축했다. 최종 구성은 스윙 `kp=130`,
착지 `kp=80`이며, advance 마지막 12% 동안 smoothstep으로 gain을 보간한다. V15 PPO 정책은
좌측·우측 선행 모두 20.48초 동안 8스텝을 완주하고 엄격 조건을 통과했다.

평균 속도는 9.125/9.011 mm/s로 V15보다 약 22% 빨라졌다. 목표 0.15 m/s의 약 6%이며,
아직 자연스러운 동적 보행은 아니다.

## 최종 제어 구성

- 전방: 월드 `-Y`
- 보폭 / sway / lift: 35 / 90 / 30 mm
- PPO 잔차 제한: 관절 가동범위의 10%
- `lift / hold / advance`: `kp=130`, `kd=0.52`
- `land / settle`: `kp=80`, `kd=0.32`
- advance 마지막 12%: `130 → 80` smoothstep 보간
- 착지 제한: 50 N

| phase | control steps | 시간 |
|---|---:|---:|
| lift | 20 | 0.80 s |
| hold | 8 | 0.32 s |
| advance | 10 | 0.40 s |
| land | 21 | 0.84 s |
| settle | 5 | 0.20 s |
| 합계 | 64 | 2.56 s |

## 직접 압축과 smooth 전환 실패

처음에는 V15 phase를 비례 압축한 `24/8/11/16/5`, 스윙/착지 `kp=130/70`을 사용했다.
발 접촉 해제율은 100%였지만 참조 착지력은 약 60 N, V15 정책은 73–84 N까지 증가했다.
advance 마지막 25%를 부드럽게 전환한 경우 전진량은 늘었지만 충격은 더 커졌다. phase와
착지 gain을 먼저 맞추지 않은 상태에서는 smooth 전환만으로 문제를 해결할 수 없었다.

## 64-step 참조 탐색

8개 phase 배분과 착지 `kp=50–110`을 좌·우 순서로 총 112회 평가했다. 선택된 hard-switch
기준은 `20/8/10/21/5`, 스윙 `kp=130`, 착지 `kp=80`이었다.

| hard-switch 참조 | 좌측 선행 | 우측 선행 |
|---|---:|---:|
| 엄격 성공 | 성공 | 성공 |
| 전방 진행 | 210.77 mm | 209.30 mm |
| 접촉 해제율 | 100% | 100% |
| 최대 착지력 | 48.07 N | 48.17 N |

## gain 전환 비교

선택된 phase에서 advance 말미 전환 비율을 비교했다.

- 0% hard-switch: 정책 양쪽 성공, 최악 착지 48.96 N
- 10%: 양쪽 성공, 최악 착지 47.67 N
- 12%: 양쪽 성공, 실제 중간 gain 표본 포함, 최악 착지 47.66 N
- 18%: 양쪽 성공하지만 오른쪽 전진량 180.85 mm로 기준 여유 0.85 mm
- 25%: 오른쪽 전진량 179.21 mm로 실패
- 40%: 양쪽 전진 기준 실패

10%는 10-sample advance의 마지막 표본에서만 gain이 바뀌어 실질적인 보간이 아니었다.
12%에서는 마지막 두 표본에 중간 gain과 착지 gain이 나타나면서 전진 여유도 유지하므로 이를
최종값으로 선택했다.

## PPO 미세조정

성공한 초기 V15 정책을 보존하고 전환 10% 환경에서 learning rate `1e-5`로 57,344-step
미세조정했다. 이후 모든 checkpoint를 최종 12% 환경에서 다시 평가했다.

- 초기 정책: 양쪽 성공, 최악 착지 47.66 N
- 25,000-step: 양쪽 성공, 최악 착지 47.67 N, 전진량 감소
- 50,000-step: 오른쪽 착지 55.22 N으로 실패
- last: 오른쪽 착지 57.34 N으로 실패

초기 정책이 가장 안전해 추가 학습 정책은 채택하지 않았다.

## 최종 결과

| 항목 | 좌측 선행 | 우측 선행 |
|---|---:|---:|
| 엄격 성공 | 성공 | 성공 |
| 낙상 | 없음 | 없음 |
| 시간 / 스텝 수 | 20.48 s / 8 | 20.48 s / 8 |
| 전방 진행량 | 186.88 mm | 184.55 mm |
| 평균 전방속도 | 9.125 mm/s | 9.011 mm/s |
| 최소 보폭 | 24.72 mm | 25.90 mm |
| 접촉 해제율 | 100% | 100% |
| 최대 착지력 | 47.35 N | 47.66 N |
| 최대 관절 토크 | 9.16 N·m | 9.20 N·m |
| 토크 포화율 | 0% | 0% |

오른쪽 전진량은 180 mm 성공 기준보다 4.55 mm만 높아 여유가 작다. V16은 명목 조건에서
성공했지만 다음 단계의 안정적인 초기값으로 보기에는 V15보다 취약하다.

## 영상과 산출물

- [정면 좌우 비교 영상](../results/khr3hv_v16_dynamic_forward_stage3/dynamic_forward_v16_world_fixed_front_side_by_side.mp4)
- [사선 좌우 비교 영상](../results/khr3hv_v16_dynamic_forward_stage3/dynamic_forward_v16_world_fixed_side_by_side.mp4)
- [정면 좌측 선행 영상](../results/khr3hv_v16_dynamic_forward_stage3/dynamic_forward_v16_left_first_trained_world_fixed_front.mp4)
- [정면 우측 선행 영상](../results/khr3hv_v16_dynamic_forward_stage3/dynamic_forward_v16_right_first_trained_world_fixed_front.mp4)
- [좌측 정량 결과](../results/khr3hv_v16_dynamic_forward_stage3/dynamic_forward_v16_left_first_trained_world_fixed_summary.json)
- [우측 정량 결과](../results/khr3hv_v16_dynamic_forward_stage3/dynamic_forward_v16_right_first_trained_world_fixed_summary.json)
- [참조 탐색](../results/khr3hv_v16_dynamic_forward_stage3/reference_search.json)
- [gain 전환 비교](../results/khr3hv_v16_dynamic_forward_stage3/transition_comparison.json)
- [PPO checkpoint 비교](../results/khr3hv_v16_dynamic_forward_stage3/checkpoint_comparison.json)
- [최종 정책](../results/khr3hv_v16_dynamic_forward_stage3/ppo_dynamic_forward_v16_final.zip)
- [환경 코드](../humanoidv2/dynamic_forward_v16_env.py)

## 재현 명령

```bash
python3 scripts/search_dynamic_forward_v16_reference.py
python3 scripts/search_dynamic_forward_v16_transition.py
python3 scripts/train_dynamic_forward_v16.py --timesteps 50000 --learning-rate 1e-5
python3 scripts/evaluate_dynamic_forward_v16.py --first-side left
python3 scripts/evaluate_dynamic_forward_v16.py --first-side right
```

## 다음 단계

V17에서 곧바로 1.92초로 압축하면 전진 기준을 잃을 가능성이 높다. 다음에는 먼저 2.56초를
유지한 채 보폭 또는 몸통 전진 목표를 조금 늘려 200 mm 이상의 여유를 회복하고, 여러 seed와
물성 변화에서 성공률을 측정하는 편이 타당하다. 그 이후에만 추가 주기 단축을 시도한다.
