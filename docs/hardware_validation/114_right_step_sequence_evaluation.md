# 114. Right Step Sequence Evaluation

## 목적

이전 단계까지는 각 동작을 따로 검증했다.

- weight shift
- right clearance
- right return

이번 단계의 목적은 이 개별 policy들을 하나의 시뮬레이션에서 순서대로 연결했을 때도 안정적으로 동작하는지 확인하는 것이다.

시퀀스:

1. weight shift
2. right clearance
3. right return
4. settle / standing 복귀

## 추가한 스크립트

- `scripts/evaluate_urdf_f_sequence.py`

역할:

- 하나의 MuJoCo episode 안에서 여러 policy를 순서대로 적용
- 각 stage마다 action scale, reward/termination 파라미터 적용
- 전체 영상, timeline, summary 저장

## Sequence v1

구성:

| Stage | Policy | Steps |
|---|---|---:|
| weight_shift | `weight_shift_left_30k` | 100 |
| right_clearance | `right_clearance_rollguard014_12mm_a007_30k` | 180 |
| right_return | `right_return_lift002_30k` | 170 |
| settle | zero | 100 |

결과:

| 항목 | 값 |
|---|---:|
| 완료 step | 499 |
| 종료 stage | settle |
| 종료 이유 | roll_limit |
| final roll | -0.1810 rad |
| final clearance | 0.00643 m |
| final right contacts | 1 |

해석:

- clearance stage 이후 오른발이 약 5.4mm 떠 있고 contact가 1개만 남았다.
- 기존 `lift002` return policy는 이 실제 sequence 상태에서 발을 충분히 내려놓지 못했다.
- settle 중 roll이 누적되어 종료됐다.

## Sequence v2

개별 평가에서 더 높은 lift pose에 대응한 `right_return_lift005_40k` policy를 새로 학습했다.

하지만 sequence에 연결했을 때는 더 빨리 실패했다.

결과:

| 항목 | 값 |
|---|---:|
| 완료 step | 362 |
| 종료 stage | right_return |
| 종료 이유 | roll_limit |
| final roll | -0.1811 rad |
| final clearance | 0.00589 m |
| final right contacts | 1 |

해석:

- 개별 `lift005` return policy는 5초 유지, final clearance 0.24mm, right contacts 4로 성공했다.
- 하지만 sequence의 실제 clearance state는 학습 시작 pose와 달라서 연결 성능은 나빴다.
- 단순히 더 높은 lift pose에서 학습한 return policy를 붙이는 것만으로는 부족했다.

## Sequence v3: 짧은 clearance 후 return

v1/v2 분석 결과, clearance stage가 너무 길면 오른발이 과도하게 떠서 return policy가 회복하기 어려웠다.

따라서 clearance stage를 180 step에서 100 step으로 줄였다.

구성:

| Stage | Policy | Steps |
|---|---|---:|
| weight_shift | `weight_shift_left_30k` | 100 |
| right_clearance | `right_clearance_rollguard014_12mm_a007_30k` | 100 |
| right_return | `right_return_lift002_30k` | 170 |
| settle | zero | 100 |

명령:

```bash
python3 scripts/evaluate_urdf_f_sequence.py \
  --clearance-steps 100 \
  --return-policy lift002 \
  --out-dir outputs/eval/urdf_f_sequence_right_step_v3_short_clearance
```

산출물:

- `outputs/eval/urdf_f_sequence_right_step_v3_short_clearance/sequence_evaluation.mp4`
- `outputs/eval/urdf_f_sequence_right_step_v3_short_clearance/sequence_evaluation.gif`
- `outputs/eval/urdf_f_sequence_right_step_v3_short_clearance/timeline.csv`
- `outputs/eval/urdf_f_sequence_right_step_v3_short_clearance/summary.json`

## Sequence v3 결과

전체 결과:

| 항목 | 값 |
|---|---:|
| 완료 step | 470 / 470 |
| 종료 이유 | 없음 |
| sim time | 9.4 s |
| final roll | 0.0093 rad |
| final pitch | -0.0845 rad |
| final clearance | 0.00011 m |
| final right contacts | 3 |
| final right force ratio | 0.277 |

단계별 결과:

| Stage | Steps | final roll | final pitch | final clearance | final right contacts | 안정 fraction |
|---|---:|---:|---:|---:|---:|---:|
| weight_shift | 100 | 0.0658 | -0.0882 | 0.00047 m | 4 | 1.0 |
| right_clearance | 100 | -0.0226 | -0.0549 | 0.00334 m | 1 | 1.0 |
| right_return | 170 | 0.0458 | -0.0752 | 0.00018 m | 4 | 1.0 |
| settle | 100 | 0.0093 | -0.0845 | 0.00011 m | 3 | 1.0 |

Gate:

| Stage | clearance gate longest | return gate longest |
|---|---:|---:|
| weight_shift | 0.60 s | 1.16 s |
| right_clearance | 1.06 s | 0.30 s |
| right_return | 0.70 s | 2.10 s |
| settle | 0.00 s | 2.00 s |

Gate 정의:

- clearance gate: right clearance `>= 2mm`, right contacts `<= 1`, roll/pitch 안정
- return gate: right clearance `<= 1mm`, right contacts `>= 3`, roll/pitch 안정

## 해석

처음으로 오른쪽 단일 step sequence가 끝까지 연결됐다.

중요한 포인트:

- clearance stage를 너무 길게 유지하면 return이 어렵다.
- 100 step 정도의 짧은 clearance 후 return으로 넘기면 성공했다.
- 오른발은 clearance stage에서 3.34mm까지 들리고 contact 1개로 줄었다.
- return stage에서 다시 clearance 0.18mm, contact 4개로 회복됐다.
- settle 후에도 roll/pitch가 안정적이다.

즉 “오른발을 들고 다시 내려놓는 단일 step”은 1차 성공으로 볼 수 있다.

## 현재 결론

성공한 것:

- 개별 policy sequence 연결
- 오른발 clearance 생성
- 오른발 contact 감소
- 오른발 contact 회복
- standing 복귀
- 전체 9.4초 sequence 비낙상 완료

아직 부족한 것:

- 전진 이동은 아직 없음
- 왼발 step은 아직 없음
- alternating gait는 아직 없음
- sequence는 hand-authored stage timing에 의존

## 다음 조치

다음 curriculum은 두 가지 중 하나다.

### 선택 A: left side mirror sequence

오른발에서 성공한 sequence를 왼발에도 적용한다.

목표:

- left weight shift
- left clearance
- left return
- standing 복귀

### 선택 B: right step forward placement

현재는 발을 들어 같은 자리 근처에 내리는 단계다.

다음은 오른발을 앞쪽으로 조금 이동시킨 뒤 내려놓는 것이다.

목표:

- right clearance
- right foot forward placement
- controlled return
- standing 복귀

현재 권장:

> 먼저 right step forward placement로 가기보다, left side mirror sequence를 검증해서 좌우 대칭 보행 가능성을 확인하는 것이 좋다.
