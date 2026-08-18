# KHR-3HV 방식 보행 강화학습 V3 실험 기록

- 실행일: 2026-08-05 (Asia/Seoul)
- 목표: 전후 발 기준궤적과 속도/lateral curriculum으로 실제 보행 유도
- 최종 판정: **20초 균형 성공, 단일지지 및 목표 전진 보행 실패**

## 1. V3 변경

KHR 논문의 좌우 이동과 교대 발 들기 구조를 유지하면서 현재 로봇용 전후 기준궤적을
추가했다. 최종 V3.4는 다음을 사용했다.

- 전후 보폭: 10 mm peak-to-peak
- 전후 위치: lift와 90° 위상차를 둔 사인파
- lateral/속도 curriculum:
  - 1단계: 30 mm, 0.05 m/s
  - 2단계: 32.5 mm, 0.10 m/s
  - 3단계: 35 mm, 0.15 m/s
- 학습량: 단계별 204,800, 총 614,400 environment steps
- 실제 스윙 발 하중이 낮아질 때 최대 0.15의 접촉 스케줄 보상
- 자세·속도·대칭·생존·낙상·토크 보상은 V2.2 구조 유지

## 2. 발견하고 수정한 물리 모델 오류

V2에서 추가한 몸통·다리 collision proxy가 초기 자세부터 서로 7–21 mm 겹쳐 있었다.
이 내부 접촉력 때문에 첫 제어 스텝에 발 간격이 150 mm에서 214 mm 이상으로 벌어졌고,
이후 약 256 mm까지 증가했다. 결과적으로 V2와 초기 V3 실험은 가짜 내부 힘의 도움을 받은
물리적으로 유효하지 않은 결과다.

수정 후 collision bit를 다음처럼 분리했다.

- 발 패드: 기존 contact bit 1
- 몸통/다리 proxy: contact type 2, affinity 0
- 바닥: proxy bit 2도 허용

따라서 proxy는 바닥과만 충돌하고 로봇 내부 및 발 패드와는 충돌하지 않는다. 수정 후 초기
내부 접촉은 0개이고 발 간격은 정확히 150 mm를 유지한다. V1은 collision proxy를 사용하지
않았으므로 이 오류의 영향을 받지 않았다.

## 3. 기준궤적 탐색

### IK 범위

처음 사용한 50 mm 보폭과 20 mm 발 들기는 hip pitch 제한 때문에 최대 32 mm의 IK 오차가
발생했다. 50 mm 보폭·10 mm 들기는 약 1 μm 오차로 풀렸지만 실제 발 가장자리는 바닥에서
떨어지지 않았다.

lift와 전후 위치가 동시에 최대가 되는 `sin/sin` 궤적을 버리고, 발을 뒤에서 들어 중간에서
중립 위치를 지나 앞에 착지하는 `sin/-cos` 위상으로 변경했다.

### lateral 안정성 경계

수정된 물리에서 lateral 30 mm는 20초 생존했지만 32.5 mm부터 기준궤적만으로는 낙상했다.
30 mm에서는 스윙 발 평균 하중이 약 31 N으로, 아직 단일지지는 아니었다. PPO가 이 경계를
점진적으로 넘어가도록 30→32.5→35 mm curriculum을 적용했다.

## 4. V3.4 학습 결과

- 총 학습량: 614,400단계
- 실행 시간: 3,142.99초 (52분 23초)
- seed: 7
- 병렬 환경: 4개

| 단계 | 명령속도 | lateral | 생존 | 20초 순전진 | 평균속도 |
|---|---:|---:|---:|---:|---:|
| 1 | 0.05 m/s | 30 mm | 500/500 | 0.0540 m | 0.00270 m/s |
| 2 | 0.10 m/s | 32.5 mm | 500/500 | 0.0711 m | 0.00356 m/s |
| 3 | 0.15 m/s | 35 mm | 500/500 | 0.0566 m | 0.00283 m/s |

모든 체크포인트를 최종 0.15 m/s 조건으로 다시 평가했을 때 최고 정책은 누적 404,800단계였다.

- 생존: 500/500단계, 20초
- 순전진: **0.0740 m**
- 평균속도: **0.00370 m/s**
- 목표 0.15 m/s 대비: 2.47%
- 성공 기준 2.4 m 대비: 3.08%

## 5. 접촉 분석

| 정책 | 양발 동시접촉 | 왼발 스윙 clear | 오른발 스윙 clear | 왼발 평균하중 | 오른발 평균하중 |
|---|---:|---:|---:|---:|---:|
| 최고 checkpoint | 100% | 0% | 0% | 46.16 N | 35.99 N |
| 최종 정책 | 100% | 0% | 0% | 46.79 N | 35.34 N |

접촉 스케줄 보상을 추가했지만 정책은 단일지지를 선택하지 않았다. 현재 결과 영상은 보행이
아니라 양발을 접촉한 채 작은 전후 움직임을 만드는 안정화 동작이다.

## 6. 결론

V3는 전후 기준과 curriculum을 구현했고, 잘못된 collision proxy를 찾아 수정했으며, 수정된
물리에서 60만 단계 학습까지 완료했다. 그러나 목표 보행에는 실패했다. 같은 환경에서 학습량만
늘리는 것은 권장하지 않는다. 체크포인트 보상은 상승하지만 순전진과 단일지지 비율은 증가하지
않기 때문이다.

다음 단계는 보행 PPO 전에 별도의 정적 단일지지 과제를 해결하는 것이다.

1. 전후 움직임을 끄고 2–4초 동안 한쪽 발 하중을 5 N 이하로 만드는 weight-transfer 정책 학습
2. 좌우 각각의 single-support 성공 자세와 필요한 hip/ankle roll 토크 확인
3. 이를 reset/teacher 상태로 사용해 한 발 들기 과제 학습
4. 단일 스텝 성공 후에만 전후 보폭과 연속 보행을 연결
5. 실제 모터 토크-속도 곡선과 발 마찰계수를 검증한 뒤 sim-to-real 진행

이는 KHR의 단순 기준운동을 버리는 것이 아니라, 더 무겁고 발 간격이 넓은 현재 로봇에서
KHR가 전제로 둔 weight transfer를 먼저 학습 가능한 하위 과제로 분리하는 것이다.

## 7. 산출물

- 최고 checkpoint 영상: [`results/khr3hv_v3_4_best_checkpoint/trained_policy.mp4`](../results/khr3hv_v3_4_best_checkpoint/trained_policy.mp4)
- 최종 정책 영상: [`results/khr3hv_v3_4/trained_policy.mp4`](../results/khr3hv_v3_4/trained_policy.mp4)
- 기준궤적 영상: [`results/khr3hv_v3_4/reference_only.mp4`](../results/khr3hv_v3_4/reference_only.mp4)
- 최종 정책: [`results/khr3hv_v3_4/ppo_khr3hv_final.zip`](../results/khr3hv_v3_4/ppo_khr3hv_final.zip)
- 체크포인트 비교: [`results/khr3hv_v3_4/checkpoint_comparison.json`](../results/khr3hv_v3_4/checkpoint_comparison.json)
- 접촉 분석: [`results/khr3hv_v3_4/contact_analysis.json`](../results/khr3hv_v3_4/contact_analysis.json)

## 8. 재실행

```bash
cd /home/king0519/projects/Humanoidv2
python3 -m pytest -q
python3 scripts/train_khr3hv_v3_curriculum.py --version v3_4 --steps-per-stage 200000 --n-envs 4
python3 scripts/compare_checkpoints.py --version v3_4
python3 scripts/analyze_v3_contacts.py --version v3_4
python3 scripts/evaluate_khr3hv_v1.py --version v3_4 --steps 500
```
