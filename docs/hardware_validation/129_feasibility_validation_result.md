# 129. 하드웨어 타당성 검증 결과 (Feasibility Validation)

> 질문: **"현재 CHIRO 하드웨어 형상으로 걸을 수 있는가?"**
> 이 문서는 실제 진행 기록(doc 100~127)을 근거로 그 답을 정리한다.

---

## 1. 검증 기준 (무엇을 통과라고 볼 것인가)

doc 103이 정의한 gate 체계를 따른다. "걸을 수 있다"를 한 번에 판정하지 않고,
보행을 구성하는 **필수 능력들을 단계별 gate로 분해**해서 각각을 입증한다.

| 능력 | gate 기준 (doc 103) |
|---|---|
| standing | 10s 안정, roll/pitch 억제 |
| weight shift | 한쪽 발 force ratio 추종 (예 0.65~0.73) |
| one-leg support | 반대발 force ≤ 5N, contacts = 0, roll ≤ 0.12 rad |
| swing clearance | 발 clearance ≥ 2mm, roll ≤ 0.12 rad |
| step return | 든 발을 다시 안정 접촉 |
| alternating | 좌우 교대 |
| forward | 전진 속도 |

---

## 2. 단계별 통과 기록 (실제 수치)

### 2-1. 물리 probe 단계 (doc 100~103) — 부분 통과, RL 필요 판정

| gate | 결과 | 수치 | 판단 |
|---|---|---:|---|
| standing | ✅ 통과 | 10s 안정, torque ~5.16 Nm | 로드/접촉/질량 사용 가능 |
| weight shift | ✅ 통과 | left force ratio ~0.73 | 하중 이동 가능 |
| right unload | ✅ 통과 | right force ~21.8N, roll 0.126 | 하중 감소 가능 |
| 순간 toe-off | ✅ 통과 | clearance 순간 9mm↑ | 발은 뜬다 |
| one-leg 0.5s | ❌ 실패 | 0.36s | **RL 커리큘럼 필요** |
| open-loop 2mm lift | ❌ 실패 | 0.198s | PD 보간만으론 부족 |
| task-space lift | △ 개선 | 0.396s | closed-loop 여지 있음 |

**1차 결론(doc 103)**: "못 걷는다"는 아니다. 단순 제어로는 부족 → **RL로 넘어갈 가치 있음**.

### 2-2. RL 커리큘럼 단계 (doc 105~123) — one-leg/clearance gate 돌파

물리 probe에서 실패했던 gate들을 RL로 **넘어섰다**:

| gate | probe 결과 | RL 결과 | doc |
|---|---|---|---|
| one-leg support | 실패(0.36s) | ✅ 반대발 unload 달성 | 107, 118 |
| swing clearance ≥2mm | 실패 | ✅ Lclr 4.3mm, Rclr 5.2mm | 119, 122 |
| roll guard ≤0.12 | — | ✅ max roll 0.046 | 111, 112 |
| alternating | — | ✅ **3-cycle 완주(1885 steps)** | 123 |
| forward | — | ✅ **+3.4cm/3cycle** | 123 |

**커리큘럼 결론**: 양발 교번 보행을 넘어짐 없이 3-cycle 완주. **"이 형상으로 걷는 것이
가능하다"를 최초로 실증.**

### 2-3. End-to-End 단계 (doc 124~127) — 대칭·연속성 강화

| 항목 | 결과 | doc |
|---|---|---|
| 단일 정책 연속 보행 | ✅ gait clock으로 달성 | 124 |
| 발 평평도(ankle twist) | ✅ 24°→7~8° | 126 |
| 좌우 양발 대칭 | ✅ 왼발 들림 2.5→15mm (양발 15/15mm) | 127 |
| 전진 | ✅ +28cm | 127 |
| 토크 | ✅ 걷기 중 max 9.1 Nm (limit 30, saturation 없음) | 본 문서 §3 |

---

## 3. 모터/토크 관점 통과 근거 (실측)

E2E v5 정책 보행 중(1140 step) 관절 토크 실측:

- **전 관절 max torque = 9.1 Nm**, 평균 ~4.4 Nm (torque limit 30 Nm)
- **saturation 발생 없음**
- doc 103 probe 대조: standing 5.16 / unload·lift 11.8 / 공격형 collapse 23.3 Nm

→ **정상 보행에서 토크 여유는 3배 이상.** 모터 토크가 보행 실패의 원인이 아님을 재확인.
   (단, MuJoCo actuator 가정 기준. 실 모터 선정은 doc 130 참조.)

---

## 4. 설계 민감도 (통과를 좌우한 형상 요인, doc 103)

검증 과정에서 보행 가능성을 좌우한 **설계 파라미터**:

| 요인 | 관찰 | 시사점 |
|---|---|---|
| base COM +Y | 0.36s → +5mm에서 0.51s / +10mm↑ 악화 / +40~60mm roll collapse | **수 mm 단위 미세조정**이 유효 |
| 발 접촉 길이(local X) | 0.36s → 1.25배 0.47s / 1.30배↑ 악화 | 무작정 크게 X, **toe/heel 위치가 관건** |
| 발 폭(local Y) | COM margin↑ but gate 악화 | 폭보다 전후 길이가 민감 |
| seed 자세 좌우 하중 | 왼발 과부하 → 외발 보행 유발(doc 127) | **좌우 대칭 질량/자세** 중요 |

---

## 5. 최종 판정

### ✅ 통과했다고 볼 수 있는 것
- **"이 하드웨어 형상으로 걷는 것이 원리적으로 가능하다"** — 커리큘럼 3-cycle 완주 +
  E2E 양발 대칭 보행으로 **실증 완료**.
- 모터 토크 여유 충분(9/30 Nm), saturation 없음.
- 필요한 설계 조정이 크지 않음(COM 수mm, 접촉 길이 미세조정).

### ⚠️ 아직 "검증 완료"라고 할 수 없는 것
- **지속 안정성**: E2E v5는 57초 후 측방 낙하(max roll 0.53 rad). 무한 지속 보행 미달.
- **actuator 현실성**: torque-speed/backlash/thermal 미반영(doc 130에서 마진으로 보완).
- **sim-to-real**: 실기 검증 없음.

### 한 줄 결론
> **1차 타당성 검증(보행 가능성)은 통과.** "안정적으로 오래 걷는다"와
> "실기에서 재현된다"는 2차 검증 과제로 남는다.

---

## 6. 검증을 닫기 위한 다음 단계

1. **측방(roll) 안정성**: 낙하가 roll이므로 lateral 발 배치/스탠스 폭 보상 추가
2. **actuator 현실화**: doc 130 기준으로 실 모터 파라미터를 XML에 반영 후 재검증
3. **CAD 반영**: doc 103의 COM/접촉 길이 민감도를 실제 설계에 확정
4. **sim-to-real**: 위 3개 정리 후 실기 벤치 테스트
