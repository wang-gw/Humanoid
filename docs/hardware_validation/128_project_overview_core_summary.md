# 128. 프로젝트 핵심 정리 (Overview & Core Summary)

> 처음 보는 사람이 이 프로젝트의 목적·구조·성과·한계를 한 번에 이해하기 위한 문서.
> 세부 근거는 각 절에 명시된 doc 번호를 참조.

---

## 1. 한 줄 요약

**CHIRO 이족 보행 로봇의 하드웨어 형상이 "걸을 수 있는 설계인가"를, MuJoCo
시뮬레이션 + 강화학습(RL)으로 검증하는 프로젝트.** 물리 probe(doc 100~103)로
타당성을 1차 판단하고, RL 커리큘럼(doc 104~123)과 End-to-End 정책(doc 124~127)으로
실제 보행을 입증했다.

---

## 2. 대상 시스템

| 항목 | 내용 |
|---|---|
| 로봇 | CHIRO humanoid (이족, 하반신 중심) |
| 시뮬레이터 | MuJoCo 3.x + Gymnasium + Stable-Baselines3 (PPO) |
| 기준 모델 | `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml` |
| 작동 관절 | **10개** (다리당 5개): hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll |
| 전진 방향 | **+Y축** (world frame) |
| 제어 | 관절 PD (kp=20, kd=12), torque limit 30 Nm |
| 시간 | sim 200 Hz (dt=0.005s), 제어 20 Hz (frame_skip=10) |

**관찰(observation)**: base roll/pitch/yaw + 각속도, 관절 q/qd, 발 접촉력·접촉수,
발 clearance, (E2E는) gait phase [sin, cos]. → 45~47차원.
**행동(action)**: 관절 목표각 offset (PD 타겟).

---

## 3. 검증 흐름 (전체 여정)

```
[물리 probe]        [RL 커리큘럼]                    [End-to-End RL]
100~102 lift probe   105 standing                    124 gait clock 정책
103 하드웨어 결정 ──▶ 106 weight-shift          ┌──▶ 125 발 mesh 정렬
                     107 right unload            │    126 발 평평도(ankle)
                     108~112 clearance/roll gate │    127 양발 대칭 (v2~v6)
                     113~121 step/return/left    │
                     122 contact 비대칭 수정      │
                     123 양발 교번 보행 ──────────┘
```

### 3-1. 물리 타당성 판단 (doc 103, 핵심 분기점)
단순 PD/open-loop로는 부족하지만 **"RL 하면 걸을 가능성 있음"**으로 결론.
- standing / weight-shift / right-unload / 순간 toe-off: **통과**
- one-leg support 0.5s gate: **실패**(0.36s) → RL 커리큘럼 필요 판단
- 토크 saturation 거의 없음 → 모터 부족은 1차 원인 아님

### 3-2. RL 커리큘럼 (doc 104~123)
7단계(standing→weight-shift→one-leg→clearance→step-return→alternating→forward)로
분리 학습. **결과(doc 123): 양발 교번 3-cycle 완주(1885 steps), 넘어짐 0, 전진 +3.4cm.**
단점: policy 5개 전환 시 distribution shift 누적.

### 3-3. End-to-End 정책 (doc 124~127)
단일 PPO가 gait clock을 받아 보행 전체를 학습. 커리큘럼의 shift 문제 우회.
- **doc 125**: 발 시각 mesh와 collision box 3축 정렬 수정
- **doc 126**: 발이 모서리로 기울던 문제(ankle twist) → flat-foot 페널티로 24°→7° 해결
- **doc 127**: 좌우 비대칭(외발 보행) → from-scratch + swing-contact 페널티로 **양발 대칭 달성**

---

## 4. 최종 성과 (수치)

| 지표 | 커리큘럼(doc123) | E2E 최종 v5(doc127) |
|---|---|---|
| 보행 형태 | 양발 교번 3-cycle | 양발 교번(연속) |
| 좌우 발 들림 | Lclr 4.3mm | **15mm / 15mm (대칭)** |
| 전진 | +3.4cm/3cycle | **+28cm** |
| 지속 시간 | 1885 step 완주 | 1140 step(57초) 후 낙하 |
| max roll | 0.046 rad | 0.53 rad(낙하 시) |
| 걷기 중 최대 토크 | — | **9.1 Nm** (limit 30) |

**핵심 성취**: ① 발목 틀어짐 해결, ② **진짜 양발 대칭 보행 입증**, ③ 토크 여유 충분.

---

## 5. 알려진 한계 (정직한 기록)

1. **지속 안정성 미달**: E2E v5는 57초 후 **측방(roll)으로 낙하**. 무한 지속 보행 아님.
2. **대칭↔안정 상충**(doc 127): 런지(넓은 앞뒤 스탠스)가 안정성을 제공 → 발을 모으면
   더 빨리 낙하. 대칭+평평+장기안정 동시 달성은 reward shaping 5회로도 미완.
3. **이상화된 actuator**: torque-speed curve, backlash, 감속기 효율, thermal 미반영.
4. **sim-to-real 없음**: 실제 하드웨어 테스트 전무.
5. **CAD 미세조정 후보 미결**(doc 103): base COM ±수mm, toe/heel 접촉 길이.

---

## 6. 핵심 교훈 (재현·확장 시 참고)

- **warm-start는 로컬 옵티멈을 못 벗어난다**: 외발 보행이 v1→v4까지 대물림.
  큰 gait 변화는 **from-scratch + 탐색 강화(ent_coef)** 로만 탈출(doc 127).
- **clearance 지표는 기울어진 발을 오판**: 접촉 게이트는 clearance가 아닌
  실제 contact 수로 해야 함(doc 126).
- **넓은 스탠스 = 측방 안정성**: 안정성 개선은 발 배치(lateral)와 직결.
- **seed 자세의 좌우 하중 비대칭**(왼발 과부하)이 모든 비대칭의 출발점.

---

## 7. 관련 산출물

| 경로 | 내용 |
|---|---|
| `envs/urdf_f_env.py` | RL 환경 + walking 보상 6항 + jitter |
| `envs/gait_walking_env.py` | gait clock 포함 E2E 환경 |
| `scripts/train_e2e_walking.py` | 체크포인트 학습 (CLI로 보상 항 제어) |
| `scripts/render_e2e_walk.py` | 추적 카메라 렌더러 |
| `outputs/train/urdf_f/e2e_walk_v5/` | **최종 채택 정책** |
| `outputs/e2e_walk_v5_tracking.mp4` | 최종 보행 영상 |
| `docs/hardware_validation/1xx_*.md` | 단계별 상세 기록 |

**다음 문서**: doc 129(타당성 검증 통과 상세), doc 130(액추에이터 선정 기준).
