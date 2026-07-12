# outputs/urdf_f_v2 — 신모델 과정기록

신모델(`urdf_f_v2`)의 **과정 산출물**(렌더 영상, 토크 프로파일, 분석 결과 등)을 모아두는
폴더다. 학습 결과(정책·체크포인트)는 `outputs/train/urdf_f_v2/`에 별도 보관한다.

구모델은 이런 산출물을 `outputs/` 루트와 `outputs/analysis/`에 두었으나, 신모델은
한 폴더로 분리한다.

## 현재 보관물

| 파일 | 설명 |
|---|---|
| `e2e_walk_v2model_standing.mp4` | v2 정책 50초 안정 스탠딩 (측면 추적 카메라, +7.2cm) |

## 생성 방법

```bash
# 렌더 영상 (v2 전용 렌더러, GaitWalkingV2Env 사용)
python3 scripts/render_urdf_f_v2_walk.py \
  --policy outputs/train/urdf_f_v2/<run>/checkpoints/ppo_<N>_steps.zip \
  --vecnorm outputs/train/urdf_f_v2/<run>/checkpoints/vecnorm_<N>.pkl \
  --out outputs/urdf_f_v2/<name>.mp4
```

과정 기록 문서는 `docs/hardware_validation/urdf_f_v2/` 참조.
