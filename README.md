# CHIRO Humanoid

CHIRO 휴머노이드 locomotion / motion tracking 실험을 위한 연구 프레임워크입니다.

이 저장소는 로봇 asset, 시뮬레이션 환경, controller/policy, objective, 학습
스크립트, 실험 결과를 서로 분리해서 관리하는 것을 목표로 합니다. 로봇 모델을
바꾸는 작업, 학습 objective를 바꾸는 작업, policy 구조를 바꾸는 작업, 실행
스크립트를 바꾸는 작업이 서로 섞이지 않도록 구성했습니다.

## 폴더 구조

```text
configs/             실험 설정 파일
docs/                실험 노트, 설계 문서, 분석 기록
envs/                시뮬레이션 환경, 로봇 XML, scene, mesh asset
outputs/             학습 결과, 평가 결과, 영상, 로그
policies/            robot별 controller / policy 관련 코드
reference_motions/   motion tracking에 사용할 reference motion 데이터
rewards/             imitation learning / reinforcement learning objective 코드
scripts/             학습, 평가, 렌더링 실행 스크립트
```

## 각 폴더 역할

### `configs/`

실험 설정을 저장하는 폴더입니다.

예를 들어 다음과 같은 내용을 넣는 용도입니다.

- 학습 hyperparameter
- objective weight 조합
- robot별 기본 설정
- 실험 버전별 config

현재는 CLI argument 기반 실행이 중심이지만, 실험이 많아지면 `configs/`에
YAML/JSON 파일을 두고 재현 가능한 실험 단위로 관리하면 됩니다.

### `docs/`

실험 과정과 판단 근거를 남기는 폴더입니다.

포트폴리오 관점에서는 단순히 “학습했다”보다, 어떤 문제가 있었고 왜 그런
설계를 했는지 남기는 것이 중요합니다. 따라서 다음 내용을 정리하는 용도로
사용합니다.

- 참고 논문 / 오픈소스 정리
- 실험별 실패 원인 분석
- objective 설계 이유
- reference motion 품질 검증 기록
- 최종 결과 요약

### `envs/`

시뮬레이션 환경과 로봇 asset을 관리하는 폴더입니다.

현재 구조는 다음과 같은 형태를 기준으로 합니다.

```text
envs/sim_env.py              Gymnasium 스타일 시뮬레이션 환경
envs/robots/<robot_name>/    robot XML, scene, mesh asset
envs/scenes/                 공통 scene 또는 환경 asset
```

로봇 모델, XML, scene, 지면, asset 경로처럼 “물리 환경 자체”와 관련된 변경은
이 폴더에서 관리합니다. policy나 objective 코드를 여기에 섞지 않는 것이
원칙입니다.

### `policies/`

로봇별 controller와 policy 관련 코드를 관리하는 폴더입니다.

현재 구조는 다음과 같습니다.

```text
policies/common/             여러 로봇이 공유할 수 있는 controller / mode 코드
policies/<robot_name>/       robot별 controller 코드
policies/registry.py         robot 이름으로 policy/controller를 불러오는 registry
```

새로운 휴머노이드 로봇을 추가한다면 `policies/<robot_name>/` 폴더를 만들고
registry에 연결하는 방식으로 확장합니다.

### `rewards/`

강화학습 또는 imitation learning에서 사용할 objective/reward를 관리하는
폴더입니다.

현재 구조는 다음과 같습니다.

```text
rewards/standing.py          posture stability objective
rewards/walking.py           locomotion objective
rewards/reference_walking.py motion reference tracking objective
```

새로운 실험을 할 때는 기존 objective를 직접 크게 고치기보다, 새로운 파일을
추가하거나 weight/config를 분리해서 비교할 수 있게 만드는 것이 좋습니다.

### `reference_motions/`

motion tracking 실험에서 입력으로 사용할 reference motion을 모아두는 폴더입니다.

현재 구조는 다음과 같습니다.

```text
reference_motions/raw/          외부에서 받은 원본 motion 파일
reference_motions/processed/    target robot / simulator에 맞게 가공한 motion 파일
reference_motions/metadata/     motion 후보, 출처, frame rate, joint mapping 기록
```

이 폴더는 “학습 결과”가 아니라 “학습 입력 데이터”를 관리하는 곳입니다. 따라서
motion imitation, reference tracking objective, motion replay 검증에 사용할
파일은 여기에 둡니다.

권장 관리 방식은 다음과 같습니다.

- `raw/`: 원본 파일을 최대한 그대로 보관합니다.
- `processed/`: target joint order, qpos/qvel, contact label, root heading 등을 맞춘 파일을 둡니다.
- `metadata/`: 어떤 데이터셋에서 왔는지, 어떤 전처리를 했는지, 어떤 실험에서 썼는지 기록합니다.

큰 motion 파일은 git에 올리지 않고, 필요한 경우 `docs/`에 출처와 재현 방법만
정리합니다.

### `scripts/`

실제로 실행하는 entrypoint를 모아둔 폴더입니다.

현재 포함된 스크립트는 다음과 같습니다.

```text
scripts/test_env.py                  scene/env 로딩 및 렌더링 확인
scripts/train.py                     policy 학습 실행
scripts/pretrain_motion_dagger.py    motion reference 기반 imitation pretrain
```

학습, 평가, 렌더링, 데이터 전처리처럼 “명령어로 실행하는 작업”은 이 폴더에
둡니다. 실험 코드가 길어질 경우 `scripts/ops/`, `scripts/eval/`처럼 하위 폴더로
나눌 수 있습니다.

### `outputs/`

학습 결과와 평가 산출물을 저장하는 폴더입니다.

현재 하위 폴더는 다음 기준으로 사용합니다.

```text
outputs/train/       학습 실행 결과, checkpoint, normalization 통계, run config
outputs/probes/      특정 policy/reference를 빠르게 확인한 rollout, 영상, metric
outputs/analysis/    torque, tracking error, contact 분석 plot / CSV
```

예상되는 산출물은 다음과 같습니다.

- policy checkpoint
- imitation pretrain 결과
- rollout 영상
- metric CSV
- torque / tracking error plot
- 실험별 summary JSON

`outputs/` 아래 파일은 기본적으로 git에 올리지 않도록 `.gitignore` 처리되어
있습니다. 대신 중요한 결과는 `docs/`에 요약하고, 필요한 영상이나 표만 별도로
선별해서 포트폴리오에 사용합니다.

## 빠른 환경 확인

```bash
python scripts/test_env.py \
  --robot <robot_name> \
  --render-mode rgb_array \
  --policy home \
  --base-controller home \
  --steps 2 \
  --sleep-sec 0
```

예상 결과:

- robot scene이 정상 로드됨
- observation이 생성됨
- RGB frame 렌더링이 정상 동작함

실시간 렌더링을 보고 싶으면 `render-mode`를 `human`으로 바꿉니다.

```bash
mjpython scripts/test_env.py \
  --robot <robot_name> \
  --render-mode human \
  --policy home \
  --base-controller home \
  --steps 3000 \
  --sleep-sec 0.02
```

## 학습 시작

CPU에서 아주 짧게 학습 실행 여부만 확인하는 명령입니다.

```bash
python scripts/train.py \
  --robot <robot_name> \
  --stage stand \
  --reward standing \
  --base-controller home \
  --run-name stand_smoke \
  --save-dir outputs/train \
  --total-timesteps 16 \
  --n-envs 1 \
  --n-steps 8 \
  --batch-size 8 \
  --n-epochs 1 \
  --device cpu \
  --no-vecnorm
```

실제 학습에서는 같은 명령에서 다음 값을 늘리면 됩니다.

- `--total-timesteps`
- `--n-envs`
- `--n-steps`
- `--batch-size`

## Motion Imitation 진입점

```bash
python scripts/pretrain_motion_dagger.py --help
```

reference motion 파일이 있을 때, policy를 imitation 기반으로 초기화하는 용도입니다.
이후 policy fine-tuning을 통해 안정성, 넘어짐 방지, contact mismatch 완화를
실험할 수 있습니다.

## 관리 원칙

- 로봇 XML / scene / mesh 변경은 `envs/`에서 관리합니다.
- controller / policy 구조 변경은 `policies/`에서 관리합니다.
- objective/reward 설계 변경은 `rewards/`에서 관리합니다.
- 실행 명령과 실험 entrypoint는 `scripts/`에 둡니다.
- motion tracking 입력 데이터는 `reference_motions/`에 둡니다.
- 실험 결과 파일은 `outputs/`에 저장하고, 중요한 해석만 `docs/`에 정리합니다.
- 큰 checkpoint, 영상, 로그 파일은 git에 올리지 않습니다.
