# Frame-normalized user-size contact PD Probe

## 목적

사용자 제공 발 크기 `70 x 120 x 40 mm`를 유지하면서, MJCF foot body 기준 좌우 동일 center로 정규화한 contact 모델을 검증한다.

이 실행은 `probe_pd_standing.py`의 자동 base-z 추정을 사용했다. box geom의 bounding sphere 때문에 base z가 과대 추정될 수 있어, 43번에서 geometry 분석값으로 다시 실행한다.

## 실행 명령

```bash
python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml --out-dir outputs/analysis/pd_standing_user_size_mass_contact_zero --doc-dir docs/hardware_validation --doc-name 42_user_size_mass_contact_zero_pd_probe.md
python3 scripts/plot_pd_standing.py --csv outputs/analysis/pd_standing_user_size_mass_contact_zero/neutral_pd_standing.csv --out-dir outputs/analysis/pd_standing_user_size_mass_contact_zero
```

## 조건

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml`
- 목표 joint pose: 전체 revolute joint `0.0 rad`
- 자동 추정 base z: `0.238197`
- 총질량: `9.211400 kg`
- contact halfsize: `[0.035, 0.06, 0.02] m`

## 결과

- 최종 base z: `2.494543`
- 최종 roll: `-1.790732` rad
- 최종 pitch: `0.559160` rad
- 최대 qvel norm: `757.324692`
- 최대 contact normal force: `2060.890093`
- 관측 최대 actuator torque: `100.000000` Nm

## 산출물

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_size_mass_contact_zero/neutral_pd_standing.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_size_mass_contact_zero/neutral_pd_standing_summary.json`
- Plot: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_size_mass_contact_zero/neutral_pd_standing_plot.png`

## 해석

자동 base-z가 geometry 분석값보다 높게 잡혔기 때문에 이 실행만으로 standing 실패를 판단하지 않는다. 동일 모델을 43번에서 base z `0.057284`로 다시 검증한다.
