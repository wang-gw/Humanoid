# 하드웨어 검증 로그

이 폴더는 보행 가능한 휴머노이드 제작을 위한 하드웨어/RL 가능성 검증 기록을 보관한다.

목표는 단순히 "시뮬레이션에서 걷는다"를 확인하는 것이 아니라, 걷지 못했을 때 원인을 하드웨어 형상, actuator 가정, 물리 모델, 제어기, reward 중 어디에서 찾아야 하는지 분리할 수 있게 만드는 것이다.

## 검증 단계

1. 모델 감사
   - URDF/xacro 구조, link mass, inertia, joint axis, joint limit, collision geometry 확인
2. MuJoCo asset 준비
   - RL 실험에 사용할 MuJoCo 모델 로드, actuator, contact, keyframe 정리
3. 정적/준정적 probe
   - standing, squat, weight shift, one-leg support 테스트
4. 토크와 접촉 분석
   - joint torque, joint velocity, contact force, base pose, saturation 기록
5. RL 준비 여부 판단
   - standing/stepping/walking 학습으로 넘어갈지, 하드웨어 형상을 수정할지 결정
6. RL 보행 실험
   - standing, balance, stepping, walking 순서로 학습 난이도를 올림
7. 모터와 감속기 구체화
   - RL 및 probe 결과를 바탕으로 최종 모터/감속기 후보를 좁힘

## 단계별 필수 기록

- 시작 가정
- 사용한 모델/커밋/스크립트
- 실행 명령
- 산출물 위치
- 주요 수치
- 판단 결과
- 다음 조치

## 신모델 (urdf_f_v2)

새 CAD(`URDF_description/`) 기반 신모델의 변환·학습 기록은 별도 폴더에 동일 형식으로
분리 정리했다: **`urdf_f_v2/README.md`** (변환 → 매핑 → armature 수정 → 안정 스탠딩 →
보행 학습 → 코드 격리).

## 최근 상태

- 새 STEP/STL 파일 확인: `24_step_and_link_stl_check.md`
- 새 STL 기반 별도 모델 생성: `25_link_stl_named_model.md`
- 새 모델 PD standing 결과: `26_link_named_pose_pd_standing_probe.md`
- 현재 판단과 다음 단계: `27_link_stl_current_decision.md`
- 좌우 foot/sole alignment 감사: `28_kinematic_alignment_audit.md`
- body-centered contact 진단 모델: `29_body_centered_foot_contact_variant.md`
- contact 진단 PD probe: `30_body_contact_pose_pd_standing_probe.md`, `31_body_contact_pose_pd_standing_probe_basez.md`
- contact 진단 후 판단: `32_contact_diagnostic_decision.md`
- STEP assembly transform 감사: `33_step_assembly_transform_audit.md`
- STEP 기반 입력 초안: `34_step_based_input_prefill.md`
- foot contact config 적용 모델: `35_apply_step_contact_config.md`
- STEP contact 모델 PD probe: `36_step_contact_pose_pd_standing_probe.md`
- 현재 실행 gate: `37_current_execution_gate.md`
- 사용자 제공 mass/contact 적용 모델: `38_user_mass_contact_model.md`
- 사용자 제공 0 rad standing PD probe: `39_user_mass_contact_zero_pd_probe.md`
- 사용자 입력 반영 후 판단: `40_user_inputs_decision.md`
- foot contact frame 후보 감사: `41_foot_contact_frame_candidates.md`
- frame-normalized contact PD probe: `42_user_size_mass_contact_zero_pd_probe.md`, `43_user_size_mass_contact_zero_pd_probe_basez.md`
- frame-normalized contact 판단: `44_frame_normalized_contact_decision.md`
- joint sign response 감사: `45_joint_sign_response_audit.md`
- actuator limit sweep: `46_actuator_limit_sweep.md`
- 정확한 base height 산정 수정: `47_exact_base_height_correction.md`
- 정확한 foot contact 기준 actuator sweep: `48_exact_base_actuator_sweep.md`
- component inertia 합산 감사: `49_component_inertia_aggregation_audit.md`
- component inertia transform 감사: `50_component_inertia_transformed_audit.md`
- component inertia 실험 모델: `51_component_inertia_experimental_variant.md`
- inertia 실험 모델 standing 비교: `52_inertia_variant_probe_comparison.md`
- quasi-static standing pose 탐색: `53_quasistatic_standing_pose_search.md`
- quasi-static pose PD probe: `54_quasistatic_pose_pd_probe.md`
- stabilizer sweep: `55_stabilized_standing_sweep.md`
- actuator limit joint clamp 재검증: `56_stabilizer_actuator_limit_recheck.md`
- roll impulse response 감사: `57_roll_impulse_response.md`
- corner foot contact variant: `58_corner_foot_contact_variant.md`
- corner contact roll impulse 비교: `59_corner_contact_roll_impulse_comparison.md`
- contact-free roll impulse 감사: `60_roll_free_impulse_response.md`
- CAD-MuJoCo 축 대응 및 edge contact 감사: `61_cad_mujoco_axis_and_edge_contact_audit.md`
- CAD assembly edge contact probe: `62_cad_assembly_edge_contact_probe.md`
- 사용자 제공 contact pad 변형 검증: `63_user_pad_contact_variant_probe.md`
- foot local Z Toe/Heel 확정 contact 모델: `64_toeheel_z_contact_axis_confirmation.md`
- Toe/Heel Z contact 모델 시각화: `65_toeheel_z_contact_visualization.md`
- PD standing 전체 로봇 시뮬레이션 시각화: `66_pd_standing_full_robot_visualization.md`
- visual mesh overlap 감사: `67_visual_mesh_overlap_audit.md`
- STEP visual assembly 및 hybrid visual 모델: `68_step_visual_assembly_and_hybrid_model.md`
- standing 전략 및 pose/contact 재검증: `69_next_standing_strategy_and_pose_contact_probe.md`
- ankle/hip axis response 감사: `70_ankle_axis_response_audit.md`
- axis-aware standing 과정 영상 기록: `71_axis_aware_standing_video.md`
- pose support 및 soft-contact 비교 검증: `72_pose_support_and_soft_contact_probe.md`
- gravity bias torque 감사: `73_gravity_bias_torque_audit.md`
- all-joint free impulse response 감사: `74_all_joint_free_impulse_response.md`
- actuator dynamics 변형 모델 probe: `75_actuator_dynamics_variant_probe.md`
- actuator dynamics 기준 pose/gain/stabilizer 재탐색: `76_actuator_dynamics_pose_gain_stabilizer_search.md`
- dynamic standing pose search: `77_dynamic_standing_pose_search.md`
- 5초 dynamic pose/controller search: `78_five_second_dynamic_pose_controller_search.md`
- weight shift sanity check: `79_weight_shift_sanity_check.md`
- multipoint contact weight shift probe: `80_multipoint_contact_weight_shift_probe.md`
- weight shift pose/trajectory search: `81_weight_shift_pose_trajectory_search.md`
- transition-aware weight shift milestone: `82_transition_aware_weight_shift_milestone.md`
- weight shift failure torque/contact audit: `83_weight_shift_failure_torque_contact_audit.md`
- force ratio feedback controller probe: `84_force_ratio_feedback_controller_probe.md`
- contact-constrained transition pose search: `85_contact_constrained_transition_pose_search.md`
- right foot unload probe: `86_right_foot_unload_probe.md`
- COM support polygon audit: `87_com_support_polygon_audit.md`
- virtual design variant probe: `88_virtual_design_variant_probe.md`
- virtual sole width sweep: `89_virtual_sole_width_sweep.md`
- foot STL vs contact footprint audit: `90_foot_stl_contact_footprint_audit.md`
- STL footprint contact gate recheck and lift probe: `91_stl_footprint_contact_gate_recheck_and_lift_probe.md`
- right foot lift trajectory probe: `92_right_foot_lift_trajectory_probe.md`
- clearance duration trajectory search: `93_clearance_duration_trajectory_search.md`
- full-body lift trajectory search: `94_fullbody_lift_trajectory_search.md`
- foot-frame IK lift probe: `95_foot_frame_ik_lift_probe.md`
- one-leg support gate probe: `96_one_leg_support_gate_probe.md`
- unload + small IK lift probe: `97_unload_plus_small_lift_probe.md`
- virtual support polygon sensitivity: `98_virtual_support_polygon_sensitivity.md`
- virtual COM sensitivity: `99_virtual_com_sensitivity.md`
- COM Y005 lift gate probe: `100_com_y005_lift_gate_probe.md`
- closed-loop lift probe: `101_closed_loop_lift_probe.md`
- task-space lift probe: `102_task_space_lift_probe.md`
- RL readiness and hardware decision: `103_rl_readiness_and_hardware_decision.md`
- URDF_F RL environment preparation: `104_urdf_f_rl_environment_preparation.md`
- standing RL first training: `105_standing_rl_first_training.md`
- weight shift RL training: `106_weight_shift_rl_training.md`
- right unload RL training: `107_right_unload_rl_training.md`
- right clearance RL training: `108_right_clearance_rl_training.md`
- gated clearance curriculum: `109_gated_clearance_curriculum.md`
- gated clearance push to 1.2mm: `110_gated_clearance_push_12mm.md`
- roll-guarded clearance: `111_roll_guarded_clearance.md`
- roll-guard 0.14 clearance pass: `112_roll_guard_014_clearance_pass.md`
- right step return: `113_right_step_return.md`
- right step sequence evaluation: `114_right_step_sequence_evaluation.md`
- left mirror sequence precheck: `115_left_mirror_sequence_precheck.md`
- left sequence environment preparation: `116_left_sequence_env_preparation.md`
- weight shift right RL training: `117_weight_shift_right_rl_training.md`
- left unload RL training: `118_left_unload_rl_training.md`
- left clearance RL training: `119_left_clearance_rl_training.md`
- left return RL training: `120_left_return_rl_training.md`
- left step sequence evaluation: `121_left_step_sequence_evaluation.md`
- contact asymmetry fix and left sequence v5: `122_contact_asymmetry_fix_and_left_sequence_v5.md`
- bilateral alternating walking (3 cycles, 2020 steps): `123_bilateral_alternating_walking.md`
