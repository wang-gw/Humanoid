from __future__ import annotations

from pathlib import Path

import numpy as np

from .walking import WalkingReward


class ReferenceWalkingReward(WalkingReward):
    """Walking reward with an optional cyclic joint-reference tracking term."""

    def __init__(
        self,
        motion_path: str,
        w_ref_joint: float = 0.5,
        w_ref_joint_mse: float = 0.0,
        ref_joint_sigma: float = 0.05,
        ref_joint_scope: str = "lower_body",
        w_foot_slip: float = 0.0,
        w_pelvis_lateral: float = 0.0,
        w_stance_lateral: float = 0.0,
        stance_lateral_margin: float = 0.08,
        w_ref_contact: float = 0.0,
        w_ref_stance_miss: float = 0.0,
        w_ref_swing_extra_contact: float = 0.0,
        w_ref_right_stance_miss: float = 0.0,
        w_ref_right_swing_clearance: float = 0.0,
        ref_right_swing_clearance_target: float = 0.09,
        w_ref_root_vel: float = 0.0,
        ref_root_vel_sigma: float = 0.05,
        w_ref_action: float = 0.0,
        w_ref_action_mse: float = 0.0,
        ref_action_sigma: float = 0.05,
        **kwargs: float,
    ) -> None:
        super().__init__(**kwargs)
        self.motion_path = str(motion_path)
        self.w_ref_joint = float(w_ref_joint)
        self.w_ref_joint_mse = float(w_ref_joint_mse)
        self.ref_joint_sigma = max(float(ref_joint_sigma), 1e-6)
        self.ref_joint_scope = str(ref_joint_scope).strip().lower()
        self.w_foot_slip = float(w_foot_slip)
        self.w_pelvis_lateral = float(w_pelvis_lateral)
        self.w_stance_lateral = float(w_stance_lateral)
        self.stance_lateral_margin = max(float(stance_lateral_margin), 0.0)
        self.w_ref_contact = float(w_ref_contact)
        self.w_ref_stance_miss = float(w_ref_stance_miss)
        self.w_ref_swing_extra_contact = float(w_ref_swing_extra_contact)
        self.w_ref_right_stance_miss = float(w_ref_right_stance_miss)
        self.w_ref_right_swing_clearance = float(w_ref_right_swing_clearance)
        self.ref_right_swing_clearance_target = max(float(ref_right_swing_clearance_target), 1e-6)
        self.w_ref_root_vel = float(w_ref_root_vel)
        self.ref_root_vel_sigma = max(float(ref_root_vel_sigma), 1e-6)
        self.w_ref_action = float(w_ref_action)
        self.w_ref_action_mse = float(w_ref_action_mse)
        self.ref_action_sigma = max(float(ref_action_sigma), 1e-6)
        self._fps = 50.0
        self._joint_pos = np.zeros((0, 0), dtype=np.float32)
        self._contact_labels = np.zeros((0, 2), dtype=np.float32)
        self._root_pos = np.zeros((0, 3), dtype=np.float32)
        self._joint_map: list[tuple[int, int]] = []
        self._foot_site_ids: dict[str, int] = {}
        self._prev_foot_pos: dict[str, np.ndarray] = {}
        self._loaded = False

    def _include_joint(self, name: str) -> bool:
        if self.ref_joint_scope in {"all", ""}:
            return True
        if self.ref_joint_scope in {"lower", "lower_body", "legs_waist"}:
            return (
                name.startswith("left_hip_")
                or name.startswith("left_knee")
                or name.startswith("left_ankle")
                or name.startswith("right_hip_")
                or name.startswith("right_knee")
                or name.startswith("right_ankle")
                or name.startswith("waist_")
            )
        if self.ref_joint_scope in {"legs_only", "legs"}:
            return (
                name.startswith("left_hip_")
                or name.startswith("left_knee")
                or name.startswith("left_ankle")
                or name.startswith("right_hip_")
                or name.startswith("right_knee")
                or name.startswith("right_ankle")
            )
        raise ValueError(
            f"Unknown ref_joint_scope={self.ref_joint_scope!r}. "
            "Use all, lower_body, or legs_only."
        )

    def _load_motion(self, env: object) -> None:
        path = Path(self.motion_path)
        if not path.exists():
            raise FileNotFoundError(f"Reference motion not found: {path}")

        data = np.load(path, allow_pickle=False)
        self._fps = float(np.asarray(data["fps"]).reshape(()))
        self._joint_pos = np.asarray(data["joint_pos"], dtype=np.float32)
        if "contact_labels" in data:
            contacts = np.asarray(data["contact_labels"], dtype=np.float32)
            if contacts.ndim == 2 and contacts.shape[1] >= 2:
                self._contact_labels = contacts[:, :2].copy()
        if "root_pos" in data:
            root_pos = np.asarray(data["root_pos"], dtype=np.float32)
            if root_pos.ndim == 2 and root_pos.shape[1] >= 3:
                self._root_pos = root_pos[:, :3].copy()
        joint_names = [str(v) for v in np.asarray(data["joint_names"]).tolist()]
        ref_index = {name: i for i, name in enumerate(joint_names)}

        actuator_names: list[str] = []
        for i in range(int(env.model.nu)):
            name = env.model.actuator(i).name
            actuator_names.append(str(name))

        self._joint_map = [
            (act_i, ref_index[name])
            for act_i, name in enumerate(actuator_names)
            if name in ref_index and self._include_joint(name)
        ]
        if not self._joint_map:
            raise ValueError(
                f"Reference motion has no joints matching model actuators: {path}"
            )
        self._foot_site_ids = {}
        for side in ("left", "right"):
            try:
                site_id = int(
                    env._mujoco.mj_name2id(
                        env.model,
                        env._mujoco.mjtObj.mjOBJ_SITE,
                        f"{side}_foot",
                    )
                )
            except Exception:
                site_id = -1
            if site_id >= 0:
                self._foot_site_ids[side] = site_id
        self._loaded = True

    def reset(self, env: object) -> None:
        super().reset(env)
        if not self._loaded:
            self._load_motion(env)
        self._prev_foot_pos = self._current_foot_positions(env)

    def _current_foot_positions(self, env: object) -> dict[str, np.ndarray]:
        out: dict[str, np.ndarray] = {}
        for side, site_id in self._foot_site_ids.items():
            out[side] = np.asarray(env.data.site_xpos[site_id], dtype=np.float32).copy()
        return out

    def _reference_frame(self, env: object) -> np.ndarray:
        if self._joint_pos.shape[0] == 0:
            return np.zeros(0, dtype=np.float32)
        get_reference_time = getattr(env, "get_reference_time", None)
        ref_time = float(get_reference_time()) if callable(get_reference_time) else float(env.data.time)
        frame = int((ref_time * self._fps) % self._joint_pos.shape[0])
        return self._joint_pos[frame]

    def _reference_contact(self, env: object) -> np.ndarray:
        if self._contact_labels.shape[0] == 0:
            return np.zeros(2, dtype=np.float32)
        get_reference_time = getattr(env, "get_reference_time", None)
        ref_time = float(get_reference_time()) if callable(get_reference_time) else float(env.data.time)
        frame = int((ref_time * self._fps) % self._contact_labels.shape[0])
        return self._contact_labels[frame].astype(np.float32)

    def _reference_root_velocity(self, env: object) -> np.ndarray:
        if self._root_pos.shape[0] < 2:
            return np.zeros(3, dtype=np.float32)
        get_reference_time = getattr(env, "get_reference_time", None)
        ref_time = float(get_reference_time()) if callable(get_reference_time) else float(env.data.time)
        ref_time_scale = abs(float(getattr(env, "reference_time_scale", 1.0)))
        n = int(self._root_pos.shape[0])
        frame = int((ref_time * self._fps) % max(n - 1, 1))
        next_frame = min(frame + 1, n - 1)
        return (
            (self._root_pos[next_frame] - self._root_pos[frame]) * self._fps * ref_time_scale
        ).astype(np.float32)

    def compute(self, env: object, action: np.ndarray) -> tuple[float, dict[str, float]]:
        if not self._loaded:
            self._load_motion(env)

        reward, info = super().compute(env, action)
        ref = self._reference_frame(env)
        joint_pos = np.asarray(env.data.qpos[7:], dtype=np.float32)
        nominal = np.asarray(getattr(env, "nominal_action", np.zeros(0)), dtype=np.float32)
        use_nominal_ref = (
            bool(getattr(env, "residual_action", False))
            and str(getattr(env, "base_controller", "")) == "reference_motion"
            and nominal.shape[0] > 0
        )

        errors: list[float] = []
        for act_i, ref_i in self._joint_map:
            if act_i >= joint_pos.shape[0]:
                continue
            if use_nominal_ref and act_i < nominal.shape[0]:
                errors.append(float(joint_pos[act_i] - nominal[act_i]))
            elif ref_i < ref.shape[0]:
                errors.append(float(joint_pos[act_i] - ref[ref_i]))
        if errors:
            mse = float(np.mean(np.square(np.asarray(errors, dtype=np.float32))))
        else:
            mse = 0.0

        r_ref_joint = float(np.exp(-mse / self.ref_joint_sigma))
        reward = float(reward + self.w_ref_joint * r_ref_joint - self.w_ref_joint_mse * mse)

        foot_pos = self._current_foot_positions(env)
        contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)
        contact_by_side = {
            "left": float(contacts[0]) if contacts.shape[0] > 0 else 0.0,
            "right": float(contacts[1]) if contacts.shape[0] > 1 else 0.0,
        }
        dt = max(float(getattr(env, "frame_skip", 1)) * float(env.model.opt.timestep), 1e-6)
        slip_terms: list[float] = []
        for side, pos in foot_pos.items():
            prev = self._prev_foot_pos.get(side)
            if prev is None or contact_by_side.get(side, 0.0) <= 0.5:
                continue
            horizontal_vel = (pos[:2] - prev[:2]) / dt
            slip_terms.append(float(np.dot(horizontal_vel, horizontal_vel)))
        p_foot_slip = float(np.mean(slip_terms)) if slip_terms else 0.0

        p_pelvis_lateral = 0.0
        if len(foot_pos) >= 2:
            foot_mid_y = float(np.mean([pos[1] for pos in foot_pos.values()]))
            pelvis_y = float(env.data.xpos[env._trunk_body_id][1])
            p_pelvis_lateral = float((pelvis_y - foot_mid_y) ** 2)

        ref_contact = self._reference_contact(env)
        p_stance_lateral = 0.0
        if len(foot_pos) >= 2:
            pelvis_y = float(env.data.xpos[env._trunk_body_id][1])
            stance_ys = [
                float(foot_pos[side][1])
                for side in ("left", "right")
                if side in foot_pos and contact_by_side.get(side, 0.0) > 0.5
            ]
            if not stance_ys and ref_contact.shape[0] >= 2:
                stance_ys = [
                    float(foot_pos[side][1])
                    for idx, side in enumerate(("left", "right"))
                    if side in foot_pos and float(ref_contact[idx]) > 0.5
                ]
            if stance_ys:
                low = min(stance_ys) - self.stance_lateral_margin
                high = max(stance_ys) + self.stance_lateral_margin
                outside = max(low - pelvis_y, pelvis_y - high, 0.0)
                p_stance_lateral = float(outside**2)

        reward = float(
            reward
            - self.w_foot_slip * p_foot_slip
            - self.w_pelvis_lateral * p_pelvis_lateral
            - self.w_stance_lateral * p_stance_lateral
        )
        env_contact = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
        contact_err = 0.0
        r_ref_contact = 0.0
        p_ref_stance_miss = 0.0
        p_ref_swing_extra_contact = 0.0
        p_ref_right_stance_miss = 0.0
        r_ref_right_swing_clearance = 0.0
        if ref_contact.shape[0] >= 2 and self._contact_labels.shape[0] > 0:
            contact_err = float(np.mean((env_contact[:2] - ref_contact[:2]) ** 2))
            r_ref_contact = float(1.0 - contact_err)
            reward = float(reward + self.w_ref_contact * r_ref_contact)
            p_ref_stance_miss = float(np.mean(np.maximum(ref_contact[:2] - env_contact[:2], 0.0)))
            p_ref_swing_extra_contact = float(np.mean(np.maximum(env_contact[:2] - ref_contact[:2], 0.0)))
            p_ref_right_stance_miss = float(max(float(ref_contact[1] - env_contact[1]), 0.0))
            right_ref_swing = float(np.clip(1.0 - ref_contact[1], 0.0, 1.0))
            if right_ref_swing > 0.0 and "right" in foot_pos:
                right_air = float(np.clip(1.0 - env_contact[1], 0.0, 1.0))
                right_z = float(foot_pos["right"][2])
                r_ref_right_swing_clearance = float(
                    right_ref_swing
                    * right_air
                    * np.clip(right_z / self.ref_right_swing_clearance_target, 0.0, 1.0)
                )
            reward = float(
                reward
                - self.w_ref_stance_miss * p_ref_stance_miss
                - self.w_ref_swing_extra_contact * p_ref_swing_extra_contact
                - self.w_ref_right_stance_miss * p_ref_right_stance_miss
                + self.w_ref_right_swing_clearance * r_ref_right_swing_clearance
            )
        ref_root_vel = self._reference_root_velocity(env)
        root_vel_err = 0.0
        r_ref_root_vel = 0.0
        if self._root_pos.shape[0] >= 2:
            base_lin_vel = np.asarray(env.data.qvel[0:3], dtype=np.float32)
            root_vel_err = float((base_lin_vel[0] - ref_root_vel[0]) ** 2)
            r_ref_root_vel = float(np.exp(-root_vel_err / self.ref_root_vel_sigma))
            reward = float(reward + self.w_ref_root_vel * r_ref_root_vel)
        ref_action = np.asarray(env.get_reference_action(), dtype=np.float32)
        action_arr = np.asarray(action, dtype=np.float32)
        n_action = min(ref_action.shape[0], action_arr.shape[0])
        ref_action_mse = 0.0
        r_ref_action = 0.0
        if n_action > 0:
            action_err = action_arr[:n_action] - ref_action[:n_action]
            ref_action_mse = float(np.mean(np.square(action_err)))
            r_ref_action = float(np.exp(-ref_action_mse / self.ref_action_sigma))
            reward = float(
                reward
                + self.w_ref_action * r_ref_action
                - self.w_ref_action_mse * ref_action_mse
            )
        self._prev_foot_pos = foot_pos
        info.update(
            {
                "reward_total": reward,
                "reward_ref_joint": r_ref_joint,
                "penalty_ref_joint_mse": mse,
                "reward_ref_contact": r_ref_contact,
                "reward_ref_root_vel": r_ref_root_vel,
                "reward_ref_action": r_ref_action,
                "ref_joint_mse": mse,
                "ref_action_mse": ref_action_mse,
                "ref_contact_mse": contact_err,
                "penalty_ref_stance_miss": p_ref_stance_miss,
                "penalty_ref_swing_extra_contact": p_ref_swing_extra_contact,
                "penalty_ref_right_stance_miss": p_ref_right_stance_miss,
                "reward_ref_right_swing_clearance": r_ref_right_swing_clearance,
                "ref_root_vel_error": root_vel_err,
                "ref_root_vx": float(ref_root_vel[0]),
                "penalty_foot_slip": p_foot_slip,
                "penalty_pelvis_lateral": p_pelvis_lateral,
                "penalty_stance_lateral": p_stance_lateral,
            }
        )
        return reward, info
