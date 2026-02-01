# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Strategy classes for quadcopter environment rewards, observations, and resets.
"""

from __future__ import annotations

import torch
import numpy as np
from typing import TYPE_CHECKING, Dict, Optional

from isaaclab.utils.math import (
    subtract_frame_transforms,
    quat_from_euler_xyz,
    euler_xyz_from_quat,
)

if TYPE_CHECKING:
    from .quadcopter_env import QuadcopterEnv

D2R = np.pi / 180.0
R2D = 180.0 / np.pi


# ==============================================================
# Backward-compat stub  –  original gate-racing strategy.
# ==============================================================
class DefaultQuadcopterStrategy:
    """Placeholder.  Original gate-racing strategy – not functional without gate USD."""

    def __init__(self, env: "QuadcopterEnv"):
        raise NotImplementedError(
            "DefaultQuadcopterStrategy requires gate infrastructure that has been "
            "removed.  Use Mark4HoverLandStrategy instead."
        )


# ==============================================================
# Mark4 Hover / Land strategy
# ==============================================================
class Mark4HoverLandStrategy:
    """Hover and land policy training for the Mark4 7-inch quadcopter.

    Reward structure (hover mode)
    -----------------------------
    * ``position``   – negative exponential of squared position error to target.
    * ``velocity``   – penalty for any translational speed (want stillness).
    * ``orientation``– penalty for deviations from level (roll / pitch).
    * ``ang_vel``    – penalty for angular rate magnitude.
    * ``action_smooth`` – penalty for action change between consecutive steps.
    * ``crash``      – large one-time penalty on ground contact.
    * ``death``      – applied at episode termination (died).

    Reward structure (land mode)
    ----------------------------
    All of the above, plus:
    * ``altitude``   – reward proportional to descent progress toward the ground.
    * ``land_bonus`` – large one-shot reward when altitude < threshold AND speed low.

    Observations (17 dims)
    -----------------------
    [pos_rel(3), vel_b(3), ang_vel_b(3), euler(3), target_pos(3), prev_actions(4) ... ]
    keeping it minimal and body-frame-friendly for sim-to-real.
    """

    def __init__(self, env: "QuadcopterEnv"):
        self.env       = env
        self.device    = env.device
        self.num_envs  = env.num_envs
        self.cfg       = env.cfg

        # Merge user-supplied reward scales with defaults
        self.rew = dict(self._DEFAULT_REWARD_SCALES)
        if hasattr(env, "rew"):
            self.rew.update(env.rew)

        # Task mode
        self.task_mode = getattr(self.cfg, "task_mode", "hover")   # "hover" | "land"

        # Land-mode constants
        self._land_altitude_threshold = 0.12   # m – "on the ground"
        self._land_speed_threshold    = 0.3    # m/s – "nearly stopped"

        # ---------------------------------------------------------------
        # Target position(s).
        # For hover: single fixed point (or randomized per reset).
        # For land:  target is [x, y, 0] on the ground, set at reset.
        # ---------------------------------------------------------------
        # Default hover target  –  directly above origin at 1.5 m
        self._hover_target = torch.tensor([0.0, 0.0, 1.5], device=self.device)

        # ---------------------------------------------------------------
        # Landing success flag (read by env._get_dones)
        # ---------------------------------------------------------------
        self.env._landed_success = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

        # ---------------------------------------------------------------
        # Episode-sum logging
        # ---------------------------------------------------------------
        if self.cfg.is_train:
            # Derive loggable keys from reward scales (strip _reward_scale / _cost)
            log_keys = []
            for k in self.rew:
                if k == "death_cost":
                    continue
                log_keys.append(k.replace("_reward_scale", ""))
            self._episode_sums = {
                key: torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
                for key in log_keys
            }

        # ---------------------------------------------------------------
        # Initialise fixed (non-randomised) per-env parameters
        # ---------------------------------------------------------------
        self._set_nominal_params()

    # ------------------------------------------------------------------
    # Parameter initialisation
    # ------------------------------------------------------------------
    def _set_nominal_params(self):
        """Write nominal parameter values into per-env tensors."""
        env = self.env
        env._K_aero[:, :2] = env._k_aero_xy_value
        env._K_aero[:, 2]  = env._k_aero_z_value

        env._kp_omega[:, :2] = env._kp_omega_rp_value
        env._ki_omega[:, :2] = env._ki_omega_rp_value
        env._kd_omega[:, :2] = env._kd_omega_rp_value

        env._kp_omega[:, 2] = env._kp_omega_y_value
        env._ki_omega[:, 2] = env._ki_omega_y_value
        env._kd_omega[:, 2] = env._kd_omega_y_value

        env._tau_m[:] = env._tau_m_value
        env._thrust_to_weight[:] = env._twr_value

    # ==================================================================
    # REWARDS
    # ==================================================================
    def get_rewards(self) -> torch.Tensor:
        env = self.env
        robot = env._robot

        # ---------------------------------------------------------------
        # Current drone state
        # ---------------------------------------------------------------
        pos_w     = robot.data.root_link_pos_w          # (N,3)
        vel_w     = robot.data.root_com_lin_vel_w       # (N,3)
        ang_vel_b = robot.data.root_ang_vel_b           # (N,3)
        quat_w    = robot.data.root_quat_w              # (N,4)

        euler = euler_xyz_from_quat(quat_w)             # tuple of 3 tensors (N,)
        roll, pitch = euler[0], euler[1]

        # ---------------------------------------------------------------
        # 1. Position error  –  exponential bell around target
        # ---------------------------------------------------------------
        pos_err_sq  = torch.sum((pos_w - env._desired_pos_w) ** 2, dim=1)   # (N,)
        pos_reward  = torch.exp(-pos_err_sq / 0.5)      # σ² = 0.25 m²  →  half-width ~0.5 m

        # ---------------------------------------------------------------
        # 2. Velocity penalty  –  want the drone still
        # ---------------------------------------------------------------
        speed_sq    = torch.sum(vel_w ** 2, dim=1)
        vel_penalty = torch.clamp(speed_sq, 0.0, 10.0)  # cap to avoid huge spikes early

        # ---------------------------------------------------------------
        # 3. Orientation penalty  –  penalise roll & pitch deviation
        # ---------------------------------------------------------------
        orient_penalty = torch.clamp(roll ** 2 + pitch ** 2, 0.0, 4.0)

        # ---------------------------------------------------------------
        # 4. Angular-velocity penalty
        # ---------------------------------------------------------------
        ang_vel_penalty = torch.clamp(torch.sum(ang_vel_b ** 2, dim=1), 0.0, 10.0)

        # ---------------------------------------------------------------
        # 5. Action-smoothness penalty
        # ---------------------------------------------------------------
        action_diff     = torch.sum((env._actions - env._previous_actions) ** 2, dim=1)
        smooth_penalty  = torch.clamp(action_diff, 0.0, 4.0)

        # ---------------------------------------------------------------
        # 6. Crash detection (ground contact accumulator)
        # ---------------------------------------------------------------
        contact_forces = env._contact_sensor.data.net_forces_w           # (N, 1, 3) or (N, 3)
        if contact_forces.dim() == 3:
            contact_forces = contact_forces.squeeze(1)
        hit = (torch.norm(contact_forces, dim=-1) > 1e-8).int()
        # Only count after initial settling
        mask = (env.episode_length_buf > 100).int()
        env._crashed = env._crashed + hit * mask

        crash_penalty = (env._crashed > 0).float()

        # ---------------------------------------------------------------
        # Assemble base rewards (shared by hover and land)
        # ---------------------------------------------------------------
        rewards_dict: Dict[str, torch.Tensor] = {
            "position":      pos_reward * self.rew.get("position_reward_scale", 1.0),
            "velocity":     -vel_penalty * self.rew.get("velocity_reward_scale", 1.0),
            "orientation":  -orient_penalty * self.rew.get("orientation_reward_scale", 1.0),
            "ang_vel":      -ang_vel_penalty * self.rew.get("ang_vel_reward_scale", 1.0),
            "action_smooth":-smooth_penalty * self.rew.get("action_smooth_reward_scale", 1.0),
            "crash":        -crash_penalty * self.rew.get("crash_reward_scale", 1.0),
        }

        # ---------------------------------------------------------------
        # Land-specific rewards
        # ---------------------------------------------------------------
        if self.task_mode == "land":
            alt = pos_w[:, 2]

            # Altitude reward: linear ramp from spawn height down to ground.
            # Spawn height is stored at reset; reward = 1 - (alt / spawn_alt), clamped.
            alt_progress = torch.clamp(1.0 - alt / self._spawn_altitude, 0.0, 1.0)
            rewards_dict["altitude"] = alt_progress

            # Landing bonus: one-shot when on ground and nearly stopped
            speed = torch.sqrt(speed_sq)
            landed = (alt < self._land_altitude_threshold) & (speed < self._land_speed_threshold)
            # Only fire once per episode: mask out envs that already landed
            just_landed = landed & (~env._landed_success)
            env._landed_success = env._landed_success | landed
            rewards_dict["land_bonus"] = just_landed.float()

        # ---------------------------------------------------------------
        # Scale and sum
        # ---------------------------------------------------------------
        scaled = {}
        for key, val in rewards_dict.items():
            scale_key = key + "_reward_scale"
            scale = self.rew.get(scale_key, 1.0)
            scaled[key] = val * scale

        reward = torch.stack(list(scaled.values()), dim=0).sum(dim=0)   # (N,)

        # Death cost on terminated episodes
        reward = torch.where(
            env.reset_terminated,
            torch.full_like(reward, self.rew["death_cost"]),
            reward,
        )

        # ---------------------------------------------------------------
        # Episode-sum logging
        # ---------------------------------------------------------------
        if self.cfg.is_train:
            for key, val in scaled.items():
                if key in self._episode_sums:
                    self._episode_sums[key] += val

        return reward

    # ==================================================================
    # OBSERVATIONS  –  17 dims, body-frame friendly
    # ==================================================================
    def get_observations(self) -> Dict[str, torch.Tensor]:
        robot = self.env._robot

        # Position relative to target  (world frame)
        pos_rel   = robot.data.root_link_pos_w - self.env._desired_pos_w   # (N,3)

        # Linear velocity in body frame
        vel_b     = robot.data.root_com_lin_vel_b                           # (N,3)

        # Angular velocity in body frame
        ang_vel_b = robot.data.root_ang_vel_b                               # (N,3)

        # Euler angles (roll, pitch, yaw)
        euler     = euler_xyz_from_quat(robot.data.root_quat_w)
        euler_t   = torch.stack(euler, dim=-1)                             # (N,3)

        # Target position (absolute world frame – useful for the policy to
        # understand where "home" is, especially if randomised per episode)
        target    = self.env._desired_pos_w                                # (N,3)

        # Previous actions (temporal context for smoother behaviour)
        prev_act  = self.env._previous_actions                             # (N,4)

        obs = torch.cat([pos_rel, vel_b, ang_vel_b, euler_t, target, prev_act], dim=-1)
        # Total: 3 + 3 + 3 + 3 + 3 + 4 = 19 dims

        return {"policy": obs}

    # ==================================================================
    # RESET
    # ==================================================================
    def reset_idx(self, env_ids: Optional[torch.Tensor]):
        env = self.env

        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = env._robot._ALL_INDICES

        n = len(env_ids)

        # ---------------------------------------------------------------
        # Logging
        # ---------------------------------------------------------------
        if self.cfg.is_train and hasattr(self, "_episode_sums"):
            extras = {}
            for key in self._episode_sums:
                extras["Episode_Reward/" + key] = (
                    torch.mean(self._episode_sums[key][env_ids]) / env.max_episode_length_s
                )
                self._episode_sums[key][env_ids] = 0.0
            env.extras.setdefault("log", {}).update(extras)
            env.extras["log"]["Episode_Termination/died"]     = torch.count_nonzero(env.reset_terminated[env_ids]).item()
            env.extras["log"]["Episode_Termination/time_out"] = torch.count_nonzero(env.reset_time_outs[env_ids]).item()

        # ---------------------------------------------------------------
        # Robot reset
        # ---------------------------------------------------------------
        env._robot.reset(env_ids)

        # ---------------------------------------------------------------
        # Domain randomisation (training only)
        # ---------------------------------------------------------------
        if self.cfg.is_train:
            self._randomize_dynamics(env_ids)

        # ---------------------------------------------------------------
        # Randomise episode_length_buf on full reset (curriculum trick)
        # ---------------------------------------------------------------
        if n == self.num_envs and self.num_envs > 1:
            env.episode_length_buf = torch.randint_like(
                env.episode_length_buf, high=int(env.max_episode_length)
            )

        # ---------------------------------------------------------------
        # Clear control state
        # ---------------------------------------------------------------
        env._actions[env_ids]              = 0.0
        env._previous_actions[env_ids]     = 0.0
        env._motor_speeds[env_ids]         = 0.0
        env._previous_omega_meas[env_ids]  = 0.0
        env._omega_err_integral[env_ids]   = 0.0
        env._crashed[env_ids]              = 0
        env._landed_success[env_ids]       = False

        # Joint state
        joint_pos = env._robot.data.default_joint_pos[env_ids]
        joint_vel = env._robot.data.default_joint_vel[env_ids]
        env._robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        # ---------------------------------------------------------------
        # Spawn position & orientation
        # ---------------------------------------------------------------
        default_root_state = env._robot.data.default_root_state[env_ids].clone()

        if self.task_mode == "hover":
            self._reset_hover(env_ids, n, default_root_state)
        else:   # "land"
            self._reset_land(env_ids, n, default_root_state)

        # ---------------------------------------------------------------
        # Write to sim
        # ---------------------------------------------------------------
        env._robot.write_root_link_pose_to_sim(default_root_state[:, :7], env_ids)
        env._robot.write_root_com_velocity_to_sim(default_root_state[:, 7:], env_ids)

    # ------------------------------------------------------------------
    # Hover reset
    # ------------------------------------------------------------------
    def _reset_hover(self, env_ids, n, root_state):
        env = self.env
        device = self.device

        # Target: fixed or randomised around a centre point.
        # Training: randomise XY ± 2 m, Z ∈ [0.8, 2.5] m
        # Play: use the fixed hover target.
        if self.cfg.is_train:
            target_x = torch.empty(n, device=device).uniform_(-2.0, 2.0)
            target_y = torch.empty(n, device=device).uniform_(-2.0, 2.0)
            target_z = torch.empty(n, device=device).uniform_(0.8, 2.5)
            target = torch.stack([target_x, target_y, target_z], dim=-1)   # (n,3)
        else:
            target = self._hover_target.unsqueeze(0).expand(n, -1).clone()

        env._desired_pos_w[env_ids] = target

        # Spawn above the target with some random offset so the drone has to
        # converge rather than starting exactly on it.
        spawn_offset_x = torch.empty(n, device=device).uniform_(-1.0, 1.0)
        spawn_offset_y = torch.empty(n, device=device).uniform_(-1.0, 1.0)
        spawn_offset_z = torch.empty(n, device=device).uniform_(0.3, 1.0)   # always above target

        root_state[:, 0] = target[:, 0] + spawn_offset_x
        root_state[:, 1] = target[:, 1] + spawn_offset_y
        root_state[:, 2] = target[:, 2] + spawn_offset_z

        # Orientation: small random roll/pitch, random yaw
        quat = quat_from_euler_xyz(
            torch.empty(n, device=device).uniform_(-0.15, 0.15),   # roll
            torch.empty(n, device=device).uniform_(-0.15, 0.15),   # pitch
            torch.empty(n, device=device).uniform_(-np.pi, np.pi), # yaw
        )
        root_state[:, 3:7] = quat

        # Zero initial velocity
        root_state[:, 7:] = 0.0

    # ------------------------------------------------------------------
    # Land reset
    # ------------------------------------------------------------------
    def _reset_land(self, env_ids, n, root_state):
        env = self.env
        device = self.device

        # Target is on the ground directly below spawn
        spawn_x = torch.empty(n, device=device).uniform_(-2.0, 2.0)
        spawn_y = torch.empty(n, device=device).uniform_(-2.0, 2.0)
        spawn_z = torch.empty(n, device=device).uniform_(1.5, 3.0)   # start high

        root_state[:, 0] = spawn_x
        root_state[:, 1] = spawn_y
        root_state[:, 2] = spawn_z

        # Ground target directly below
        env._desired_pos_w[env_ids, 0] = spawn_x
        env._desired_pos_w[env_ids, 1] = spawn_y
        env._desired_pos_w[env_ids, 2] = 0.0   # ground

        # Store spawn altitude for altitude-progress reward
        if not hasattr(self, "_spawn_altitude"):
            self._spawn_altitude = torch.ones(self.num_envs, device=device)
        self._spawn_altitude[env_ids] = spawn_z

        # Orientation: nearly level
        quat = quat_from_euler_xyz(
            torch.empty(n, device=device).uniform_(-0.1, 0.1),
            torch.empty(n, device=device).uniform_(-0.1, 0.1),
            torch.empty(n, device=device).uniform_(-np.pi, np.pi),
        )
        root_state[:, 3:7] = quat

        # Zero initial velocity
        root_state[:, 7:] = 0.0

    # ------------------------------------------------------------------
    # Domain randomisation  –  training only
    # ------------------------------------------------------------------
    def _randomize_dynamics(self, env_ids: torch.Tensor):
        """Perturb dynamics parameters within ± ranges around nominal values."""
        device = self.device
        n      = len(env_ids)
        env    = self.env

        # Helper: uniform sample in [nominal * lo_frac, nominal * hi_frac]
        def _rand_scale(nominal, lo, hi):
            r = torch.rand(n, device=device)
            return nominal * lo + r * nominal * (hi - lo)

        # --- Thrust-to-weight ±5 % ---
        env._thrust_to_weight[env_ids] = _rand_scale(self.cfg.thrust_to_weight, 0.95, 0.05)

        # --- Aerodynamics ×[0.5, 2.0] ---
        k_xy = _rand_scale(self.cfg.k_aero_xy, 0.5, 1.5)
        k_z  = _rand_scale(self.cfg.k_aero_z,  0.5, 1.5)
        env._K_aero[env_ids, :2] = k_xy.unsqueeze(1)
        env._K_aero[env_ids, 2]  = k_z

        # --- PID roll/pitch ±15 % (D ±30 %) ---
        env._kp_omega[env_ids, :2] = _rand_scale(self.cfg.kp_omega_rp, 0.85, 0.30).unsqueeze(1)
        env._ki_omega[env_ids, :2] = _rand_scale(self.cfg.ki_omega_rp, 0.85, 0.30).unsqueeze(1)
        env._kd_omega[env_ids, :2] = _rand_scale(self.cfg.kd_omega_rp, 0.70, 0.60).unsqueeze(1)

        # --- PID yaw ±15 % (D ±30 %) ---
        env._kp_omega[env_ids, 2] = _rand_scale(self.cfg.kp_omega_y,  0.85, 0.30)
        env._ki_omega[env_ids, 2] = _rand_scale(self.cfg.ki_omega_y,  0.85, 0.30)
        env._kd_omega[env_ids, 2] = _rand_scale(self.cfg.kd_omega_y,  0.70, 0.60)

        # --- Motor time constant ±30 % ---
        env._tau_m[env_ids] = _rand_scale(self.cfg.tau_m, 0.70, 0.60).unsqueeze(1).expand(-1, 4)