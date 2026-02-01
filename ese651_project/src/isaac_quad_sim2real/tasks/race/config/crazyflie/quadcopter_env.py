# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.envs.ui import BaseEnvWindow
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.visualization_markers import VisualizationMarkersCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import subtract_frame_transforms, quat_from_euler_xyz, euler_xyz_from_quat, wrap_to_pi, matrix_from_quat
from isaaclab.sensors import ContactSensor, ContactSensorCfg

from collections import deque
import math

from typing import List
from dataclasses import dataclass, field

# Import strategy class
from .quadcopter_strategies import Mark4HoverLandStrategy

##
# Drone config – Mark4 7" frame
##
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

# ---------------------------------------------------------------------------
# MARK4_CFG  –  replace usd_path with your Mark4 USD once it's ready.
# The USD must expose joints named m1_joint … m4_joint and a link named "body".
# Mass, inertia, and rotor positions are read automatically from the USD physics
# properties, so get those right in the USD first.
# ---------------------------------------------------------------------------
MARK4_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    collision_group=0,
    spawn=sim_utils.UsdFileCfg(
        # TODO: replace with your Mark4 USD path
        usd_path=f"usd/mark4.usda",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=10.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
        copy_from_source=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.0),          # start 1 m up
        joint_pos={
            ".*": 0.0,
        },
        joint_vel={
            # TODO: set idle RPM appropriate for Mark4 ESCs once known.
            # These seed the motor-speed filter so the drone doesn't drop on
            # the first frame.  A value near hover RPM is ideal.
            "m1_joint":  800.0,
            "m2_joint": -800.0,
            "m3_joint":  800.0,
            "m4_joint": -800.0,
        },
    ),
    actuators={
        "dummy": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=0.0,
            damping=0.0,
        ),
    },
)

D2R = np.pi / 180.0
R2D = 180.0 / np.pi


GOAL_MARKER_CFG = VisualizationMarkersCfg(
    markers={
        "sphere": sim_utils.SphereCfg(
            radius=0.08,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        ),
    }
)


class QuadcopterEnvWindow(BaseEnvWindow):
    """Window manager for the Quadcopter environment."""

    def __init__(self, env: QuadcopterEnv, window_name: str = "IsaacLab"):
        super().__init__(env, window_name)
        with self.ui_window_elements["main_vstack"]:
            with self.ui_window_elements["debug_frame"]:
                with self.ui_window_elements["debug_vstack"]:
                    self._create_debug_vis_ui_element("targets", self.env)


@configclass
class QuadcopterEnvCfg(DirectRLEnvCfg):
    # ---------------------------------------------------------------
    # Task mode  –  "hover" or "land".  Passed through to the strategy.
    # ---------------------------------------------------------------
    task_mode: str = "hover"   # "hover" | "land"

    # env
    episode_length_s = 30.0
    action_space = 4
    observation_space = 1   # placeholder; actual obs dim is assembled in strategy
    state_space = 0
    debug_vis = True

    sim_rate_hz = 500
    policy_rate_hz = 50
    pid_loop_rate_hz = 500
    decimation = sim_rate_hz // policy_rate_hz
    pid_loop_decimation = sim_rate_hz // pid_loop_rate_hz

    ui_window_class_type = QuadcopterEnvWindow

    # simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1 / sim_rate_hz,
        render_interval=decimation,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=0.0, replicate_physics=True)

    # robot
    robot: ArticulationCfg = MARK4_CFG.replace(
        prim_path="/World/envs/env_.*/Robot",
    )
    # Contact sensor – only ground collisions matter now
    contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/body",
        update_period=0.0,
        history_length=6,
        debug_vis=False,
        force_threshold=0.0,
    )

    beta = 1.0   # action smoothing: 1.0 = no smoothing

    # ---------------------------------------------------------------
    # Reset / termination bounds
    # ---------------------------------------------------------------
    min_altitude = 0.05          # ground-contact threshold (m)
    max_altitude = 5.0           # ceiling (m)
    max_time_on_ground = 1.5     # s before "stuck on ground" termination

    # ---------------------------------------------------------------
    # Motor / propulsion  –  MARK4 PLACEHOLDERS
    # Fill in from bench-test data before training.
    # ---------------------------------------------------------------
    # Arm length: center-to-motor distance.
    # Mark4 diagonal ~295-310 mm  →  arm ≈ 150 mm.
    arm_length: float = 0.150                # TODO: measure your frame

    # Thrust coefficient  k_eta:  F = k_eta * omega^2  (per motor)
    # Bench-test range for 1350KV + 7" props: 1.5e-5 … 3e-5
    k_eta: float = 2.0e-5                    # TODO: bench test

    # Torque coefficient  k_m:   M = k_m  * omega^2  (per motor)
    # Typical ratio k_m/k_eta ≈ 0.01 … 0.05 for this class of prop
    k_m: float = 6.0e-7                      # TODO: bench test

    # Motor time constant (s).  Larger motors → slower response.
    tau_m: float = 0.015                     # TODO: bench test (expect 0.01-0.02)

    # ESC throttle → RPM mapping endpoints.
    # TODO: characterize your ESC+motor combo.  These are in the same units
    # that k_eta was measured with (usually raw RPM or a normalized speed).
    motor_speed_min: float = 0.0
    motor_speed_max: float = 3500.0          # TODO: bench test

    # Thrust-to-weight ratio.  Typical 7" racer: 4-6.
    # Computed as (max_total_thrust) / (mass * g).  Measure on bench.
    thrust_to_weight: float = 5.0            # TODO: bench test

    # ---------------------------------------------------------------
    # PID  –  start conservative, tune up
    # ---------------------------------------------------------------
    # Roll / Pitch
    kp_omega_rp: float = 80.0
    ki_omega_rp: float = 40.0
    kd_omega_rp: float = 3.0
    i_limit_rp:  float = 20.0

    # Yaw
    kp_omega_y:  float = 50.0
    ki_omega_y:  float = 10.0
    kd_omega_y:  float = 1.5
    i_limit_y:   float = 100.0

    # ---------------------------------------------------------------
    # Action scaling  –  conservative starting points for a 7" quad
    # ---------------------------------------------------------------
    body_rate_scale_xy: float = 150.0 * D2R   # ~2.6 rad/s max roll/pitch rate
    body_rate_scale_z:  float = 200.0 * D2R   # ~3.5 rad/s max yaw rate

    # ---------------------------------------------------------------
    # Aerodynamic drag (tune later)
    # ---------------------------------------------------------------
    k_aero_xy: float = 3.0e-6
    k_aero_z:  float = 4.0e-6

    # ---------------------------------------------------------------
    # Termination
    # ---------------------------------------------------------------
    max_tilt_thresh: float = 150.0 * D2R   # crash if tilted past this

    # ---------------------------------------------------------------
    # Reward dict (populated by train script or play script)
    # ---------------------------------------------------------------
    rewards: dict = {}

    # Parameters from train.py or play.py
    is_train = None

    # Strategy class
    strategy_class: type = Mark4HoverLandStrategy


class QuadcopterEnv(DirectRLEnv):
    cfg: QuadcopterEnvCfg

    def __init__(self, cfg: QuadcopterEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self.iteration = 0

        if len(cfg.rewards) > 0:
            self.rew = cfg.rewards
        elif self.cfg.is_train:
            raise ValueError("rewards not provided")

        # ---------------------------------------------------------------
        # Action / control state tensors
        # ---------------------------------------------------------------
        self._actions           = torch.zeros(self.num_envs, self.cfg.action_space, device=self.device)
        self._previous_actions  = torch.zeros(self.num_envs, self.cfg.action_space, device=self.device)

        self._thrust            = torch.zeros(self.num_envs, 1, 3, device=self.device)
        self._moment            = torch.zeros(self.num_envs, 1, 3, device=self.device)
        self._wrench_des        = torch.zeros(self.num_envs, 4, device=self.device)
        self._motor_speeds      = torch.zeros(self.num_envs, 4, device=self.device)
        self._motor_speeds_des  = torch.zeros(self.num_envs, 4, device=self.device)

        # PID state
        self._previous_omega_meas   = torch.zeros(self.num_envs, 3, device=self.device)
        self._omega_err_integral    = torch.zeros(self.num_envs, 3, device=self.device)

        # ---------------------------------------------------------------
        # Target / task state
        # ---------------------------------------------------------------
        self._desired_pos_w = torch.zeros(self.num_envs, 3, device=self.device)
        self._crashed       = torch.zeros(self.num_envs, device=self.device, dtype=torch.int)

        # ---------------------------------------------------------------
        # Motor allocation matrices
        # ---------------------------------------------------------------
        r = self.cfg.arm_length * np.sqrt(2.0) / 2.0   # perpendicular distance to each rotor
        self._rotor_positions = torch.tensor(
            [
                [ r,  r, 0],
                [ r, -r, 0],
                [-r, -r, 0],
                [-r,  r, 0],
            ],
            dtype=torch.float32,
            device=self.device,
        )
        self._rotor_directions = torch.tensor([1, -1, 1, -1], device=self.device)
        self.k = self.cfg.k_m / self.cfg.k_eta   # torque/thrust ratio

        # f_to_TM : [4×4]  maps per-motor forces → [Fz, Mx, My, Mz]
        self.f_to_TM = torch.cat(
            [
                torch.tensor([[1, 1, 1, 1]], device=self.device),
                torch.cat(
                    [
                        torch.linalg.cross(
                            self._rotor_positions[i],
                            torch.tensor([0.0, 0.0, 1.0], device=self.device),
                        ).view(-1, 1)[0:2]
                        for i in range(4)
                    ],
                    dim=1,
                ).to(self.device),
                self.k * self._rotor_directions.view(1, -1),
            ],
            dim=0,
        )
        self.TM_to_f = torch.linalg.inv(self.f_to_TM)

        # ---------------------------------------------------------------
        # Robot physical properties (read from USD via physics view)
        # ---------------------------------------------------------------
        self._body_id       = self._robot.find_bodies("body")[0]
        self._robot_mass    = self._robot.root_physx_view.get_masses()[0].sum()
        self._gravity_magnitude = torch.tensor(self.sim.cfg.gravity, device=self.device).norm()
        self._robot_weight  = (self._robot_mass * self._gravity_magnitude).item()

        self.inertia_tensor = (
            self._robot.root_physx_view.get_inertias()[0, self._body_id, :]
            .view(-1, 3, 3)
            .tile(self.num_envs, 1, 1)
            .to(self.device)
        )

        # ---------------------------------------------------------------
        # Per-env parameter tensors (support domain randomisation)
        # ---------------------------------------------------------------
        self._K_aero           = torch.zeros(self.num_envs, 3, device=self.device)
        self._kp_omega         = torch.zeros(self.num_envs, 3, device=self.device)
        self._ki_omega         = torch.zeros(self.num_envs, 3, device=self.device)
        self._kd_omega         = torch.zeros(self.num_envs, 3, device=self.device)
        self._tau_m            = torch.zeros(self.num_envs, 4, device=self.device)
        self._thrust_to_weight = torch.zeros(self.num_envs, device=self.device)

        # Nominal values (strategy __init__ may overwrite via domain rand)
        self._twr_value         = self.cfg.thrust_to_weight
        self._k_aero_xy_value   = self.cfg.k_aero_xy
        self._k_aero_z_value    = self.cfg.k_aero_z
        self._kp_omega_rp_value = self.cfg.kp_omega_rp
        self._ki_omega_rp_value = self.cfg.ki_omega_rp
        self._kd_omega_rp_value = self.cfg.kd_omega_rp
        self._kp_omega_y_value  = self.cfg.kp_omega_y
        self._ki_omega_y_value  = self.cfg.ki_omega_y
        self._kd_omega_y_value  = self.cfg.kd_omega_y
        self._tau_m_value       = self.cfg.tau_m

        # ---------------------------------------------------------------
        # Strategy (rewards, obs, resets)
        # ---------------------------------------------------------------
        self.strategy = self.cfg.strategy_class(self)

        self.set_debug_vis(self.cfg.debug_vis)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def update_iteration(self, iter):
        self.iteration = iter

    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            if not hasattr(self, "goal_pos_visualizer"):
                marker_cfg = GOAL_MARKER_CFG.copy()
                marker_cfg.prim_path = "/Visuals/Command/goal_position"
                self.goal_pos_visualizer = VisualizationMarkers(marker_cfg)
            self.goal_pos_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_pos_visualizer"):
                self.goal_pos_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        self.goal_pos_visualizer.visualize(self._desired_pos_w)

    # ------------------------------------------------------------------
    # Scene setup  –  no gates, just ground + robot + contact sensor
    # ------------------------------------------------------------------
    def _setup_scene(self):
        self._robot = Articulation(self.cfg.robot)
        self.scene.articulations["robot"] = self._robot

        self._contact_sensor = ContactSensor(self.cfg.contact_sensor)
        self.scene.sensors["contact_sensor"] = self._contact_sensor

        self.cfg.terrain.num_envs    = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)

        self.scene.clone_environments(copy_from_source=False)

        # Dome light
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    # ------------------------------------------------------------------
    # Motor allocation  –  wrench → per-motor speeds
    # ------------------------------------------------------------------
    def _compute_motor_speeds(self, wrench_des):
        f_des = torch.matmul(wrench_des, self.TM_to_f.t())
        motor_speed_squared = f_des / self.cfg.k_eta
        motor_speeds_des = torch.sign(motor_speed_squared) * torch.sqrt(torch.abs(motor_speed_squared))
        motor_speeds_des = motor_speeds_des.clamp(self.cfg.motor_speed_min, self.cfg.motor_speed_max)
        return motor_speeds_des

    # ------------------------------------------------------------------
    # PID: body-rate error → moment command
    # ------------------------------------------------------------------
    def _get_moment_from_ctbr(self, actions):
        omega_des = torch.zeros(self.num_envs, 3, device=self.device)
        omega_des[:, 0] = self.cfg.body_rate_scale_xy * actions[:, 1]   # roll rate
        omega_des[:, 1] = self.cfg.body_rate_scale_xy * actions[:, 2]   # pitch rate
        omega_des[:, 2] = self.cfg.body_rate_scale_z  * actions[:, 3]   # yaw rate

        omega_meas = self._robot.data.root_ang_vel_b
        omega_err  = omega_des - omega_meas

        # Integrator with clamping
        self._omega_err_integral += omega_err / self.cfg.pid_loop_rate_hz
        limits = torch.tensor(
            [self.cfg.i_limit_rp, self.cfg.i_limit_rp, self.cfg.i_limit_y],
            device=self._omega_err_integral.device,
        )
        self._omega_err_integral = torch.clamp(self._omega_err_integral, min=-limits, max=limits)

        # Derivative (filtered via finite difference)
        self._previous_omega_meas = torch.where(
            torch.abs(self._previous_omega_meas) < 1e-4,
            omega_meas,
            self._previous_omega_meas,
        )
        omega_meas_dot = (omega_meas - self._previous_omega_meas) * self.cfg.pid_loop_rate_hz
        self._previous_omega_meas = omega_meas.clone()

        omega_dot = (
            self._kp_omega * omega_err
            + self._ki_omega * self._omega_err_integral
            - self._kd_omega * omega_meas_dot
        )

        cmd_moment = torch.bmm(self.inertia_tensor, omega_dot.unsqueeze(2)).squeeze(2)
        return cmd_moment

    # ==================================================================
    # DirectRLEnv interface  –  called every step in this order
    # ==================================================================

    def _pre_physics_step(self, actions: torch.Tensor):
        self._actions = actions.clone().clamp(-1.0, 1.0)
        # Action smoothing
        self._actions = self.cfg.beta * self._actions + (1 - self.cfg.beta) * self._previous_actions
        self._previous_actions = self._actions.clone()

        # Thrust command: map [-1,1] → [0, TWR * weight]
        self._wrench_des[:, 0] = (
            (self._actions[:, 0] + 1.0) / 2.0
        ) * self._robot_weight * self._thrust_to_weight

        self.pid_loop_counter = 0

    def _apply_action(self):
        if self.pid_loop_counter % self.cfg.pid_loop_decimation == 0:
            self._wrench_des[:, 1:] = self._get_moment_from_ctbr(self._actions)
            self._motor_speeds_des  = self._compute_motor_speeds(self._wrench_des)
        self.pid_loop_counter += 1

        # First-order motor lag
        motor_accel = (self._motor_speeds_des - self._motor_speeds) / self._tau_m
        self._motor_speeds += motor_accel * self.physics_dt
        self._motor_speeds = self._motor_speeds.clamp(self.cfg.motor_speed_min, self.cfg.motor_speed_max)

        # Actual forces from current motor speeds
        motor_forces = self.cfg.k_eta * self._motor_speeds ** 2
        wrench = torch.matmul(motor_forces, self.f_to_TM.t())

        # Aerodynamic drag (linear velocity dependent)
        lin_vel_b = self._robot.data.root_com_lin_vel_b
        theta_dot = torch.sum(self._motor_speeds, dim=1, keepdim=True)
        drag = -theta_dot * self._K_aero.unsqueeze(0) * lin_vel_b

        self._thrust[:, 0, :]  = drag
        self._thrust[:, 0, 2] += wrench[:, 0]        # total thrust along z
        self._moment[:, 0, :]  = wrench[:, 1:]       # moments

        self._robot.set_external_force_and_torque(self._thrust, self._moment, body_ids=self._body_id)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        episode_time = self.episode_length_buf * self.cfg.sim.dt * self.cfg.decimation
        alt = self._robot.data.root_link_pos_w[:, 2]

        # Stuck on ground too long
        cond_ground = torch.logical_and(alt < self.cfg.min_altitude, episode_time > self.cfg.max_time_on_ground)
        # Hit ceiling
        cond_ceiling = alt > self.cfg.max_altitude
        # Sustained contact (crash accumulator set in strategy)
        cond_crashed = self._crashed > 100
        # Flipped past tilt limit
        euler = euler_xyz_from_quat(self._robot.data.root_quat_w)
        tilt = torch.abs(euler[0]) + torch.abs(euler[1])
        cond_flipped = tilt > self.cfg.max_tilt_thresh

        died = cond_ground | cond_ceiling | cond_crashed | cond_flipped

        time_out = self.episode_length_buf >= self.max_episode_length - 1

        # Landing success  –  strategy can set this flag
        if hasattr(self, '_landed_success'):
            time_out = time_out | self._landed_success

        return died, time_out

    def _get_rewards(self) -> torch.Tensor:
        return self.strategy.get_rewards()

    def _reset_idx(self, env_ids: torch.Tensor | None):
        self.strategy.reset_idx(env_ids)
        super()._reset_idx(env_ids)

    def _get_observations(self) -> dict:
        return self.strategy.get_observations()