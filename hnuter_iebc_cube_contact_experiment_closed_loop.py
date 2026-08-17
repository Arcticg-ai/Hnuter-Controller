#!/usr/bin/env python3
"""Automated Gazebo-only closed-loop IEBC cube-contact experiment for HNUTER.

This entry point deliberately stays separate from the real-aircraft IEBC
controller.  It reuses :class:`InteractionEnergyBarrierFilter` and the PX4
position-setpoint conversion from
``hnuter_external_controller_px4_position_hardware_iebc_closed_loop.py``, but adds a
simulation-only state machine, Gazebo contact/wrench transport, and PX4
VehicleCommand messages so the complete experiment can be reproduced without
a transmitter.

Experiment sequence (Gazebo world X == controller ENU X):

1. Arm, enter Offboard and rise to the cube centre height.
2. Hold position and yaw until the probe's physical +X axis faces the cube.
3. Apply a persistent constant -X virtual force, pinning the light door proxy
   against the rail's lower stop before the probe can move it.
4. Approach with the lightweight cylindrical probe until contact is measured.
5. Increase the forward position reference slowly so contact force rises.
6. Release the virtual resistance either at an externally scheduled PUSH time
   (the barrier experiment default) or by the legacy sustained-force threshold.
7. Keep IEBC active after release while freezing only the environment-storage
   estimate, so controller storage to kinetic-energy conversion remains visible.
8. Record nominal/safe references, reference powers, contact diagnostics and the
   complete IEBC energy state to CSV.

This file must never be used on real hardware.  It is guarded by an explicit
environment variable and by the expected Gazebo world name.
"""

import csv
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

# Conservative, explicit defaults for this one-dimensional contact experiment.
# They must be set before importing the base IEBC module because its constructor
# reads the environment.
os.environ.setdefault('HNUTER_IEBC_ENABLE', '1')
os.environ.setdefault('HNUTER_IEBC_MASS_KG', '4.5')
os.environ.setdefault('HNUTER_IEBC_LAMBDA_BAR_KG', '4.5')
# The legacy experiment used 1.2 J for K_I + Sbar only. The revised
# certificate also includes V_c; replaying the two geometry-valid legacy 7 N
# successes gives about 1.66/1.87 J under K_I + V_c + Sbar. Keep this
# Gazebo-only default above those traces with explicit margin. Real hardware
# must select E_max from its own certified interaction-energy limit.
os.environ.setdefault('HNUTER_IEBC_E_MAX_J', '2.5')
os.environ.setdefault('HNUTER_IEBC_AXIS_X', '1.0')
os.environ.setdefault('HNUTER_IEBC_AXIS_Y', '0.0')
os.environ.setdefault('HNUTER_IEBC_AXIS_Z', '0.0')
os.environ.setdefault('HNUTER_IEBC_CBF_GAMMA', '4.0')
os.environ.setdefault('HNUTER_IEBC_KC_NPM', '11.25')
os.environ.setdefault('HNUTER_IEBC_DC_NSPM', '11.25')
os.environ.setdefault('HNUTER_IEBC_REF_SYNC_GAIN', '0.0')
os.environ.setdefault('HNUTER_IEBC_MAX_REF_SPEED_MPS', '0.12')
os.environ.setdefault('HNUTER_IEBC_MAX_REF_ACCEL_MPS2', '3.0')
os.environ.setdefault('HNUTER_IEBC_POWER_MARGIN_W', '0.05')
os.environ.setdefault('HNUTER_IEBC_FORCE_ERROR_BOUND_N', '0.20')
os.environ.setdefault('HNUTER_IEBC_RESIDUAL_POWER_BOUND_W', '0.0')
os.environ.setdefault('HNUTER_IEBC_STORAGE_INITIAL_J', '0.0')
os.environ.setdefault('HNUTER_IEBC_ACCEL_FF_MODE', 'nominal')
os.environ.setdefault('HNUTER_IEBC_WRENCH_SOURCE', 'proxy')

from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleCommand

from hnuter_external_controller_px4_position_hardware_iebc_closed_loop import HnuterController
from hnuter_log_paths import diagnostic_csv_path


def smoothstep01(value: float) -> tuple:
    """Return cubic smooth-step position, first and second derivatives."""
    u = float(np.clip(value, 0.0, 1.0))
    return (3.0 * u ** 2 - 2.0 * u ** 3,
            6.0 * u * (1.0 - u),
            6.0 * (1.0 - 2.0 * u))


def wrap_pi(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return float(math.atan2(math.sin(angle_rad), math.cos(angle_rad)))


class ContactForceFilter:
    """Age-gated first-order filter for Gazebo's contact-wrench samples."""

    def __init__(self, tau_s: float = 0.08, timeout_s: float = 0.15):
        self.tau_s = max(float(tau_s), 0.0)
        self.timeout_s = max(float(timeout_s), 0.01)
        self.raw_n = 0.0
        self.filtered_n = 0.0
        self.last_sample_monotonic = -math.inf

    def feed(self, force_n: float, received_s: float = None) -> None:
        self.raw_n = max(float(force_n), 0.0)
        self.last_sample_monotonic = (
            time.monotonic() if received_s is None else float(received_s))

    def update(self, dt: float, now_s: float = None) -> float:
        now_s = time.monotonic() if now_s is None else float(now_s)
        target = self.raw_n if now_s - self.last_sample_monotonic <= self.timeout_s else 0.0
        alpha = 1.0 if self.tau_s <= 1e-6 else float(np.clip(dt / (self.tau_s + dt), 0.0, 1.0))
        self.filtered_n += alpha * (target - self.filtered_n)
        return self.filtered_n


class SustainedForceThreshold:
    """Debounce a filtered force threshold without using object motion."""

    def __init__(self, threshold_n: float, hold_s: float):
        self.threshold_n = max(float(threshold_n), 0.0)
        self.hold_s = max(float(hold_s), 0.0)
        self.since_s = None

    def reset(self) -> None:
        self.since_s = None

    def update(self, force_n: float, now_s: float) -> bool:
        force_n = float(force_n)
        now_s = float(now_s)
        if not math.isfinite(force_n) or force_n < self.threshold_n:
            self.since_s = None
            return False
        if self.since_s is None:
            self.since_s = now_s
        return now_s - self.since_s >= self.hold_s


class HnuterIebcClosedLoopCubeContactExperiment(HnuterController):
    """SITL-only controller and Gazebo experiment coordinator."""

    EXPECTED_WORLD = 'hnuter_cube_contact'
    CUBE_MODEL = 'interaction_cube'
    VEHICLE_MODEL_PREFIX = 'hnuter_contact_'
    CONTACT_TOPIC = '/hnuter/cube_contact'
    FORCE_MARKER_NAMESPACE = 'hnuter_virtual_resistance'

    STAGE_WAIT = 'WAIT_CONTROL'
    STAGE_TAKEOFF = 'TAKEOFF'
    STAGE_ALIGN = 'ALIGN_YAW'
    STAGE_APPROACH = 'APPROACH'
    STAGE_LOAD_SETTLE = 'LOAD_SETTLE'
    STAGE_PUSH = 'PUSH_RAMP'
    STAGE_RELEASE = 'RELEASE_OBSERVE'
    STAGE_COMPLETE = 'COMPLETE'
    STAGE_FAILED = 'FAILED'

    def __init__(self):
        if os.environ.get('HNUTER_IEBC_CUBE_SIM', '0') != '1':
            raise RuntimeError(
                'Refusing to start: set HNUTER_IEBC_CUBE_SIM=1 only for the '
                'HNUTER Gazebo cube-contact experiment.')

        self.world_name = os.environ.get('HNUTER_GZ_WORLD', self.EXPECTED_WORLD)
        if self.world_name != self.EXPECTED_WORLD:
            raise RuntimeError(
                f'Expected Gazebo world {self.EXPECTED_WORLD!r}, got {self.world_name!r}.')

        super().__init__()

        # Free-flight actuator power is not environment interaction. Keep the
        # reference filter transparent through takeoff and approach, then arm
        # and reset IEBC exactly at measured probe contact.
        self._iebc_requested = bool(self.iebc.enabled)
        self.iebc.enabled = False
        self.iebc.reset()

        qos_command = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_command)

        self.cmd_set_mode = getattr(VehicleCommand, 'VEHICLE_CMD_DO_SET_MODE', 176)
        self.cmd_arm_disarm = getattr(VehicleCommand, 'VEHICLE_CMD_COMPONENT_ARM_DISARM', 400)
        self._startup_ticks = 0
        self._last_mode_request_s = -math.inf
        self._last_arm_request_s = -math.inf

        self.virtual_force_n = abs(float(os.environ.get('HNUTER_CUBE_FORCE_N', '2.0')))
        self.release_mode = os.environ.get(
            'HNUTER_CUBE_RELEASE_MODE', 'force').strip().lower()
        if self.release_mode not in ('force', 'time'):
            raise ValueError(
                'HNUTER_CUBE_RELEASE_MODE must be "force" or "time", got '
                f'{self.release_mode!r}.')
        self.release_time_s = max(
            float(os.environ.get('HNUTER_CUBE_RELEASE_TIME_S', '85.0')), 0.1)
        # This is a force-triggered breakaway latch, not a displacement test.
        # The low-pass filter rejects one-step collision impulses; a short
        # continuous hold prevents a single filtered sample from releasing the
        # load. The default margin is zero so the virtual resistance disappears
        # as soon as measured force genuinely reaches it.
        self.release_force_margin_n = max(
            float(os.environ.get('HNUTER_CUBE_RELEASE_FORCE_MARGIN_N', '0.0')), 0.0)
        self.release_force_hold_s = max(
            float(os.environ.get('HNUTER_CUBE_RELEASE_FORCE_HOLD_S', '0.04')), 0.0)
        self.release_force_threshold_n = self.virtual_force_n + self.release_force_margin_n
        self.force_release_latch = SustainedForceThreshold(
            self.release_force_threshold_n, self.release_force_hold_s)
        self.barrier_tolerance_j = max(float(os.environ.get('HNUTER_CUBE_BARRIER_TOL_J', '0.02')), 0.0)
        self.qp_slack_tolerance_w = max(
            float(os.environ.get('HNUTER_CUBE_QP_SLACK_TOL_W', '0.02')), 0.0)
        self.qp_infeasible_hold_s = max(
            float(os.environ.get('HNUTER_CUBE_QP_INFEASIBLE_HOLD_S', '0.10')), 0.0)
        self.stop_barrier_tolerance_m = max(float(os.environ.get(
            'HNUTER_CUBE_STOP_BARRIER_TOL_M', '0.02')), 0.0)
        self.require_barrier_active = os.environ.get(
            'HNUTER_CUBE_REQUIRE_BARRIER_ACTIVE', '0').strip().lower() in (
                '1', 'true', 'yes', 'on')
        self.intervention_velocity_tolerance_mps = max(float(os.environ.get(
            'HNUTER_CUBE_INTERVENTION_VEL_TOL_MPS', '0.001')), 0.0)
        self.intervention_hold_s = max(float(os.environ.get(
            'HNUTER_CUBE_INTERVENTION_HOLD_S', '1.0')), 0.0)
        self.takeoff_height_m = max(float(os.environ.get('HNUTER_CUBE_TAKEOFF_M', '1.10')), 0.3)
        self.takeoff_time_s = max(float(os.environ.get('HNUTER_CUBE_TAKEOFF_TIME_S', '5.0')), 1.0)
        # The shortened probe tip is 0.75 m ahead of the vehicle origin.  The
        # cube's near face is at world X=2.6 m, so retain enough travel to make
        # contact without restoring the artificial one-metre lever arm.
        self.approach_distance_m = max(float(os.environ.get('HNUTER_CUBE_APPROACH_M', '2.10')), 0.5)
        self.approach_speed_mps = max(float(os.environ.get('HNUTER_CUBE_APPROACH_MPS', '0.12')), 0.02)
        # The long nose probe turns a 35 mm/s push into a dynamic contact test:
        # 10 N repeatedly lost yaw before force/allocation saturation, while the
        # same load completed at 20 mm/s.  Use the validated quasi-static rate;
        # force-sweep overrides remain explicit through the environment.
        self.push_speed_mps = max(float(os.environ.get('HNUTER_CUBE_PUSH_MPS', '0.050')), 0.005)
        self.max_push_distance_m = max(float(os.environ.get('HNUTER_CUBE_MAX_PUSH_M', '4.50')), 0.1)
        self.load_settle_s = max(float(os.environ.get('HNUTER_CUBE_LOAD_SETTLE_S', '1.5')), 0.2)
        self.release_observe_s = max(float(os.environ.get('HNUTER_CUBE_OBSERVE_S', '7.0')), 1.0)
        self.release_settle_position_tol_m = max(float(os.environ.get(
            'HNUTER_CUBE_SETTLE_POS_TOL_M', '0.05')), 0.01)
        self.release_settle_hold_s = max(float(os.environ.get(
            'HNUTER_CUBE_SETTLE_HOLD_S', '1.0')), 0.1)
        self.max_push_time_s = max(float(os.environ.get('HNUTER_CUBE_MAX_PUSH_TIME_S', '110.0')), 2.0)
        self.yaw_tolerance_rad = math.radians(max(
            float(os.environ.get('HNUTER_CUBE_YAW_TOL_DEG', '3.0')), 0.5))
        self.yaw_hold_s = max(float(os.environ.get('HNUTER_CUBE_YAW_HOLD_S', '1.0')), 0.2)
        self.yaw_timeout_s = max(float(os.environ.get('HNUTER_CUBE_YAW_TIMEOUT_S', '12.0')), 2.0)
        self.yaw_loss_tolerance_rad = math.radians(max(
            float(os.environ.get('HNUTER_CUBE_YAW_LOSS_TOL_DEG', '5.0')), 1.0))
        self.yaw_loss_hold_s = max(
            float(os.environ.get('HNUTER_CUBE_YAW_LOSS_HOLD_S', '0.25')), 0.05)
        # This is an outer world-heading loop around PX4's geometric yaw loop.
        # Gain 2.0 excited the KR_Y=20 inner-loop mode at about 1.3 Hz even
        # before contact.  Keep the outer correction deliberately slower.
        self.yaw_align_gain = max(
            float(os.environ.get('HNUTER_CUBE_YAW_ALIGN_GAIN', '0.6')), 0.0)
        self.yaw_align_max_rate_rad_s = math.radians(max(
            float(os.environ.get('HNUTER_CUBE_YAW_ALIGN_RATE_DPS', '15.0')), 1.0))
        self.yaw_command_bias_rad = math.radians(
            float(os.environ.get('HNUTER_CUBE_YAW_CMD_BIAS_DEG', '0.0')))

        # Gazebo and the base controller's ENU representation share world X.
        # This was confirmed from /world/.../pose/info against PX4 odometry;
        # using ENU Y makes the vehicle pass beside the cube.
        self.interaction_axis_enu = np.array([1.0, 0.0, 0.0], dtype=float)
        # The probe is fixed to body +X and the cube rail lies on Gazebo world
        # +X. This HNUTER SITL model's physical Gazebo yaw was calibrated
        # against the position-controller input: its world yaw follows the
        # controller's ENU yaw value directly (the PX4 bridge handles the
        # internal NED conversion). Keep this model-specific mapping here,
        # isolated from the real-aircraft controller.
        self.desired_world_yaw = 0.0
        # The PX4 local-yaw zero is initialized independently on every SITL
        # run.  This value is therefore only an optional initial trim;
        # Gazebo's world-heading outer loop continuously maps it to the physical
        # probe heading while valid contact geometry is required.
        self.desired_controller_yaw = self.yaw_command_bias_rad
        self.stage = self.STAGE_WAIT
        self.stage_start_s = 0.0
        self.experiment_origin_enu = None
        self.contact_origin_enu = None
        self.nominal_reference_enu = None
        self.release_target_enu = None
        self.terminal_hold_enu = None
        self.release_vehicle_position_enu = None
        self.release_vehicle_velocity_enu = None
        self.release_cube_x = math.nan
        self.release_contact_force_n = math.nan
        self.release_contact_force_raw_n = math.nan
        self.loaded_cube_x = math.nan
        self.yaw_aligned_since_s = None
        self.yaw_loss_since_s = None
        self.virtual_force_active = False
        self.release_event_seen = False
        self.peak_post_release_speed_mps = 0.0
        self.peak_post_release_position_delta_m = 0.0
        self.min_interaction_barrier_j = math.inf
        self.max_qp_slack_w = 0.0
        self.qp_infeasible_since_s = None
        self.barrier_active_seen = False
        self.reference_intervention_since_s = None
        self.max_reference_intervention_duration_s = 0.0
        self.max_reference_velocity_reduction_mps = 0.0
        self.min_stop_distance_barrier_m = math.inf
        self.max_release_excursion_m = 0.0
        self.recovery_complete_seen = False
        self.recovery_complete_time_s = math.nan
        self.release_settle_since_s = None
        self.release_settled = False
        self.release_settle_time_s = math.nan
        self.release_settle_anchor_enu = None
        self.release_position_change_m = math.inf
        self.max_recovery_dissipation_slack_w = 0.0
        self._latest_contact_sample = (0.0, 0.0, math.nan)

        self._transport_lock = threading.Lock()
        self.contact_filter = ContactForceFilter()
        self.cube_x_m = math.nan
        self.cube_y_m = math.nan
        self.vehicle_gz_position = np.full(3, math.nan)
        self.vehicle_gz_yaw = math.nan
        self.gz_node = GazeboNode()
        self.gz_node.subscribe(Contacts, self.CONTACT_TOPIC, self._contact_callback)
        self.gz_node.subscribe(
            Pose_V, f'/world/{self.world_name}/pose/info', self._pose_callback)
        self._persistent_wrench_pub = self.gz_node.advertise(
            f'/world/{self.world_name}/wrench/persistent', EntityWrench)
        self._clear_wrench_pub = self.gz_node.advertise(
            f'/world/{self.world_name}/wrench/clear', Entity)

        self.csv_path = Path(diagnostic_csv_path('hnuter_iebc_cube_contact_closed_loop'))
        self._csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow([
            'px4_time_s', 'stage', 'vehicle_enu_x_m', 'vehicle_enu_y_m',
            'vehicle_enu_z_m', 'vehicle_enu_vx_mps', 'vehicle_enu_vy_mps',
            'vehicle_enu_vz_mps', 'target_enu_x_m', 'target_enu_y_m',
            'target_z_relative_m', 'vehicle_gz_yaw_deg', 'target_gz_yaw_deg',
            'controller_yaw_cmd_deg', 'yaw_error_deg', 'cube_world_x_m', 'contact_force_raw_n',
            'contact_force_filtered_n', 'release_force_threshold_n',
            'force_threshold_hold_s', 'release_mode', 'scheduled_release_time_s',
            'cube_breakaway_m', 'virtual_force_active',
            'virtual_force_n', 'release_event_seen', 'iebc_active', 'iebc_barrier_active',
            'iebc_infeasible', 'iebc_storage_update_enabled',
            'barrier_active_seen', 'reference_intervention_duration_s',
            'max_reference_velocity_reduction_mps',
            'iebc_power_w', 'iebc_power_error_bound_w',
            'iebc_storage_rate_w', 'iebc_storage_j', 'iebc_kinetic_j',
            'iebc_controller_storage_j', 'iebc_energy_j', 'iebc_barrier_j',
            'iebc_constraint_barrier_j', 'iebc_energy_reserve_j',
            'min_interaction_barrier_j', 'iebc_reference_error_m',
            'iebc_nominal_reference_position_m', 'iebc_safe_reference_position_m',
            'iebc_velocity_mps', 'iebc_nominal_reference_velocity_mps',
            'iebc_task_reference_velocity_mps', 'iebc_safe_reference_velocity_mps',
            'iebc_energy_gradient_n', 'iebc_uncontrolled_power_w',
            'iebc_allowed_power_w', 'iebc_reference_power_nominal_w',
            'iebc_reference_power_safe_w', 'equivalent_stiffness_force_n',
            'contact_power_gt_w', 'iebc_qp_slack_w', 'max_iebc_qp_slack_w',
            'iebc_accel_safe_mps2',
            'iebc_mode', 'iebc_recoverable_energy_j',
            'iebc_release_excursion_m', 'iebc_stop_distance_barrier_m',
            'iebc_reserved_stop_distance_m', 'iebc_rho',
            'iebc_release_position_m', 'recovery_complete_seen',
            'recovery_complete_time_s', 'iebc_recovery_dissipation_slack_w',
            'release_position_change_m', 'release_settled', 'release_settle_time_s',
            'max_recovery_dissipation_slack_w', 'iebc_recovery_phase',
            'iebc_recovery_reference_velocity_mps',
            'iebc_recovery_rate_infeasible', 'iebc_recovery_terminal_position_m',
            'iebc_recovery_stop_candidate_s', 'iebc_recovery_stop_latched',
            'iebc_recovery_rebase_energy_j',
        ])
        self._last_csv_flush_s = time.monotonic()

        self.get_logger().warn(
            'GAZEBO-ONLY IEBC cube experiment enabled: this node may Arm and '
            'enter Offboard automatically. Never run it with a real flight controller.')
        self.get_logger().info(
            f'Virtual cube load={self.virtual_force_n:.2f} N; release mode={self.release_mode}; '
            f'scheduled release={self.release_time_s:.2f} s; force release threshold='
            f'{self.release_force_threshold_n:.2f} N for {self.release_force_hold_s:.3f} s; '
            f'cube displacement is diagnostic only; yaw tolerance='
            f'{math.degrees(self.yaw_tolerance_rad):.1f} deg; initial yaw trim='
            f'{math.degrees(self.yaw_command_bias_rad):+.1f} deg; auto-align gain='
            f'{self.yaw_align_gain:.1f}; yaw loss limit='
            f'{math.degrees(self.yaw_loss_tolerance_rad):.1f} deg for '
            f'{self.yaw_loss_hold_s:.2f} s; Kc={self.iebc.k_c:.2f} N/m; '
            f'Dc={self.iebc.d_c:.2f} N s/m; Emax={self.iebc.e_max:.2f} J; '
            f'energy reserve={self.iebc.energy_reserve_j:.3f} J; '
            f'stop distance={self.iebc.stop_distance:.2f} m; certified brake force='
            f'{self.iebc.brake_force_cert:.2f} N; '
            f'barrier-active required={self.require_barrier_active}; '
            f'closed-loop static certificate ceiling='
            f'{math.sqrt(2.0 * self.iebc.k_c * self.iebc.e_max):.2f} N; CSV={self.csv_path}')

    # Gazebo transport -------------------------------------------------
    @staticmethod
    def _vector_x(vector) -> float:
        return float(getattr(vector, 'x', 0.0))

    def _contact_callback(self, message: Contacts) -> None:
        max_force_x = 0.0
        for contact in message.contact:
            force_body_1 = 0.0
            force_body_2 = 0.0
            for wrench in contact.wrench:
                if wrench.HasField('body_1_wrench'):
                    force_body_1 += self._vector_x(wrench.body_1_wrench.force)
                if wrench.HasField('body_2_wrench'):
                    force_body_2 += self._vector_x(wrench.body_2_wrench.force)
            max_force_x = max(max_force_x, abs(force_body_1), abs(force_body_2))

        with self._transport_lock:
            self.contact_filter.feed(max_force_x)

    def _pose_callback(self, message: Pose_V) -> None:
        cube_x = math.nan
        cube_y = math.nan
        vehicle_position = None
        vehicle_yaw = math.nan
        for pose in message.pose:
            name = str(pose.name)
            if name == self.CUBE_MODEL or name.endswith(f'::{self.CUBE_MODEL}'):
                cube_x = float(pose.position.x)
                cube_y = float(pose.position.y)
            # /pose/info contains the model and all of its scoped child links.
            # A startswith() match also selected e.g.
            # ``hnuter_contact_0::contact_probe``.  That link has a fixed
            # +90 deg pitch relative to base_link, so extracting its Euler yaw
            # produces an apparent yaw drift during contact even when the
            # vehicle model remains aligned.  Only the unscoped model pose is
            # the physical heading used by the geometry gate.
            elif (name.startswith(self.VEHICLE_MODEL_PREFIX)
                  and '::' not in name):
                vehicle_position = np.array([
                    float(pose.position.x), float(pose.position.y), float(pose.position.z)])
                q = pose.orientation
                vehicle_yaw = math.atan2(
                    2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
                    1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2))
        if math.isfinite(cube_x):
            with self._transport_lock:
                self.cube_x_m = cube_x
                self.cube_y_m = cube_y
                if vehicle_position is not None:
                    self.vehicle_gz_position = vehicle_position
                    self.vehicle_gz_yaw = vehicle_yaw

    def _set_virtual_force(self) -> None:
        message = EntityWrench()
        message.entity.name = self.CUBE_MODEL
        message.entity.type = Entity.MODEL
        message.wrench.force.x = -self.virtual_force_n
        published = bool(self._persistent_wrench_pub.publish(message))
        self.virtual_force_active = True
        marker_visible = self._set_virtual_force_marker()
        self.get_logger().info(
            f'Applied persistent cube virtual force Fx={-self.virtual_force_n:.2f} N '
            f'(Gazebo publish={published}, GUI marker={marker_visible}).')

    def _clear_virtual_force(self) -> None:
        message = Entity()
        message.name = self.CUBE_MODEL
        message.type = Entity.MODEL
        published = bool(self._clear_wrench_pub.publish(message))
        self.virtual_force_active = False
        self._clear_virtual_force_marker()
        self.get_logger().warn(
            f'CLEARED cube virtual force (Gazebo publish={published}); observing vehicle response.')

    @staticmethod
    def _red_marker(marker_id: int, marker_type: int, action: int = Marker.ADD_MODIFY) -> Marker:
        marker = Marker()
        marker.ns = HnuterIebcClosedLoopCubeContactExperiment.FORCE_MARKER_NAMESPACE
        marker.id = marker_id
        marker.action = action
        marker.type = marker_type
        marker.visibility = Marker.GUI
        for color in (marker.material.ambient, marker.material.diffuse, marker.material.emissive):
            color.r = 0.96
            color.g = 0.05
            color.b = 0.03
            color.a = 1.0
        marker.material.lighting = True
        return marker

    @classmethod
    def _virtual_force_markers(cls, force_n: float, cube_x: float, cube_y: float) -> tuple:
        """Build a -world-X arrow whose length encodes the active load."""
        length_m = float(np.clip(0.05 * abs(force_n), 0.40, 1.20))
        marker_y = cube_y - 1.35
        marker_z = 2.75
        tail_x = cube_x - 0.15
        shaft_end_x = tail_x - length_m

        shaft = cls._red_marker(1, Marker.LINE_LIST)
        shaft.scale.x = 0.065
        for x in (tail_x, shaft_end_x):
            point = shaft.point.add()
            point.x = x
            point.y = marker_y
            point.z = marker_z

        head = cls._red_marker(2, Marker.CONE)
        head.pose.position.x = shaft_end_x - 0.15
        head.pose.position.y = marker_y
        head.pose.position.z = marker_z
        # Marker cone axis is local +Z; -90 deg about Y points it along -X.
        head.pose.orientation.w = math.cos(-math.pi / 4.0)
        head.pose.orientation.y = math.sin(-math.pi / 4.0)
        head.scale.x = 0.28
        head.scale.y = 0.28
        head.scale.z = 0.35
        return shaft, head

    def _set_virtual_force_marker(self) -> bool:
        with self._transport_lock:
            cube_x = self.cube_x_m
            cube_y = self.cube_y_m
        if not (math.isfinite(cube_x) and math.isfinite(cube_y)):
            return False

        # MarkerManager's /marker service uses an Empty response. The Python
        # transport binding reports ``False`` for that void response even when
        # the request is accepted and the marker is created. Service discovery
        # is therefore the meaningful availability check here.
        try:
            marker_service_available = '/marker' in self.gz_node.service_list()
        except Exception:
            marker_service_available = False
        if not marker_service_available:
            return False

        for marker in self._virtual_force_markers(self.virtual_force_n, cube_x, cube_y):
            try:
                self.gz_node.request('/marker', marker, Marker, Empty, 100)
            except Exception:
                return False
        return True

    def _clear_virtual_force_marker(self) -> None:
        try:
            if '/marker' not in self.gz_node.service_list():
                return
        except Exception:
            return
        for marker_id in (1, 2):
            marker = self._red_marker(marker_id, Marker.NONE, Marker.DELETE_MARKER)
            try:
                self.gz_node.request('/marker', marker, Marker, Empty, 50)
            except Exception:
                pass

    # PX4 simulation-only authority -----------------------------------
    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        message = VehicleCommand()
        message.command = int(command)
        message.param1 = float(param1)
        message.param2 = float(param2)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        message.timestamp = self.timestamp_now_us()
        self.vehicle_command_pub.publish(message)

    def offboard_startup_tick(self):
        self.publish_offboard_control_mode()
        self._startup_ticks += 1
        self._update_hardware_control_gate()

        if self.data_received and self.px4_timestamp > 0:
            if not self._hardware_control_active:
                self._hold_current_position()
            self.publish_px4_trajectory_setpoint()

        if not self.data_received or self._startup_ticks < 30:
            return

        now_s = time.monotonic()
        if not self.is_offboard() and now_s - self._last_mode_request_s >= 1.0:
            self._publish_vehicle_command(self.cmd_set_mode, param1=1.0, param2=6.0)
            self._last_mode_request_s = now_s
            self.get_logger().info('SITL experiment requesting Offboard mode.')

        if not self.armed and now_s - self._last_arm_request_s >= 1.0:
            self._publish_vehicle_command(self.cmd_arm_disarm, param1=1.0)
            self._last_arm_request_s = now_s
            self.get_logger().info('SITL experiment requesting Arm.')

    # Experiment state machine ----------------------------------------
    def _set_stage(self, stage: str, current_time: float) -> None:
        previous = self.stage
        self.stage = stage
        self.stage_start_s = float(current_time)
        if stage in (self.STAGE_COMPLETE, self.STAGE_FAILED):
            self.terminal_hold_enu = self.position.copy()
        self.get_logger().warn(f'Experiment stage: {previous} -> {stage}')

    def _begin_hardware_control(self):
        super()._begin_hardware_control()
        self.desired_controller_yaw = wrap_pi(
            self.initial_yaw + self.yaw_command_bias_rad)
        self.experiment_origin_enu = self.position.copy()
        self.contact_origin_enu = None
        self.nominal_reference_enu = self.position.copy()
        self.release_target_enu = None
        self.force_release_latch.reset()
        self.release_event_seen = False
        self.release_contact_force_n = math.nan
        self.release_contact_force_raw_n = math.nan
        self.loaded_cube_x = math.nan
        self.max_qp_slack_w = 0.0
        self.qp_infeasible_since_s = None
        self.barrier_active_seen = False
        self.reference_intervention_since_s = None
        self.max_reference_intervention_duration_s = 0.0
        self.max_reference_velocity_reduction_mps = 0.0
        self.min_stop_distance_barrier_m = math.inf
        self.max_release_excursion_m = 0.0
        self.recovery_complete_seen = False
        self.recovery_complete_time_s = math.nan
        self.release_settle_since_s = None
        self.release_settled = False
        self.release_settle_time_s = math.nan
        self.release_settle_anchor_enu = None
        self.release_position_change_m = math.inf
        self.max_recovery_dissipation_slack_w = 0.0
        self.yaw_aligned_since_s = None
        self.yaw_loss_since_s = None
        self._set_virtual_force()
        self._set_stage(self.STAGE_TAKEOFF, self.px4_timestamp / 1_000_000.0 - self.sim_start_time_s)

    def _set_reference(
            self, position_enu: np.ndarray, velocity_enu=None, acceleration_enu=None,
            yaw_enu: float = None) -> None:
        position_enu = np.asarray(position_enu, dtype=float).reshape(3)
        self.nominal_reference_enu = position_enu.copy()
        self.target_position = position_enu.copy()
        self.target_position[2] -= self._z0
        self.target_velocity = (np.zeros(3) if velocity_enu is None
                                else np.asarray(velocity_enu, dtype=float).reshape(3))
        self.target_acceleration = (np.zeros(3) if acceleration_enu is None
                                    else np.asarray(acceleration_enu, dtype=float).reshape(3))
        commanded_yaw = (self.desired_controller_yaw if yaw_enu is None
                         else float(yaw_enu))
        self.target_attitude = np.array([0.0, 0.0, commanded_yaw], dtype=float)
        # PX4 position mode publishes manual_des_yaw, not target_attitude[2].
        # Both must be updated or the aircraft preserves its arbitrary startup
        # heading while translating toward the cube.
        self.manual_des_yaw = commanded_yaw
        self.target_attitude_rate = np.zeros(3)

    def _gazebo_yaw_error(self) -> float:
        with self._transport_lock:
            vehicle_yaw = self.vehicle_gz_yaw
        return (wrap_pi(self.desired_world_yaw - vehicle_yaw)
                if math.isfinite(vehicle_yaw) else math.nan)

    def _update_alignment_yaw_command(self, yaw_error: float, dt: float) -> None:
        """Calibrate PX4's local-yaw command to the Gazebo world heading.

        This Gazebo-only outer loop maintains the physical head-on geometry.
        The independent five-degree gate still aborts a trial that cannot
        track it.
        """
        if not math.isfinite(yaw_error) or dt <= 0.0:
            return
        command_rate = float(np.clip(
            self.yaw_align_gain * yaw_error,
            -self.yaw_align_max_rate_rad_s,
            self.yaw_align_max_rate_rad_s))
        self.desired_controller_yaw = wrap_pi(
            self.desired_controller_yaw + command_rate * dt)

    def _contact_force(self, dt: float) -> tuple:
        with self._transport_lock:
            filtered = self.contact_filter.update(dt)
            raw = self.contact_filter.raw_n
            cube_x = self.cube_x_m
        return raw, filtered, cube_x

    def _should_release(
            self, elapsed: float, contact_force: float, current_time: float) -> bool:
        if self.release_mode == 'time':
            return elapsed >= self.release_time_s
        return self.force_release_latch.update(contact_force, current_time)

    def update_trajectory(self, current_time: float, dt: float):
        if not self._hardware_control_active or self.experiment_origin_enu is None:
            self._hold_current_position()
            return

        elapsed = max(0.0, current_time - self.stage_start_s)
        raw_force, contact_force, cube_x = self._contact_force(dt)
        self._latest_contact_sample = (raw_force, contact_force, cube_x)
        origin = self.experiment_origin_enu
        hover_position = origin + np.array([0.0, 0.0, self.takeoff_height_m])

        # A valid force-limit trial must remain a head-on push, not merely be
        # aligned once before approach. Continue the slow world-heading outer
        # loop because PX4 local yaw and Gazebo model Euler yaw are not a
        # constant offset once contact introduces roll/pitch. Abort if physical
        # yaw still leaves the allowed cone long enough.
        if self.stage in (self.STAGE_APPROACH, self.STAGE_LOAD_SETTLE, self.STAGE_PUSH):
            yaw_error = self._gazebo_yaw_error()
            self._update_alignment_yaw_command(yaw_error, dt)
            yaw_lost = (not math.isfinite(yaw_error)
                        or abs(yaw_error) > self.yaw_loss_tolerance_rad)
            if yaw_lost:
                if self.yaw_loss_since_s is None:
                    self.yaw_loss_since_s = current_time
                elif current_time - self.yaw_loss_since_s >= self.yaw_loss_hold_s:
                    if self.virtual_force_active:
                        self._clear_virtual_force()
                    self._set_stage(self.STAGE_FAILED, current_time)
                    self.get_logger().error(
                        'Physical probe yaw alignment was lost during interaction; '
                        f'error={math.degrees(yaw_error):.2f} deg, limit='
                        f'{math.degrees(self.yaw_loss_tolerance_rad):.2f} deg for '
                        f'{self.yaw_loss_hold_s:.2f} s.')
                    self._write_csv(current_time, raw_force, contact_force, cube_x)
                    return
            else:
                self.yaw_loss_since_s = None

        if self.stage in (
                self.STAGE_LOAD_SETTLE, self.STAGE_PUSH, self.STAGE_RELEASE
        ) and self.iebc.enabled:
            barrier_j = float(self.iebc.debug.get('h_i', self.iebc.e_max))
            self.min_interaction_barrier_j = min(self.min_interaction_barrier_j, barrier_j)
            if barrier_j < -self.barrier_tolerance_j:
                self._clear_virtual_force()
                self._set_stage(self.STAGE_FAILED, current_time)
                self.get_logger().error(
                    f'IEBC barrier violated during interaction: h={barrier_j:.3f} J, '
                    f'tolerance={self.barrier_tolerance_j:.3f} J.')
                self._write_csv(current_time, raw_force, contact_force, cube_x)
                return

            qp_slack_w = float(self.iebc.debug.get('qp_slack_w', 0.0))
            self.max_qp_slack_w = max(self.max_qp_slack_w, qp_slack_w)
            qp_infeasible = (bool(self.iebc.debug.get('infeasible', False))
                             and qp_slack_w > self.qp_slack_tolerance_w)
            if qp_infeasible:
                if self.qp_infeasible_since_s is None:
                    self.qp_infeasible_since_s = current_time
                elif current_time - self.qp_infeasible_since_s >= self.qp_infeasible_hold_s:
                    self._clear_virtual_force()
                    self._set_stage(self.STAGE_FAILED, current_time)
                    self.get_logger().error(
                        'Closed-loop IEBC QP remained infeasible; strict energy '
                        f'guarantee was lost: slack={qp_slack_w:.3f} W for '
                        f'{self.qp_infeasible_hold_s:.2f} s.')
                    self._write_csv(current_time, raw_force, contact_force, cube_x)
                    return
            else:
                self.qp_infeasible_since_s = None

        if self.stage == self.STAGE_TAKEOFF:
            u, du, ddu = smoothstep01(elapsed / self.takeoff_time_s)
            position = origin + np.array([0.0, 0.0, self.takeoff_height_m * u])
            velocity = np.array([0.0, 0.0, self.takeoff_height_m * du / self.takeoff_time_s])
            acceleration = np.array([0.0, 0.0, self.takeoff_height_m * ddu / self.takeoff_time_s ** 2])
            self._set_reference(position, velocity, acceleration, yaw_enu=self.initial_yaw)
            if elapsed >= self.takeoff_time_s and abs(self.position[2] - hover_position[2]) < 0.20:
                self.yaw_aligned_since_s = None
                self._set_stage(self.STAGE_ALIGN, current_time)

        elif self.stage == self.STAGE_ALIGN:
            yaw_error = self._gazebo_yaw_error()
            self._update_alignment_yaw_command(yaw_error, dt)
            self._set_reference(hover_position)
            if math.isfinite(yaw_error) and abs(yaw_error) <= self.yaw_tolerance_rad:
                if self.yaw_aligned_since_s is None:
                    self.yaw_aligned_since_s = current_time
                elif current_time - self.yaw_aligned_since_s >= self.yaw_hold_s:
                    self.yaw_loss_since_s = None
                    self._set_stage(self.STAGE_APPROACH, current_time)
            else:
                self.yaw_aligned_since_s = None

            if elapsed >= self.yaw_timeout_s:
                self._set_stage(self.STAGE_FAILED, current_time)
                self.get_logger().error(
                    'Physical probe yaw did not align with the cube before timeout; '
                    f'error={math.degrees(yaw_error):.2f} deg.')

        elif self.stage == self.STAGE_APPROACH:
            distance = min(self.approach_speed_mps * elapsed, self.approach_distance_m)
            position = hover_position + self.interaction_axis_enu * distance
            velocity = (self.interaction_axis_enu * self.approach_speed_mps
                        if distance < self.approach_distance_m else np.zeros(3))
            self._set_reference(position, velocity)

            if contact_force >= 0.20:
                self.contact_origin_enu = self.position.copy()
                self.iebc.enabled = self._iebc_requested
                self.iebc.reset()
                self.contact_filter.filtered_n = 0.0
                self.force_release_latch.reset()
                self._set_stage(self.STAGE_LOAD_SETTLE, current_time)
            elif distance >= self.approach_distance_m and elapsed > (
                    self.approach_distance_m / self.approach_speed_mps + 3.0):
                self._set_stage(self.STAGE_FAILED, current_time)
                self.get_logger().error('No probe/cube contact detected within the configured approach distance.')

        elif self.stage == self.STAGE_LOAD_SETTLE:
            self._set_reference(self.contact_origin_enu)
            if elapsed >= self.load_settle_s:
                self.loaded_cube_x = cube_x
                self._set_stage(self.STAGE_PUSH, current_time)

        elif self.stage == self.STAGE_PUSH:
            push_distance = min(self.push_speed_mps * elapsed, self.max_push_distance_m)
            nominal = self.contact_origin_enu + self.interaction_axis_enu * push_distance
            velocity = (self.interaction_axis_enu * self.push_speed_mps
                        if push_distance < self.max_push_distance_m else np.zeros(3))
            self._set_reference(nominal, velocity)

            if self._should_release(elapsed, contact_force, current_time):
                # The old forward nominal point is not a braking target.  Keep
                # the ordinary nominal layer at the measured release pose;
                # RECOVERY ignores its interaction-axis coordinate and evolves
                # the continuous safe_s state under the two hard barriers.
                self.release_target_enu = self.position.copy()
                self.release_vehicle_position_enu = self.position.copy()
                self.release_vehicle_velocity_enu = self.velocity.copy()
                self.release_cube_x = cube_x
                self.release_contact_force_n = contact_force
                self.release_contact_force_raw_n = raw_force
                self._clear_virtual_force()
                if self.iebc.enabled:
                    measured_s = float(np.dot(
                        self.interaction_axis_enu, self.position))
                    self.iebc.enter_recovery(measured_s)
                self.release_event_seen = True
                self._set_stage(self.STAGE_RELEASE, current_time)

            elif elapsed >= self.max_push_time_s:
                self._clear_virtual_force()
                self._set_stage(self.STAGE_FAILED, current_time)
                if self.release_mode == 'time':
                    self.get_logger().error(
                        f'Scheduled release at {self.release_time_s:.2f} s was not executed '
                        f'before push timeout {self.max_push_time_s:.2f} s.')
                else:
                    self.get_logger().error(
                        f'Contact force did not reach {self.release_force_threshold_n:.2f} N '
                        f'for {self.release_force_hold_s:.3f} s before the push timeout; '
                        f'filtered contact force={contact_force:.2f} N, '
                        f'cube travel={cube_x - self.loaded_cube_x:.3f} m (diagnostic only).')

        elif self.stage == self.STAGE_RELEASE:
            self._set_reference(
                self.release_target_enu, np.zeros(3), np.zeros(3))
            speed = float(np.linalg.norm(self.velocity))
            displacement = float(np.linalg.norm(self.position - self.release_vehicle_position_enu))
            self.peak_post_release_speed_mps = max(self.peak_post_release_speed_mps, speed)
            self.peak_post_release_position_delta_m = max(
                self.peak_post_release_position_delta_m, displacement)
            evaluation_due = self.release_settled or elapsed >= self.release_observe_s
            if evaluation_due:
                failed_reasons = []
                if not self.release_event_seen:
                    failed_reasons.append('scheduled release was not observed')
                if self.iebc.enabled:
                    if self.min_interaction_barrier_j < -self.barrier_tolerance_j:
                        failed_reasons.append(
                            f'minimum barrier {self.min_interaction_barrier_j:.3f} J')
                    if self.max_qp_slack_w > self.qp_slack_tolerance_w:
                        failed_reasons.append(
                            f'maximum QP slack {self.max_qp_slack_w:.3f} W')
                    if self.min_stop_distance_barrier_m < -self.stop_barrier_tolerance_m:
                        failed_reasons.append(
                            'minimum stopping-distance barrier '
                            f'{self.min_stop_distance_barrier_m:.3f} m')
                if not self.release_settled:
                    failed_reasons.append(
                        'position did not settle before timeout: '
                        f'position change={self.release_position_change_m:.3f} m '
                        f'(limit {self.release_settle_position_tol_m:.3f} m)')
                if self.require_barrier_active:
                    if not self.barrier_active_seen:
                        failed_reasons.append('barrier never became active before release')
                    if self.max_reference_intervention_duration_s < self.intervention_hold_s:
                        failed_reasons.append(
                            'safe reference was not below nominal for the required '
                            f'{self.intervention_hold_s:.2f} s')

                if failed_reasons:
                    self._set_stage(self.STAGE_FAILED, current_time)
                    self.get_logger().error(
                        'EXPERIMENT FAILED: ' + '; '.join(failed_reasons) +
                        f'; CSV={self.csv_path}')
                    return

                self._set_stage(self.STAGE_COMPLETE, current_time)
                cube_delta = cube_x - self.release_cube_x if (
                    math.isfinite(cube_x) and math.isfinite(self.release_cube_x)) else math.nan
                self.get_logger().warn(
                    f'EXPERIMENT COMPLETE: virtual load was cleared by {self.release_mode} release; '
                    f'release force={self.release_contact_force_n:.3f} N '
                    f'(raw={self.release_contact_force_raw_n:.3f} N); '
                    f'barrier_seen={self.barrier_active_seen}, '
                    f'max intervention={self.max_reference_intervention_duration_s:.3f} s; '
                    f'recovery mode={self.iebc.mode}, stop time='
                    f'{self.recovery_complete_time_s:.3f} s, min h_D='
                    f'{self.min_stop_distance_barrier_m:.3f} m; '
                    f'settled in {self.release_settle_time_s:.3f} s with '
                    f'1 s position change={self.release_position_change_m:.3f} m; '
                    f'post-release peak vehicle speed={self.peak_post_release_speed_mps:.3f} m/s, '
                    f'peak vehicle displacement={self.peak_post_release_position_delta_m:.3f} m, '
                    f'cube dx={cube_delta:.3f} m, CSV={self.csv_path}')

        elif self.stage in (self.STAGE_COMPLETE, self.STAGE_FAILED):
            hold = (self.release_target_enu if self.release_target_enu is not None
                    else self.terminal_hold_enu)
            self._set_reference(hold)

        # CSV is written by _after_iebc_reference_update(), after the nominal
        # reference has been filtered, so nominal and safe values share a sample.

    def _after_iebc_reference_update(self, current_time: float, dt: float) -> None:
        del dt
        debug = self.iebc.debug
        if self.stage == self.STAGE_PUSH and self.iebc.enabled:
            self.barrier_active_seen = (
                self.barrier_active_seen
                or bool(debug.get('barrier_active', False)))
            velocity_reduction = max(
                0.0,
                float(debug.get('v_task_i', 0.0))
                - float(debug.get('v_safe_i', 0.0)))
            self.max_reference_velocity_reduction_mps = max(
                self.max_reference_velocity_reduction_mps, velocity_reduction)
            if velocity_reduction > self.intervention_velocity_tolerance_mps:
                if self.reference_intervention_since_s is None:
                    self.reference_intervention_since_s = current_time
                duration = current_time - self.reference_intervention_since_s
                self.max_reference_intervention_duration_s = max(
                    self.max_reference_intervention_duration_s, duration)
            else:
                self.reference_intervention_since_s = None
        else:
            self.reference_intervention_since_s = None

        if self.stage == self.STAGE_RELEASE and self.iebc.enabled:
            stop_barrier = float(debug.get('stop_distance_barrier', math.inf))
            excursion = float(debug.get('release_excursion', 0.0))
            self.min_stop_distance_barrier_m = min(
                self.min_stop_distance_barrier_m, stop_barrier)
            self.max_release_excursion_m = max(
                self.max_release_excursion_m, excursion)
            self.max_recovery_dissipation_slack_w = max(
                self.max_recovery_dissipation_slack_w,
                float(debug.get('recovery_dissipation_slack_w', 0.0)))
            if (not self.recovery_complete_seen
                    and debug.get('mode') == self.iebc.MODE_HOLD):
                self.recovery_complete_seen = True
                self.recovery_complete_time_s = max(
                    0.0, current_time - self.stage_start_s)

            if debug.get('mode') == self.iebc.MODE_HOLD:
                if self.release_settle_anchor_enu is None:
                    self.release_settle_anchor_enu = self.position.copy()
                    self.release_settle_since_s = current_time
                    self.release_position_change_m = 0.0
                else:
                    self.release_position_change_m = float(np.linalg.norm(
                        self.position - self.release_settle_anchor_enu))
                    if (self.release_position_change_m
                            > self.release_settle_position_tol_m):
                        self.release_settle_anchor_enu = self.position.copy()
                        self.release_settle_since_s = current_time
                        self.release_position_change_m = 0.0
                    elif (not self.release_settled
                          and current_time - self.release_settle_since_s
                          >= self.release_settle_hold_s):
                        self.release_settled = True
                        self.release_settle_time_s = max(
                            0.0, current_time - self.stage_start_s)
            else:
                self.release_settle_since_s = None
                self.release_settle_anchor_enu = None
                self.release_position_change_m = math.inf

        raw_force, filtered_force, cube_x = self._latest_contact_sample
        self._write_csv(current_time, raw_force, filtered_force, cube_x)

    def _write_csv(self, current_time: float, raw_force: float, filtered_force: float, cube_x: float) -> None:
        debug = self.iebc.debug
        yaw_error = self._gazebo_yaw_error()
        cube_breakaway_m = (cube_x - self.loaded_cube_x if
                            math.isfinite(cube_x) and math.isfinite(self.loaded_cube_x)
                            else math.nan)
        contact_power_gt_w = filtered_force * float(
            np.dot(self.interaction_axis_enu, self.velocity))
        self._csv.writerow([
            f'{current_time:.6f}', self.stage,
            *(f'{value:.6f}' for value in self.position),
            *(f'{value:.6f}' for value in self.velocity),
            *(f'{value:.6f}' for value in self.target_position),
            f'{math.degrees(self.vehicle_gz_yaw):.6f}',
            f'{math.degrees(self.desired_world_yaw):.6f}',
            f'{math.degrees(self.desired_controller_yaw):.6f}',
            f'{math.degrees(yaw_error):.6f}',
            f'{cube_x:.6f}', f'{raw_force:.6f}', f'{filtered_force:.6f}',
            f'{self.release_force_threshold_n:.6f}', f'{self.release_force_hold_s:.6f}',
            self.release_mode, f'{self.release_time_s:.6f}',
            f'{cube_breakaway_m:.6f}', int(self.virtual_force_active), f'{self.virtual_force_n:.6f}',
            int(self.release_event_seen),
            int(bool(debug.get('active', False))),
            int(bool(debug.get('barrier_active', False))),
            int(bool(debug.get('infeasible', False))),
            int(bool(debug.get('storage_update_enabled', False))),
            int(self.barrier_active_seen),
            f'{self.max_reference_intervention_duration_s:.6f}',
            f'{self.max_reference_velocity_reduction_mps:.6f}',
            f"{debug.get('p_hat', 0.0):.6f}",
            f"{debug.get('p_bar_e', 0.0):.6f}",
            f"{debug.get('s_dot_bar', 0.0):.6f}",
            f"{debug.get('s_bar', 0.0):.6f}",
            f"{debug.get('k_i', 0.0):.6f}",
            f"{debug.get('v_c', 0.0):.6f}",
            f"{debug.get('e_i', 0.0):.6f}",
            f"{debug.get('h_i', 0.0):.6f}",
            f"{debug.get('h_constraint', 0.0):.6f}",
            f"{debug.get('energy_reserve_j', 0.0):.6f}",
            f'{self.min_interaction_barrier_j:.6f}',
            f"{debug.get('e_ref', 0.0):.6f}",
            f"{debug.get('s_nom_i', 0.0):.6f}",
            f"{debug.get('s_safe_i', 0.0):.6f}",
            f"{debug.get('v_i', 0.0):.6f}",
            f"{debug.get('v_nom_i', 0.0):.6f}",
            f"{debug.get('v_task_i', 0.0):.6f}",
            f"{debug.get('v_safe_i', 0.0):.6f}",
            f"{debug.get('g_e', 0.0):.6f}",
            f"{debug.get('pi_e', 0.0):.6f}",
            f"{debug.get('p_allow', 0.0):.6f}",
            f"{debug.get('p_ref_nominal', 0.0):.6f}",
            f"{debug.get('p_ref_safe', 0.0):.6f}",
            f"{debug.get('equivalent_stiffness_force_n', 0.0):.6f}",
            f'{contact_power_gt_w:.6f}',
            f"{debug.get('qp_slack_w', 0.0):.6f}", f'{self.max_qp_slack_w:.6f}',
            f"{debug.get('a_safe_i', 0.0):.6f}",
            str(debug.get('mode', 'disabled')),
            f"{debug.get('recoverable_energy', 0.0):.6f}",
            f"{debug.get('release_excursion', 0.0):.6f}",
            f"{debug.get('stop_distance_barrier', math.inf):.6f}",
            f"{debug.get('reserved_stop_distance', 0.0):.6f}",
            f"{debug.get('rho', 0.0):.6f}",
            f"{debug.get('release_s', math.nan):.6f}",
            int(self.recovery_complete_seen),
            f'{self.recovery_complete_time_s:.6f}',
            f"{debug.get('recovery_dissipation_slack_w', 0.0):.6f}",
            f'{self.release_position_change_m:.6f}',
            int(self.release_settled),
            f'{self.release_settle_time_s:.6f}',
            f'{self.max_recovery_dissipation_slack_w:.6f}',
            str(debug.get('recovery_phase', 'inactive')),
            f"{debug.get('recovery_reference_velocity', 0.0):.6f}",
            int(bool(debug.get('recovery_rate_infeasible', False))),
            f"{debug.get('recovery_terminal_s', math.nan):.6f}",
            f"{debug.get('recovery_stop_candidate_s', 0.0):.6f}",
            int(bool(debug.get('recovery_stop_latched', False))),
            f"{debug.get('recovery_rebase_energy_j', 0.0):.6f}",
        ])
        now_s = time.monotonic()
        if now_s - self._last_csv_flush_s >= 1.0:
            self._csv_file.flush()
            self._last_csv_flush_s = now_s

    def print_status(self):
        super().print_status()
        if self.data_received:
            _, filtered_force, cube_x = self._contact_force(0.02)
            self.get_logger().info(
                f'Cube experiment: stage={self.stage} | contact={filtered_force:.2f} N | '
                f'virtual_load={self.virtual_force_active} | cube_x={cube_x:.3f} m | '
                f'yaw={math.degrees(self.vehicle_gz_yaw):+.1f} deg -> '
                f'{math.degrees(self.desired_world_yaw):+.1f} deg | '
                f'release_seen={self.release_event_seen} | IEBC mode={self.iebc.mode}')

    def destroy_node(self):
        try:
            if self.virtual_force_active:
                self._clear_virtual_force()
        except Exception:
            pass
        try:
            self._csv_file.flush()
            self._csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = HnuterIebcClosedLoopCubeContactExperiment()
    exit_code = 1
    try:
        while rclpy.ok() and controller.stage not in (
                controller.STAGE_COMPLETE, controller.STAGE_FAILED):
            rclpy.spin_once(controller, timeout_sec=0.1)
        exit_code = 0 if controller.stage == controller.STAGE_COMPLETE else 1
    except KeyboardInterrupt:
        controller.get_logger().info('Cube-contact experiment interrupted.')
        exit_code = 130
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
