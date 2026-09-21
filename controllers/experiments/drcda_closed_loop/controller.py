"""SITL-only DRCDA with measured joints and a reachable-wrench interface."""

from __future__ import annotations

import math

import numpy as np
import rclpy

from controllers.experiments.drcda_closed_loop.feedback import JointAngleFeedback
from controllers.experiments.drcda_closed_loop.governor import (
    MeasuredDRCDAAllocator,
    ReachableWrenchContract,
)
from controllers.simulation.hnuter_external_direct_drcda import HnuterDRCDAController


class HnuterClosedLoopDRCDAController(HnuterDRCDAController):
    def _node_name(self):
        return 'hnuter_controller_drcda_closed_loop'

    def _diagnostic_file_prefix(self):
        return 'experiments/drcda_closed_loop/hnuter_drcda_closed_loop'

    def __init__(self):
        self._feedback = None
        self._contract = None
        self._joint_age_s = math.inf
        self._feedback_available = False
        super().__init__()
        original = self.drcda
        self.drcda = MeasuredDRCDAAllocator(original.model, original.config)
        self._feedback = JointAngleFeedback()
        self.get_logger().info('Closed-loop DRCDA: Gazebo joint-angle feedback required')

    def _preferred_drcda_command(self, motor_controls, alpha1, alpha2, theta1, theta2):
        return super()._preferred_drcda_command(
            motor_controls, alpha1, alpha2, theta1, theta2
        )

    def _post_drcda_allocation(self, result, previous_command, dt, angle_limits):
        self._contract = ReachableWrenchContract.from_allocation(
            self.drcda, result, previous_command, dt, angle_limits,
            float(self.angular_velocity_frd[2]), float(self.J[2, 2]),
        )

    def _apply_drcda_antiwindup(self, wrench_residual, dt):
        if self._contract is not None:
            wrench_residual = (
                self._contract.reachable_wrench - self._contract.requested_wrench
            )
        return super()._apply_drcda_antiwindup(wrench_residual, dt)

    def _drcda_servo_output_state(self):
        command = self.drcda.command[:4]
        cfg = self.drcda.config
        gains = np.where(command >= 0.0, cfg.servo_gain_positive,
                         cfg.servo_gain_negative)
        return command * gains

    def _direct_prearm_failure_reason(self):
        reason = super()._direct_prearm_failure_reason()
        if reason:
            return reason
        if self._feedback is not None and self.takeoff_requested:
            measured, age = self._feedback.read()
            if measured is None:
                return f'Gazebo joint feedback absent or stale ({age:.2f}s)'
        return ''

    def _diagnostic_extra_header(self):
        return super()._diagnostic_extra_header() + [
            'cl_joint_feedback_age_s', 'cl_joint_feedback_available',
            'cl_reachability_gap_norm', 'cl_predicted_yaw_rate_rad_s',
            *[f'cl_raw_wrench_{axis}' for axis in ('fx', 'fy', 'fz', 'tx', 'ty', 'tz')],
            *[f'cl_reachable_wrench_{axis}' for axis in ('fx', 'fy', 'fz', 'tx', 'ty', 'tz')],
            *[f'cl_reachable_rate_{axis}' for axis in ('fx', 'fy', 'fz', 'tx', 'ty', 'tz')],
            *[f'cl_measured_joint_{axis}_rad' for axis in ('a_l', 'b_l', 'a_r', 'b_r')],
            *[f'cl_commanded_joint_{axis}_rad' for axis in ('a_l', 'b_l', 'a_r', 'b_r')],
        ]

    def _diagnostic_extra_values(self):
        contract = self._contract
        return super()._diagnostic_extra_values() + [
            self._joint_age_s if math.isfinite(self._joint_age_s) else float('nan'),
            int(self._feedback_available),
            float(np.linalg.norm(contract.requested_wrench - contract.reachable_wrench))
            if contract else float('nan'),
            contract.predicted_yaw_rate_frd if contract else float('nan'),
            *([*contract.requested_wrench] if contract else [float('nan')] * 6),
            *([*contract.reachable_wrench] if contract else [float('nan')] * 6),
            *([*contract.reachable_rate] if contract else [float('nan')] * 6),
            *([*self.drcda.measured_angles] if self.drcda.measured_angles is not None
              else [float('nan')] * 4),
            *self._drcda_servo_output_state(),
        ]

    def publish_direct_actuator_setpoint(self, motor_controls, alpha1, alpha2, theta1, theta2):
        active = (self._drcda_ready and self._drcda_active_call
                  and self.armed and self.takeoff_requested)
        if not active:
            self._contract = None
            self.drcda.measured_angles = None
            return super().publish_direct_actuator_setpoint(
                motor_controls, alpha1, alpha2, theta1, theta2
            )

        self._feedback.stop_calibration()
        measured, self._joint_age_s = self._feedback.read()
        self._feedback_available = measured is not None
        if measured is None:
            # Stale physical state invalidates the model interface.
            self.drcda.measured_angles = None
            self._contract = None
            self.get_logger().error('Gazebo joint feedback stale; refusing DRCDA output')
            return self.publish_idle_direct_actuator_setpoint()
        self.drcda.measured_angles = measured
        self.drcda.state[:4] = measured

        return super().publish_direct_actuator_setpoint(
            motor_controls, alpha1, alpha2, theta1, theta2
        )


def main(args=None):
    rclpy.init(args=args)
    controller = HnuterClosedLoopDRCDAController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
