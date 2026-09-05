#!/usr/bin/env python3
"""SITL controller for paper NDA and the independent DRCDA v2 allocator."""

from __future__ import annotations

import os
from pathlib import Path

from controllers.experiments.drcda_v2.allocator import (
    PaperNormalizedDifferentialAllocator,
    ReachabilityNormalizedDRCDAAllocator,
)
from controllers.simulation.hnuter_external_direct_drcda import (
    HnuterDRCDAController,
)

import rclpy


V2_VARIANTS = ('paper_nda', 'drcda_v2')


class HnuterDRCDAv2Controller(HnuterDRCDAController):
    def __init__(self) -> None:
        requested = os.environ.get(
            'HNUTER_DRCDA_V2_VARIANT', 'drcda_v2'
        ).strip().lower()
        if requested not in V2_VARIANTS:
            raise ValueError(
                f'unknown HNUTER_DRCDA_V2_VARIANT={requested!r}; '
                f'choose from {", ".join(V2_VARIANTS)}'
            )

        os.environ.setdefault(
            'HNUTER_TUNING_FILE',
            str(
                Path(__file__).resolve().parents[3]
                / 'config/experiments/drcda_v2_tuning.json'
            ),
        )
        original_variant = os.environ.get('HNUTER_DRCDA_VARIANT')
        os.environ['HNUTER_DRCDA_VARIANT'] = 'full'
        super().__init__()
        if original_variant is None:
            os.environ.pop('HNUTER_DRCDA_VARIANT', None)
        else:
            os.environ['HNUTER_DRCDA_VARIANT'] = original_variant

        model = self.drcda.model
        config = self.drcda.config
        if requested == 'paper_nda':
            # The paper baseline uses actuator feedback plus identified rate
            # limits, without DRCDA's command-history reachability predictor.
            config.servo_gain_positive[:] = 1.0
            config.servo_gain_negative[:] = 1.0
            config.servo_tau_positive_s[:] = config.prediction_dt_s * 0.25
            config.servo_tau_negative_s[:] = config.prediction_dt_s * 0.25
            config.servo_delay_positive_s[:] = 0.0
            config.servo_delay_negative_s[:] = 0.0
            allocator = PaperNormalizedDifferentialAllocator(
                model,
                config,
                nullspace_gain=float(os.environ.get('HNUTER_PAPER_NDA_NULLSPACE_GAIN', '0.08')),
            )
            self._drcda_model_name = 'paper_rate_normalized_no_delay'
        else:
            allocator = ReachabilityNormalizedDRCDAAllocator(
                model,
                config,
                lm_damping=float(os.environ.get('HNUTER_DRCDA_V2_DAMPING', '0.0002')),
                max_normalized_step=float(
                    os.environ.get('HNUTER_DRCDA_V2_MAX_STEP', '0.65')
                ),
                line_search_steps=int(
                    os.environ.get('HNUTER_DRCDA_V2_LINE_SEARCH_STEPS', '5')
                ),
            )
            self._drcda_model_name = 'reachability_normalized_v2'
        allocator.reset(
            angle_state=self.drcda.state[:4],
            thrust_state=self.drcda.state[4:],
        )
        self.drcda = allocator
        self._drcda_variant = requested
        self._load_tuning_file(force=True)
        self.get_logger().info(
            f'Experimental allocator selected: {requested}; '
            'baseline DRCDA files remain unchanged.'
        )

    def _apply_tuning(self, data: dict):
        requested = getattr(self, '_drcda_variant', 'full')
        if requested in V2_VARIANTS:
            self._drcda_variant = 'full'
            try:
                super()._apply_tuning(data)
            finally:
                self._drcda_variant = requested
            allocator = getattr(self, 'drcda', None)
            if isinstance(allocator, ReachabilityNormalizedDRCDAAllocator):
                allocator.lm_damping = max(
                    self._tuning_float(
                        data, 'drcda_v2_lm_damping', allocator.lm_damping
                    ),
                    1e-10,
                )
                allocator.max_normalized_step = self._tuning_float(
                    data,
                    'drcda_v2_max_normalized_step',
                    allocator.max_normalized_step,
                )
                allocator.max_normalized_step = float(
                    max(0.05, min(allocator.max_normalized_step, 2.0))
                )
                allocator.line_search_steps = max(
                    int(self._tuning_float(
                        data,
                        'drcda_v2_line_search_steps',
                        allocator.line_search_steps,
                    )),
                    1,
                )
            if isinstance(allocator, PaperNormalizedDifferentialAllocator):
                allocator.nullspace_gain = max(
                    self._tuning_float(
                        data, 'paper_nda_nullspace_gain', allocator.nullspace_gain
                    ),
                    0.0,
                )
            return
        super()._apply_tuning(data)


def main(args=None):
    rclpy.init(args=args)
    controller = HnuterDRCDAv2Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Experimental DRCDA controller stopped.')
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
