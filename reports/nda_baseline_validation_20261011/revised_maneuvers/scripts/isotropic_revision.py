"""Allocation-only candidate: near-term force tracking in all body axes."""
import os
import rclpy
from controllers.experiments.maneuver_revision import RevisedManeuverMixin
from controllers.experiments.fixed_attitude_switching.controller import controller_class


class IsotropicNearForceMixin(RevisedManeuverMixin):
    def __init__(self):
        super().__init__()
        if hasattr(self,'drcda') and self._switch_method not in {'paper_nda','no_nearfield'}:
            cfg=self.drcda.config
            # Same physical N^-2 penalty for Fx,Fy,Fz, not just body-z lift.
            value=cfg.immediate_wrench_weight[2]/cfg.wrench_scale[2]**2
            cfg.immediate_wrench_weight[:3]=value*cfg.wrench_scale[:3]**2
        self._record_revision()


def main():
    rclpy.init()
    cls=type('IsotropicRevisionController',(IsotropicNearForceMixin,controller_class(os.environ['HNUTER_SWITCH_METHOD'])),{})
    node=cls()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
