"""Qualification using the frozen revision controller and identical gains."""
import os
import numpy as np
import rclpy
from controllers.experiments.maneuver_revision import RevisedManeuverMixin
from controllers.experiments.fixed_attitude_switching.controller import controller_class


class QualificationMixin(RevisedManeuverMixin):
    def __init__(self):
        super().__init__()
        self.lissajous_period_s=20.

    def _switch_reference(self,elapsed):
        prelude=self.switch_ingress_s+self.switch_settle_s
        if self.maneuver_scenario=='qualification_hover' and elapsed>=prelude:
            done=elapsed-prelude>=60.
            return self._switch_a.copy(),np.zeros(3),np.zeros(3),'complete' if done else 'hover',done
        return super()._switch_reference(elapsed)


def main():
    rclpy.init()
    cls=type('QualificationController',(QualificationMixin,controller_class(os.environ['HNUTER_SWITCH_METHOD'])),{})
    node=cls()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
