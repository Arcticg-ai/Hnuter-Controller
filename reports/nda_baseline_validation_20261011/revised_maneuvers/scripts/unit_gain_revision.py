"""Test unit DC joint gain without changing lag, rate bounds, or outer gains."""
import os
import rclpy
from controllers.experiments.maneuver_revision import RevisedManeuverMixin
from controllers.experiments.fixed_attitude_switching.controller import controller_class


class UnitGainMixin(RevisedManeuverMixin):
    def __init__(self):
        super().__init__()
        if hasattr(self,'drcda'):
            self.drcda.config.servo_gain_positive[:]=1.
            self.drcda.config.servo_gain_negative[:]=1.
        self._record_revision()


def main():
    rclpy.init()
    cls=type('UnitGainRevisionController',(UnitGainMixin,controller_class(os.environ['HNUTER_SWITCH_METHOD'])),{})
    node=cls()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
