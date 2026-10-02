import numpy as np

from controllers.experiments.narrow_portal_course.controller import (
    GATES,
    build_course,
    course_waypoints,
    cubic_hermite,
)


def test_cubic_hermite_preserves_endpoint_position_and_velocity():
    start = np.array([0.0, 0.0, 1.0])
    finish = np.array([2.0, 1.0, 1.2])
    start_velocity = np.array([0.4, 0.0, 0.0])
    finish_velocity = np.array([0.2, 0.2, 0.0])
    position, velocity, _ = cubic_hermite(
        start, finish, start_velocity, finish_velocity, 0.0, 2.0
    )
    np.testing.assert_allclose(position, start)
    np.testing.assert_allclose(velocity, start_velocity)
    position, velocity, _ = cubic_hermite(
        start, finish, start_velocity, finish_velocity, 2.0, 2.0
    )
    np.testing.assert_allclose(position, finish)
    np.testing.assert_allclose(velocity, finish_velocity)


def test_course_centers_match_all_six_physical_gates():
    waypoints = course_waypoints(np.array([0.0, 0.0, 1.3]))
    by_name = {waypoint.name: waypoint for waypoint in waypoints}
    for gate in GATES:
        waypoint = by_name[f'{gate.name}_center']
        np.testing.assert_allclose(
            waypoint.position[:2], [gate.x_m, gate.center_y_m]
        )
        assert gate.lower_z_m < waypoint.position[2] < gate.upper_z_m


def test_speed_scale_shortens_course_without_moving_waypoints():
    waypoints = course_waypoints(np.array([0.0, 0.0, 1.3]))
    positions_a, _, _, times_a = build_course(waypoints, 1.0)
    positions_b, _, _, times_b = build_course(waypoints, 1.5)
    np.testing.assert_allclose(positions_a, positions_b)
    assert times_b[-1] < times_a[-1]
