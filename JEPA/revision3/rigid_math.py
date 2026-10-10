"""Frame algebra shared by sensor reconstruction and offline evaluation."""
import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    k = skew(axis)
    return np.eye(3) + np.sin(angle)*k + (1-np.cos(angle))*(k@k)


def quat_rotation(q):
    q = np.asarray(q, float) / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def parallel_axis(m, d):
    d = np.asarray(d)
    return m*(d@d*np.eye(3)-np.outer(d, d))


def physical_inertia(j, margin=1e-8):
    j = np.asarray(j)
    if j.shape != (3, 3) or not np.all(np.isfinite(j)) or not np.allclose(j, j.T, atol=1e-12):
        return False
    eig = np.linalg.eigvalsh(j)
    return bool(eig[0] > margin and eig[2] <= eig[0]+eig[1]+margin)


def kinematics(geometry, joints):
    """All positions/velocities in base FLU. qdot is actual slowed joint rate.

    Geometry contains fixed transforms only, no payload mass or inertia.
    Articulation is about the joint frame, not about the child link origin.
    """
    links = geometry['links']
    parents = {j['child']: j for j in geometry['joints'].values()}
    result = {}

    def visit(name):
        if name in result:
            return result[name]
        if name not in parents:
            value = (np.asarray(links[name]['T0']), np.zeros(3), np.zeros(3))
        else:
            j = parents[name]
            tp, wp, vp = visit(j['parent'])
            tj = tp @ np.linalg.inv(np.asarray(links[j['parent']]['T0'])) @ np.asarray(j['T0'])
            vj = vp + np.cross(wp, tj[:3, 3]-tp[:3, 3])
            angle, speed = joints.get(j['name'], (0., 0.))
            if j['type'] == 'fixed':
                angle = speed = 0.
            tr = np.eye(4)
            tr[:3, :3] = rotation(j['axis'], angle)
            tc = tj @ tr @ np.linalg.inv(np.asarray(j['T0'])) @ np.asarray(links[name]['T0'])
            wc = wp + tj[:3, :3] @ np.asarray(j['axis']) * speed
            vc = vj + np.cross(wc, tc[:3, 3]-tj[:3, 3])
            value = (tc, wc, vc)
        result[name] = value
        return value
    for name in links:
        visit(name)
    return result
