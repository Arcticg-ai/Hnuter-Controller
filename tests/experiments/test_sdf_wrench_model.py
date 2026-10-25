from pathlib import Path

import numpy as np
import pytest

from controllers.experiments.sdf_wrench_model import SDFWrenchModel

SDF=Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl/Tools/simulation/gz/models/hnuter/model.sdf')


@pytest.fixture
def model():
    if not SDF.exists():pytest.skip('no local Hnuter SITL SDF')
    return SDFWrenchModel(SDF)


def test_zero_geometry_matches_actual_sdf(model):
    assert model.mass==pytest.approx(4.5,abs=1e-6)
    assert model.positions[:2,1].mean()-model.positions[2:4,1].mean()==pytest.approx(.82,abs=1e-5)
    np.testing.assert_allclose(model.reaction_ratios,[.016]*4+[.02112])
    np.testing.assert_allclose(model.zero_directions[2:],[ [0,0,1] ]*3,atol=1e-7)
    assert model.zero_directions[0,1]==pytest.approx(.0593949,abs=1e-6)


@pytest.mark.parametrize('angles',[[0,0,0,0],[.3,-.4,.2,.5],[1.4,1.0,-1.1,-.8],[3.,.8,-3.,-.7]])
def test_full_geometry_jacobian(model,angles):
    q=np.array([*angles,11.,9.,12.,8.,-2.])
    numerical=np.column_stack([(model.wrench(q+np.eye(9)[j]*1e-6)
                               -model.wrench(q-np.eye(9)[j]*1e-6))/(2e-6) for j in range(9)])
    np.testing.assert_allclose(model.jacobian(q),numerical,rtol=1e-6,atol=1e-7)


def test_rotor_axes_and_moment_arm_are_not_frozen(model):
    p,d,dp,dd=model.geometry(np.array([.4,.8,-.2,.3]))
    assert np.linalg.norm(p-model.positions)>.01
    np.testing.assert_allclose(np.linalg.norm(d,axis=1),1,atol=1e-12)
    np.testing.assert_allclose(d[0],d[1],atol=1e-7)
    np.testing.assert_allclose(np.cross(p[2]-p[3],d[2]),0,atol=1e-7)
    assert np.linalg.norm(dp)>.01 and np.linalg.norm(dd)>.1
