"""Five-parameter angular-momentum estimator; no SDF, ULog or truth imports.

    theta = [rx(m), ry(m), Jxx, Jyy, Jzz (kg m^2)] at the COM.
    R maps body FRD to NED; external actuator wrench is about fixed body O.
    Known rz, products of inertia, calibration and bounded torque bias required.
"""
from dataclasses import dataclass
import numpy as np
from rigid_math import skew, physical_inertia

# Engineering scales: 2 mm COM, 0.05 kg m^2 inertia; fixed before replay.
SCALE = np.array([.002, .002, .05, .05, .05])


@dataclass
class Window:
    a: np.ndarray
    b: np.ndarray
    duration: float
    start: float
    end: float


def integral_window(t, rotations, omega, force, torque_o, rz, offdiag,
                    max_age=0., max_gap=.03, nuisance_world=None):
    """Short non-overlapping windows. Rows have units N m s.

    nuisance_world is a *known* angular-momentum correction time series;
    loaded truth must never be used here in live estimation.
    """
    t, r, w, f, tau = map(np.asarray, (t, rotations, omega, force, torque_o))
    if len(t) < 5 or not .25 <= t[-1]-t[0] <= .55:
        raise ValueError('window_duration')
    if np.any(np.diff(t) <= 0) or np.max(np.diff(t)) > max_gap or not 0 <= max_age <= .025:
        raise ValueError('timestamp_or_delay')
    if not all(np.all(np.isfinite(x)) for x in (t,r,w,f,tau,offdiag)):
        raise ValueError('nonfinite')
    if not np.allclose(np.diag(offdiag), 0):
        raise ValueError('offdiag_contains_diagonal')
    # ∫ R τO = Δ(R Jc ω) + ∫ R(r × F). No gravity term at COM.
    lever = np.trapz(np.array([-rr@skew(ff) for rr, ff in zip(r, f)]), t, axis=0)
    columns = [lever[:, 0], lever[:, 1]]
    for i in range(3):
        ei = np.eye(3)[i]
        columns.append(r[-1]@ei*w[-1, i] - r[0]@ei*w[0, i])
    b = np.trapz(np.einsum('nij,nj->ni', r, tau), t, axis=0)
    b -= lever[:, 2]*rz + r[-1]@offdiag@w[-1] - r[0]@offdiag@w[0]
    if nuisance_world is not None:
        b -= nuisance_world[-1]-nuisance_world[0]
    return Window(np.column_stack(columns), b, float(t[-1]-t[0]), float(t[0]), float(t[-1]))


class Estimator:
    def __init__(self, prior, offdiag, sigma_impulse=.004, forgetting=.995, torque_bias_bound_nm=None):
        self.theta = np.array(prior, float)
        self.offdiag = np.array(offdiag, float)
        self.sigma = sigma_impulse
        self.forgetting = forgetting
        # Caller must supply an independently justified bound, never infer it
        # from this same COM fit. Unknown constant torque and COM are confounded.
        self.torque_bias_bound_nm = torque_bias_bound_nm
        self.rows = []
        self.train = []
        self.holdout = []

    def add(self, window, holdout=False):
        dest = self.holdout if holdout else self.train
        if self.train or self.holdout:
            previous = max(x.end for x in self.train+self.holdout)
            if window.start < previous-1e-8:
                raise ValueError('overlap_or_time_reversal')
        dest.append(window)

    def information(self, windows=None):
        windows = self.train if windows is None else windows
        if not windows:
            return dict(rank=0, singular_values=[0.]*5, projector=np.zeros((5,5)),
                        condition=None, a=np.empty((0,5)), b=np.empty(0), vh=np.eye(5), keep=np.zeros(5,bool))
        weights = self.forgetting**(np.arange(len(windows)-1,-1,-1)/2)
        a = np.vstack([x.a*w for x,w in zip(windows, weights)])*SCALE/self.sigma
        b = np.concatenate([x.b*w for x,w in zip(windows, weights)])/self.sigma
        _, s, vh = np.linalg.svd(a, full_matrices=False)
        # Absolute SNR threshold AND relative condition threshold. A prior adds
        # no information and cannot rescue a missing singular direction.
        keep = (s >= 5.) & (s >= s[0]/100.)
        v = vh[keep].T
        condition = float(s[0]/s[keep][-1]) if np.any(keep) else None
        return dict(rank=int(sum(keep)), singular_values=s.tolist(), projector=v@v.T,
                    condition=condition, a=a, b=b, vh=vh, keep=keep)

    def propose(self):
        info = self.information()
        out = {k:v for k,v in info.items() if k not in ('a','b','vh','keep','projector')}
        out['projector'] = info['projector'].tolist()
        out['accepted'] = False
        out['theta'] = self.theta.tolist()
        if len(self.train) < 12 or len(self.holdout) < 5:
            return dict(out, reason='independent_sample_count')
        if self.torque_bias_bound_nm is None or not 0<=self.torque_bias_bound_nm<=.01:
            return dict(out, reason='unverified_constant_torque_bias')
        if not info['rank']:
            return dict(out, reason='no_supported_direction')
        a,b,vh,keep = [info[k] for k in ('a','b','vh','keep')]
        v = vh[keep].T
        # Finite weak prior regularization within data-supported space only.
        av = a@v
        delta = v @ np.linalg.solve(av.T@av+np.eye(v.shape[1])*.01, av.T@(b-a@(self.theta/SCALE)))
        candidate = self.theta + SCALE*delta
        if not self.physical(candidate):
            return dict(out, reason='physical_constraints')
        held = np.concatenate([x.a@candidate-x.b for x in self.holdout])
        hrms = float(np.sqrt(np.mean(held**2)))
        out['holdout_rms_Nms'] = hrms
        # Holdout predictive residual includes bias/model mismatch, not just fit.
        if hrms > 2*self.sigma:
            return dict(out, reason='independent_residual')
        # Independently fit early and late disjoint halves in same supported
        # directions; reject changing parameter fits even if pooled fit is good.
        halves = [self.train[:len(self.train)//2],self.train[len(self.train)//2:]]
        fits=[]
        for half in halves:
            ah=np.vstack([x.a for x in half])*SCALE/self.sigma
            bh=np.concatenate([x.b for x in half])/self.sigma
            sv=np.linalg.svd(ah@v,compute_uv=False)
            if sv[-1]<3.:
                return dict(out, reason='split_information_insufficient')
            fits.append(np.linalg.lstsq(ah@v,bh-ah@(self.theta/SCALE),rcond=None)[0])
        split=float(np.linalg.norm(fits[0]-fits[1]))
        out['split_delta_normalized']=split
        out['supported_95ci_normalized']=(1.96/np.array(info['singular_values'])[keep]).tolist()
        out['unsupported_directions'] = int(5-info['rank'])
        if split>.5:
            return dict(out, reason='estimate_unstable')
        return dict(out, accepted=True, reason='supported_directions_only', theta=candidate.tolist())

    def physical(self, theta):
        return (np.all(np.isfinite(theta)) and np.all(np.abs(theta[:2])<.3)
                and np.all((theta[2:]>.005)&(theta[2:]<1.))
                and physical_inertia(np.diag(theta[2:])+self.offdiag))
