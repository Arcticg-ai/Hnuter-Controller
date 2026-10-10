"""Offline-tested estimator core; no ROS/PX4 integration or mass truth access.

Caller must provide synchronized, independently reconstructed thrust and NED
velocity. This module cannot establish sensor provenance by itself.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Config:
    prior_kg: float = 4.5
    lower_kg: float = 3.5
    upper_kg: float = 7.0
    gravity: float = 9.8  # local default.sdf; HnuterControl uses 9.81
    window_s: float = .4
    max_gap_s: float = .045  # measured 20 ms DDS stream has 28–32 ms scheduling jitter
    forgetting: float = .98
    prior_weight: float = 1.
    min_windows: int = 20
    holdout_windows: int = 5
    stable_std_fraction: float = .015
    residual_fraction: float = .03


class MassEstimator:
    def __init__(self, cfg=Config()):
        if not (0 < cfg.lower_kg < cfg.prior_kg < cfg.upper_kg and 0 < cfg.forgetting <= 1
                and cfg.window_s > cfg.max_gap_s > 0 and cfg.prior_weight > 0
                and cfg.min_windows >= 8 and cfg.holdout_windows >= 1):
            raise ValueError('Invalid estimator configuration')
        self.cfg = cfg
        self.mass = cfg.prior_kg
        self.yy = self.yq = 0.
        self.history = []
        self.holdout = []
        self.rows = []
        self.candidate = None
        self.accepted = False
        self.reason = 'insufficient_windows'
        self.start = self.prev = None
        self.q = 0.

    def invalidate(self, reason):
        self.start = self.prev = None
        self.q = 0.
        # Independent holdout must be contiguous with the frozen candidate.
        self.candidate = None
        self.holdout.clear()
        self.accepted = False
        self.reason = reason
        self.yy = self.yq = 0.
        self.history.clear()
        self.mass = self.cfg.prior_kg

    def push(self, t, vz_ned, thrust_up, eligible=True):
        if not eligible or not all(math.isfinite(x) for x in (t, vz_ned, thrust_up)) or thrust_up <= 0:
            self.invalidate('invalid_or_ineligible_sample')
            return None
        sample = (t, vz_ned, thrust_up)
        if self.prev is None:
            self.start = self.prev = sample
            return None
        dt = t - self.prev[0]
        if not 0 < dt <= self.cfg.max_gap_s:
            self.invalidate('timestamp_gap_or_reversal')
            return None
        self.q += .5 * (self.prev[2] + thrust_up) * dt
        self.prev = sample
        duration = t - self.start[0]
        if duration + 1e-9 < self.cfg.window_s:
            return None
        y = self.cfg.gravity * duration - (vz_ned - self.start[1])
        q = self.q
        self.start = self.prev = sample  # shared boundary, disjoint integration intervals
        self.q = 0.
        if y <= .5*self.cfg.gravity*duration:
            self.invalidate('insufficient_positive_Y')
            return None
        raw_mass = q/y
        if not self.cfg.lower_kg < raw_mass < self.cfg.upper_kg:
            self.invalidate('outside_physical_bounds')
            return None
        mode = 'validation' if self.candidate is not None else 'training'
        if mode == 'training':
            self.yy = self.cfg.forgetting*self.yy + y*y
            self.yq = self.cfg.forgetting*self.yq + y*q
            self.mass = (self.yq+self.cfg.prior_weight*self.cfg.prior_kg)/(self.yy+self.cfg.prior_weight)
            self.history.append(self.mass)
            if len(self.history) >= self.cfg.min_windows and np.std(self.history[-8:])/self.mass < self.cfg.stable_std_fraction:
                self.candidate = self.mass
                self.reason = 'awaiting_independent_holdout'
        residual = q-self.mass*y
        if mode == 'validation' and not self.accepted:
            self.holdout.append(residual/max(abs(q), 1e-9))
            if len(self.holdout) >= self.cfg.holdout_windows:
                rms = float(np.sqrt(np.mean(np.square(self.holdout))))
                self.accepted = rms <= self.cfg.residual_fraction
                self.reason = 'accepted' if self.accepted else 'holdout_residual_rejected'
                if not self.accepted:
                    self.candidate = None
                    self.holdout.clear()
        row = dict(t=t, Q_Ns=q, Y_m_s=y, raw_mass_kg=raw_mass, mass_kg=self.mass,
                   mode=mode, residual_Ns=residual, accepted=self.accepted, reason=self.reason)
        self.rows.append(row)
        return row


def thrust_up_from_rotors(omega_rad_s, thrust_axes_body_frd, body_to_ned, coefficients):
    """Physical rotor rad/s and thrust-positive unit axes, NOT esc_rpm feedback.

    For command-only input a separately validated lag/servo observer is required.
    No such observer is claimed implemented here. Reverse signed speed is allowed.
    """
    omega, axes, rotation, k = map(np.asarray, (omega_rad_s, thrust_axes_body_frd, body_to_ned, coefficients))
    if axes.shape != (len(omega), 3) or k.shape != omega.shape or rotation.shape != (3, 3):
        raise ValueError('Invalid shapes')
    if not all(np.all(np.isfinite(a)) for a in (omega,axes,rotation,k)) or np.any(k <= 0):
        raise ValueError('Invalid rotor inputs')
    if not np.allclose(np.linalg.norm(axes, axis=1), 1.) or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(rotation),1):
        raise ValueError('Invalid rotation or axis')
    force_body = np.sum((k*omega*np.abs(omega))[:, None]*axes, axis=0)
    return -float((rotation@force_body)[2])


def vertical_reference(t, duration=16., amplitude=.08, frequency=.4):
    """NED z offset and analytic v/a; sin^4 envelope gives C2 zero endpoints."""
    if not 0 < t < duration:
        return np.zeros(3)
    b, w = math.pi/duration, 2*math.pi*frequency
    s, c = math.sin(b*t), math.cos(b*t)
    e, de, dde = s**4, 4*b*s**3*c, 4*b*b*(3*s*s*c*c-s**4)
    sw, cw = math.sin(w*t), math.cos(w*t)
    return amplitude*np.array([e*sw, de*sw+e*w*cw, dde*sw+2*de*w*cw-e*w*w*sw])


def bumpless_candidate(m_old, m_new, a_base, integral, xy_limit=2., z_limit=3.):
    """Algebra-only candidate, valid only before acceleration/force saturation.

    Firmware still needs state gates, rate limiting, and recomputation of all
    acceleration/force limits. Return None instead of clipping the integral.
    """
    a, old = np.asarray(a_base, dtype=float), np.asarray(integral, dtype=float)
    if min(m_old,m_new) <= 0 or not np.all(np.isfinite([m_old,m_new,*a,*old])):
        raise ValueError('Invalid mass or state')
    g = np.array([0.,0.,9.81])  # HnuterControl.cpp:149 in the audited checkout
    new = m_old/m_new*(a+old-g)-a+g
    if np.linalg.norm(new[:2]) > xy_limit or abs(new[2]) > z_limit:
        return None
    return new
