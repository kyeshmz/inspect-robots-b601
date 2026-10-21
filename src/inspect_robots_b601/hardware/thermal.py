"""Predicted motor temperature and the torque budget that keeps it under a limit;
no I/O or clock.

Vendored from the native B601 follower so this repo's motor logs carry the same per-joint
thermal object (measured_c, estimated temp_c, torque_nm, allowed_torque_nm, latched, feedback):
froghollow_remote/arm/thermal.py
sha256 76eb141da1c87f7daa8f98bbf53ef6e459305b0839944c9116f48fe928ca3c29, lines 16-295 and 338-348.
Only the imports differ
(`finite` is local; the JOINTS-keyed test offset and report are not vendored),
and the code carries type annotations, docstrings and lint rewrites; the arithmetic is unchanged.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, replace
from functools import cached_property
from typing import Any, TypeGuard, cast

# (phi, gamma) of one exact thermal step:
# (winding, housing)' = phi (winding, housing) + gamma * heat.
Step = tuple[tuple[tuple[float, float], tuple[float, float]], tuple[float, float]]

__all__ = [
    "MOTOR_THERMALS",
    "THERMAL_SPECS",
    "JointThermal",
    "MotorThermal",
    "ThermalGuard",
    "ThermalSpec",
    "temperature_level",
]


def finite(value: object) -> TypeGuard[float]:
    return type(value) in (int, float) and math.isfinite(cast(float, value))


@dataclass(frozen=True)
class ThermalSpec:
    """Two thermal nodes, winding and housing, in rises above ambient
    with the total resistance normalised to 1.

    Heat is expressed as the winding rise it would settle at,
    so rated stall torque settles at rated_rise_k.
    """

    rated_stall_torque_nm: float
    peak_torque_nm: float
    hold_floor_nm: float
    torque_constant_nm_a: float
    rated_rise_k: float
    winding_share: float
    tau_winding_s: float
    tau_housing_s: float
    ambient_c: float
    limit_c: float
    trip_c: float
    resume_c: float
    horizons_s: tuple[float, ...] = (2.0, 10.0, 60.0)

    def heat(self, torque_nm: float) -> float:
        """The heat of a steady torque_nm, as the winding rise it would settle at."""
        return self.rated_rise_k * (torque_nm / self.rated_stall_torque_nm) ** 2

    def torque_for(self, heat: float) -> float:
        """The torque whose heat is ``heat``, the inverse of heat(); negative heat gives 0."""
        return self.rated_stall_torque_nm * math.sqrt(max(0.0, heat) / self.rated_rise_k)

    @property
    def continuous_torque_nm(self) -> float:
        """The torque whose winding settles exactly at limit_c from ambient_c."""
        return self.torque_for(self.limit_c - self.ambient_c)

    def step(self, seconds: float) -> Step:
        """Exact (phi, gamma) for constant heat over `seconds`:
        (w, h)' = phi (w, h) + gamma * heat.
        """
        f = self.winding_share
        a, b = -1 / self.tau_winding_s, 1 / self.tau_winding_s
        c, d = (1 - f) / (f * self.tau_housing_s), -1 / (f * self.tau_housing_s)
        mean = (a + d) / 2
        spread = math.sqrt(mean * mean - (a * d - b * c))
        fast, slow = mean - spread, mean + spread
        e_fast, e_slow = math.exp(fast * seconds), math.exp(slow * seconds)
        # A 2x2 exponential is k0 I + k1 A,
        # with k0 + k1 * lambda = exp(lambda t) at both eigenvalues.
        k1 = (e_slow - e_fast) / (slow - fast)
        k0 = e_fast - k1 * fast
        phi = ((k0 + k1 * a, k1 * b), (k1 * c, k0 + k1 * d))
        # Unit heat settles at (1, 1 - f), so gamma = (I - phi) times that steady state.
        gamma = (1 - phi[0][0] - phi[0][1] * (1 - f), (1 - phi[1][1]) * (1 - f) - phi[1][0])
        return phi, gamma

    @cached_property
    def horizon_steps(self) -> tuple[Step, ...]:
        """step() over each of horizons_s, shortest first."""
        return tuple(self.step(seconds) for seconds in sorted(self.horizons_s))


# The manual fits. rs-06: rated stall 8 Nm, peak 36 Nm and the 1.09 Nm/A output torque constant
# are the RS06 manual's; rated_rise_k reads its 8 Nm continuous rating as settling at the 145 C
# fault from 25 C; winding_share and both time constants are fitted to its locked-rotor overload
# table by the follower's fit_thermal_model.py. rs-00 is inferred, not fitted: rated stall is its
# 5 Nm rotating rating / sqrt(2), peak is its factory 0x700B, and the thermal constants are
# borrowed from rs-06; only its 1.48 Nm/A (RMS) output torque constant is the RS00 manual's.
# The envelope sits below the motor's own 135 C warning and 145 C fault
# to leave margin for model error.
# hold_floor_nm is the least torque the guard ever leaves a motor: measured holds on this arm
# were at most 0.4 Nm on the shoulder at -30 deg and the gripper's own hold clamp is 1.0 Nm
# (2026-10-19), so 3.0 Nm on rs-06 and 1.0 Nm on rs-00 keep the pose with margin,
# and the heat of holding the floor settles far below limit_c.
_ENVELOPE: dict[str, Any] = {
    "rated_rise_k": 120.0,
    "winding_share": 0.20,
    "tau_winding_s": 4.5,
    "tau_housing_s": 300.0,
    "ambient_c": 35.0,
    "limit_c": 115.0,
    "trip_c": 125.0,
    "resume_c": 105.0,
}
THERMAL_SPECS = {
    "rs-06": ThermalSpec(
        rated_stall_torque_nm=8.0,
        peak_torque_nm=36.0,
        hold_floor_nm=3.0,
        torque_constant_nm_a=1.09,
        **_ENVELOPE,
    ),
    "rs-00": ThermalSpec(
        rated_stall_torque_nm=3.5,
        peak_torque_nm=14.0,
        hold_floor_nm=1.0,
        torque_constant_nm_a=1.48,
        **_ENVELOPE,
    ),
}


# Arm fits: the follower's fit_thermal_logs.py on one B601-RS's logged holds (2026-10-19),
# one row per motor model in the order it prints them: heat gain (K of steady winding rise per
# Nm^2), winding share, tau_w (s), tau_h (s). rs-06 is shoulder_lift over two press runs with
# ambient at its 34 C idle temperature. rs-00 is the gripper over its 2 and 3 Nm squeezes,
# a 40-minute 1.5 Nm hold and two long cool-downs, ambient 29 C. Everything else in an arm spec,
# the 35 C ambient included, is the manual spec's;
# 35 C is at or above every idle temperature measured on the arm.
_ARM_FITS = {
    "rs-06": (1.506, 0.144, 38.5, 1368.0),
    "rs-00": (10.400, 0.237, 52.0, 1578.0),
}


def _arm_spec(
    manual: ThermalSpec,
    gain_k_per_nm2: float,
    winding_share: float,
    tau_winding_s: float,
    tau_housing_s: float,
) -> ThermalSpec:
    # The fit's gain is the steady winding rise per Nm^2, rated_rise_k / rated_stall_torque_nm^2.
    return replace(
        manual,
        rated_stall_torque_nm=math.sqrt(manual.rated_rise_k / gain_k_per_nm2),
        winding_share=winding_share,
        tau_winding_s=tau_winding_s,
        tau_housing_s=tau_housing_s,
    )


@dataclass(frozen=True)
class MotorThermal:
    """A motor model's two fits. The manual one alone describes sub-second bursts;
    the arm one, long holds.

    Peak, floor, torque constant, ambient and envelope are the manual spec's in both.
    """

    manual: ThermalSpec
    arm: ThermalSpec

    @property
    def specs(self) -> dict[str, ThermalSpec]:
        """Both fits, keyed "manual" and "arm"."""
        return {"manual": self.manual, "arm": self.arm}

    @property
    def peak_torque_nm(self) -> float:
        """The manual spec's peak torque."""
        return self.manual.peak_torque_nm

    @property
    def hold_floor_nm(self) -> float:
        """The manual spec's holding floor."""
        return self.manual.hold_floor_nm

    @property
    def torque_constant_nm_a(self) -> float:
        """The manual spec's output torque constant."""
        return self.manual.torque_constant_nm_a

    @property
    def continuous_torque_nm(self) -> float:
        """The lower of the two fits' continuous torques."""
        return min(spec.continuous_torque_nm for spec in self.specs.values())


MOTOR_THERMALS = {
    model: MotorThermal(manual, _arm_spec(manual, *_ARM_FITS[model]))
    for model, manual in THERMAL_SPECS.items()
}


class _Estimate:
    """One spec's winding and housing rises above its ambient,
    with its own latch and torque budget.
    """

    def __init__(self, spec: ThermalSpec) -> None:
        self.spec = spec
        self.winding_k = self.housing_k = 0.0
        self.latched = False

    @property
    def temp_c(self) -> float:
        return self.spec.ambient_c + self.winding_k

    @property
    def housing_c(self) -> float:
        return self.spec.ambient_c + self.housing_k

    def update(self, torque_nm: float, measured_c: float | None, dt: float | None) -> None:
        """Run dt seconds at torque_nm, or seed from measured_c when dt is None,
        then fuse measured_c.
        """
        spec = self.spec
        reading = None if measured_c is None else measured_c - spec.ambient_c
        if dt is None:
            self.winding_k = self.housing_k = 0.0 if reading is None else max(0.0, reading)
        else:
            phi, gamma = spec.step(dt)
            heat = spec.heat(torque_nm)
            self.winding_k, self.housing_k = (
                phi[0][0] * self.winding_k + phi[0][1] * self.housing_k + gamma[0] * heat,
                phi[1][0] * self.winding_k + phi[1][1] * self.housing_k + gamma[1] * heat,
            )
            if reading is not None:
                self._fuse(reading, dt)
        hottest = self.temp_c if measured_c is None else max(self.temp_c, measured_c)
        # Below the floor the guard can no longer trade torque for cooling,
        # so it holds at the floor and latches.
        starved = self.budget_nm() < spec.hold_floor_nm
        if hottest >= spec.trip_c or starved:
            self.latched = True
        elif hottest < spec.resume_c:
            self.latched = False

    def _fuse(self, reading: float, dt: float) -> None:
        # Readings are whole degrees, rounded or truncated,
        # so only a gap outside [m - 0.5, m + 1] is news.
        if self.winding_k < reading - 0.5:
            # A hotter winding than modelled means more heat went in,
            # and at steady state the housing carries 1 - f of the winding rise.
            self.housing_k += (reading - self.winding_k) * (1 - self.spec.winding_share)
            self.winding_k = reading
        elif self.winding_k > reading + 1.0:
            # 10% of the gap per second, whatever the update rate, because the sensor may lag.
            self.winding_k -= (1 - 0.9**dt) * (self.winding_k - (reading + 1.0))

    def budget_nm(self) -> float:
        spec = self.spec
        headroom = spec.limit_c - spec.ambient_c
        heat = min(
            (headroom - phi[0][0] * self.winding_k - phi[0][1] * self.housing_k) / gamma[0]
            for phi, gamma in spec.horizon_steps
        )
        return min(spec.peak_torque_nm, spec.torque_for(heat))

    def cold_nm(self) -> float:
        """The budget of a motor at ambient: the standing allowance heat is not yet limiting."""
        spec = self.spec
        return spec.torque_for(
            min((spec.limit_c - spec.ambient_c) / gamma[0] for _, gamma in spec.horizon_steps)
        )

    def predicted_c(self, torque_nm: float) -> float:
        phi, gamma = self.spec.horizon_steps[-1]
        return (
            self.spec.ambient_c
            + phi[0][0] * self.winding_k
            + phi[0][1] * self.housing_k
            + gamma[0] * self.spec.heat(abs(torque_nm))
        )


class JointThermal:
    """A joint's estimate under each of its motor's fits,
    acting on whichever is stricter at every step.
    """

    def __init__(self, motor: MotorThermal) -> None:
        self.motor = motor
        self.estimates = {name: _Estimate(spec) for name, spec in motor.specs.items()}
        self.last_s: float | None = None
        self.measured_c: float | None = None
        self.torque_nm = 0.0
        self.feedback = "missing"
        self._derating = False

    @property
    def temp_c(self) -> float:
        """The hottest estimated winding temperature across the fits."""
        return max(estimate.temp_c for estimate in self.estimates.values())

    @property
    def housing_c(self) -> float:
        """The hottest estimated housing temperature across the fits."""
        return max(estimate.housing_c for estimate in self.estimates.values())

    @property
    def latched(self) -> bool:
        """Whether any fit has latched."""
        return any(estimate.latched for estimate in self.estimates.values())

    def update(self, torque_nm: object, measured_c: object, now_s: float) -> None:
        """Step every fit to now_s at the previous torque, fuse measured_c, then keep torque_nm.

        The first update seeds the fits from measured_c instead; non-finite values count as missing.
        """
        self.feedback = "ok" if finite(torque_nm) else "missing"
        self.measured_c = measured_c if finite(measured_c) else None
        dt = None if self.last_s is None else min(1.0, max(0.0, now_s - self.last_s))
        for estimate in self.estimates.values():
            # Zero-order hold: the elapsed interval ran at the torque reported last time.
            estimate.update(self.torque_nm, self.measured_c, dt)
        if finite(torque_nm):
            self.torque_nm = abs(torque_nm)
        self.last_s = now_s

    def allowed_torque_nm(self) -> float:
        """The torque the guard leaves the motor:
        the stricter thermal budget, but never below the holding floor.
        """
        return max(
            self.motor.hold_floor_nm,
            min(estimate.budget_nm() for estimate in self.estimates.values()),
        )

    def derating(self, ceiling_nm: float) -> bool:
        """Whether heat, not the guard's standing cold allowance or the ceiling,
        now limits this motor.

        On once the allowance is more than 15% below that reference,
        off only once it is back within 10%, so the operator flag does not flicker at the threshold.
        A motor resting at 50 C is already 8% short of its cold allowance on either model,
        so the 10% release lets a rested warm joint clear.
        Repeating a call without an update changes nothing.
        """
        cold = min(estimate.cold_nm() for estimate in self.estimates.values())
        reference = min(ceiling_nm, self.motor.peak_torque_nm, cold)
        shortfall = 1 - self.allowed_torque_nm() / reference
        self._derating = shortfall > (0.10 if self._derating else 0.15)
        return self._derating

    def predicted_c(self, torque_nm: float) -> float:
        """The hottest fit's winding temperature after the longest horizon at a steady torque_nm."""
        return max(estimate.predicted_c(torque_nm) for estimate in self.estimates.values())

    def snapshot(self) -> dict[str, Any]:
        """The joint's rounded thermal state for the motor log, each fit's own under "models"."""
        models = {
            name: {
                "temp_c": round(estimate.temp_c, 2),
                "housing_c": round(estimate.housing_c, 2),
                "budget_nm": round(estimate.budget_nm(), 3),
                "predicted_c": round(estimate.predicted_c(self.torque_nm), 2),
                "latched": estimate.latched,
            }
            for name, estimate in self.estimates.items()
        }
        return {
            "temp_c": round(self.temp_c, 2),
            "housing_c": round(self.housing_c, 2),
            "measured_c": self.measured_c,
            "torque_nm": round(self.torque_nm, 3),
            "allowed_torque_nm": round(self.allowed_torque_nm(), 3),
            "predicted_c": round(self.predicted_c(self.torque_nm), 2),
            "latched": self.latched,
            "feedback": self.feedback,
            "models": models,
        }


class ThermalGuard:
    """One JointThermal per joint, keyed by joint name, from a joint-to-motor-model mapping."""

    def __init__(self, models: Mapping[str, str]) -> None:
        self.models = models
        self.reset()

    def reset(self) -> None:
        """Fresh per-joint state, so the first reading after a (re)connect seeds both nodes."""
        self.joints: dict[str, JointThermal] = {
            joint: JointThermal(MOTOR_THERMALS[model]) for joint, model in self.models.items()
        }

    def update(self, states: Mapping[str, tuple[object, object]], now_s: float) -> None:
        """Update each joint from its (torque, measured temperature); an absent joint reads none."""
        for joint, thermal in self.joints.items():
            thermal.update(*states.get(joint, (None, None)), now_s)

    def allowed(self) -> dict[str, float]:
        """Each joint's allowed torque in Nm."""
        return {joint: thermal.allowed_torque_nm() for joint, thermal in self.joints.items()}

    @property
    def latched(self) -> bool:
        """Whether any joint has latched."""
        return any(thermal.latched for thermal in self.joints.values())

    def snapshot(self) -> dict[str, dict[str, Any]]:
        """Each joint's snapshot, keyed by joint name."""
        return {joint: thermal.snapshot() for joint, thermal in self.joints.items()}


# The operator-facing wire contract, shared with Terra:
# critical and hot equal the guard's trip and resume.
TEMPERATURE_LEVELS = ((125, "critical"), (105, "hot"), (80, "warm"))
_SEVERITY = ("unknown", "ok", "warm", "hot", "critical")


def temperature_level(temp_c: float | None, latched: bool = False) -> str:
    """The operator-facing level for temp_c: critical when latched, unknown without a reading."""
    if latched:
        return "critical"
    if temp_c is None:
        return "unknown"
    return next((level for threshold, level in TEMPERATURE_LEVELS if temp_c >= threshold), "ok")
