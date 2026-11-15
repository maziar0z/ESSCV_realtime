"""ESSCV driver model for the road test: IDM inversion (MPC acceleration -> target speed shown to the driver)
and online calibration of the IDM to the driver.

Display (same rules as the simulation's MPCIDM maincontrol_plotter.py, no leader):
  IDM:  acc = a * [1 - (v/v0)^4 - (s*/s)^2],  s* = s0 + v*T
  * s (gap) is "no leader" (MAX_GAP*1.2) except on red/yellow before the stop bar, where it is the
    distance to the stop bar; within 80 m of the stop bar s* = MINGAP + 1.
  * target v0 = v * (1 - acc_mpc/a - (s*/s)^2)^(-1/4); if that has no solution or the car is at standstill,
    target = v + acc_mpc*dt (simulation's fallback).
  * target above the speed limit -> show the current speed (the simulation's "Driver_acc = 0" rule).
  * after the stop bar -> show the speed limit (the simulation accelerates back to free flow at 2 m/s^2).

Calibration (road test has no leader, so the simulation's spacing fit does not apply): fit a and T so that
the IDM, driven by the target speed that was *shown* and integrated over each 1 s tick, reproduces the
driver's measured speed at the next tick. Integrating matters: at 1 Hz a responsive driver closes much of the
gap to the shown speed within the tick, so fitting the IDM's instantaneous acceleration to the tick-average
acceleration underestimates a (by up to 2x in tests) and leaves T at its bound. Least squares with bounds
(scipy L-BFGS-B), on consecutive ticks (<= 1.5 s apart) within 500 m of the stop bar with v > 1 m/s, using
only ticks whose shown target came from the IDM inversion. s0 stays fixed: with no leader it only acts very
close to a red stop bar (where s* is fixed within 80 m), so the data cannot identify it.
"""
import numpy as np
import pandas as pd
from scipy.optimize import minimize

DELTA = 4
MAX_GAP = 80.0
MINGAP = 2.5
ACCEL_RATE = 2.0        # simulation accel_rate (Veh_Parameter().max_acc)
DISPLAY_DT = 0.5        # simulation dt used in the fallback target
NEAR_SIGNAL = 80.0      # m: within this distance of the stop bar s* = MINGAP + 1
CON_RANGE = 500.0       # V2I range (m)
RESTART_SPEED = 5.0     # m/s (~11 mph): below this, an accelerating MPC plan sets a floor on the target
RESTART_AHEAD = 3.0     # s: the floor is the MPC's planned speed this far ahead

# initial IDM parameters (simulation IDM_Param: a=1.61, s0=4, T=2.5 capped at 1.5 in the simulation)
INIT_PARAMS = {"a": 1.61, "T": 1.5, "s0": 4.0}
BOUNDS = {"a": (0.4, 4.5), "T": (0.5, 2.0)}   # same as RLRWS; s0 is not calibrated (fixed at 4 m)
CAL_SUBSTEPS = 10       # integration substeps per tick in the calibration
CAL_ACC_LIMITS = (-4.5, 3.0)


def gap_to_use(pos_ego, red_now):
    """Gap s for the IDM: no leader, except the stop bar on red/yellow (before the bar, within range)."""
    spacing = MAX_GAP * 1.2
    if red_now and 0 < pos_ego <= CON_RANGE:
        spacing = min(spacing, max(pos_ego, 1.0))
    return spacing


def target_speed(acc_mpc, spd, pos_ego, red_now, freev, params):
    """Speed (m/s) to show the driver so that an IDM driver produces acc_mpc."""
    if pos_ego < 0:   # passed the stop bar: back to free-flow speed
        return freev
    a, T, s0 = params["a"], params["T"], params["s0"]
    spacing = gap_to_use(pos_ego, red_now)
    s_star = s0 + max(0.0, spd * T)
    if s_star / spacing < 0.2:     # adjust a if spacing is high and a is low
        a = max(a, ACCEL_RATE)
    if abs(pos_ego) <= NEAR_SIGNAL:
        s_star = MINGAP + 1
    bracket = 1 - acc_mpc / a - (s_star / spacing) ** 2
    if bracket >= 0 and spd > 0.1:
        v0 = bracket ** (-1 / DELTA) * spd
    else:   # no v0 gives acc_mpc (or standstill): fallback
        v0 = spd + acc_mpc * DISPLAY_DT
    if v0 > freev:   # above the speed limit: hold the current speed
        v0 = spd
    return max(0.0, v0)


def passer(pos_ego, spd_ego, red_now):
    """RLRWS passer rule: close to the stop bar, slow, and red -> tell the driver to stop."""
    return 0 <= pos_ego <= 60 and spd_ego <= 4 and red_now


def display_target(acc_mpc, spd, pos_ego, red_now, freev, params, in_range, stop_speed=None, restart_speed=None):
    """Target speed (m/s) for the display, including the out-of-range and stop-at-red cases.
    stop_speed: the MPC's planned speed ~1 s ahead when it must stop for a red (else None); the display
    never shows more than that, because the IDM inversion alone cannot ask for a stop (no leader).
    restart_speed: the MPC's planned speed ~3 s ahead. When the car is slow and the MPC wants to accelerate,
    the display shows at least that, because the inversion (target proportional to the current speed) can
    only suggest a few mph more than the current speed when starting from (near) standstill.
    Returns (target, source); source says which rule set the value ("limit", "passer", "stop", "restart",
    "idm"). Only "idm" rows are used for calibration: the others are not IDM-inversion outputs."""
    if pos_ego < 0:
        return freev, "limit"           # passed the stop bar: back to free-flow speed
    if not in_range:   # no signal ahead within range / no SPaT: speed limit
        return freev, "limit"
    if passer(pos_ego, spd, red_now):
        return 0.0, "passer"
    target = target_speed(acc_mpc, spd, pos_ego, red_now, freev, params)
    if stop_speed is not None:   # must stop: never more than the stopping speed, no restart floor
        return (stop_speed, "stop") if stop_speed < target else (target, "idm")
    if restart_speed is not None and spd < RESTART_SPEED and acc_mpc > 0 and restart_speed > target:
        return min(restart_speed, freev), "restart"
    return target, "idm"


def idm_acc(params, spd, v0_shown, pos_ego, red_now):
    a, T, s0 = params
    spacing = np.array([gap_to_use(p, r) for p, r in zip(pos_ego, red_now)])
    s_star = np.where(np.abs(pos_ego) <= NEAR_SIGNAL, MINGAP + 1, s0 + np.maximum(0, spd * T))
    v0 = np.maximum(v0_shown, 0.1)
    return a * (1 - (spd / v0) ** DELTA - (s_star / spacing) ** 2)


def load_calibration_rows(csv_path):
    """Ticks usable for calibration from record_data:
    (speed, shown target, distance to bar, red, speed at the next tick, tick length)."""
    df = pd.read_csv(csv_path)
    need = {"timestamp", "spd_ego", "pos_ego", "target_speed_mps", "red_now"}
    if df.empty or not need.issubset(df.columns):
        return None
    idm_shown = df["display_source"].eq("idm") if "display_source" in df else pd.Series(True, index=df.index)
    try:
        t = pd.to_datetime(df["timestamp"], format="mixed", errors="coerce")
    except (TypeError, ValueError):
        t = pd.to_datetime(df["timestamp"], errors="coerce")
    dt = t.diff().dt.total_seconds()
    valid = (dt > 0) & (dt <= 1.5)   # consecutive ticks only (calibration rows are not contiguous otherwise)
    # the driver responds over (t-1, t] to what was shown at t-1, starting from the state at t-1
    prev = df.shift(1)
    m = (valid & (prev["pos_ego"] >= 0) & (prev["pos_ego"] <= CON_RANGE) & (prev["spd_ego"] > 1)
         & prev["target_speed_mps"].notna() & idm_shown.shift(1, fill_value=False)).values
    if m.sum() == 0:
        return None
    return (prev["spd_ego"].values[m].astype(float), prev["target_speed_mps"].values[m].astype(float),
            prev["pos_ego"].values[m].astype(float), prev["red_now"].values[m].astype(bool),
            df["spd_ego"].values[m].astype(float), dt.values[m].astype(float))


def integrate_ticks(params, spd, v0_shown, pos, red, dt, n=CAL_SUBSTEPS):
    """Speed at the end of each tick for an IDM driver who starts at (spd, pos) and is shown v0_shown."""
    v, p = spd.copy(), pos.copy()
    for _ in range(n):
        acc = np.clip(idm_acc(params, v, v0_shown, p, red), *CAL_ACC_LIMITS)
        v = np.maximum(0.0, v + acc * dt / n)
        p = p - v * dt / n
    return v


def calibrate(rows, params):
    """Least-squares fit of (a, T) to the measured speed at the next tick, integrating the IDM over each tick;
    s0 is kept as is. Returns (new params dict, J = mean squared speed error, (m/s)^2)."""
    spd, v0_shown, pos, red, v_next, dt = rows
    s0 = params["s0"]
    J = lambda x: float(np.mean((integrate_ticks([x[0], x[1], s0], spd, v0_shown, pos, red, dt) - v_next) ** 2))
    res = minimize(J, [params["a"], params["T"]], method="L-BFGS-B", bounds=[BOUNDS["a"], BOUNDS["T"]])
    return {"a": float(res.x[0]), "T": float(res.x[1]), "s0": s0}, float(res.fun)
