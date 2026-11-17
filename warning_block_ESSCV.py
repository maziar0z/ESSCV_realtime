"""ESSCV road-test block: replaces warning_block_Jianshe.py (RLRWS warning) with the ESSCV speed suggestion.

Same pipeline position and ports as warning_block_Jianshe.py:
  receives  PredData on 10889 (from traffic_prediction_block.py) and CavData on 10900 (forwarded OBU message)
  sends     the display message on 10890 (to plotting_block_ESSCV.py)
Every 1 s:
  1. ESSCV MPC (esscv_optimization.py, no leader) with the SPaT timeline from the OBU message
  2. MPC acceleration 1 s ahead -> target speed via the calibrated IDM (esscv_driver_model.py)
     (only with fresh SPaT from the current intersection, within 500 m; otherwise 'no advice', MPC skipped)
  3. send {target speed or None, current speed} for display, log the row
  4. every 10 s: recalibrate the IDM on this run's data and blend 50/50 (as RLRWS does)
A failure in one tick is logged and the loop continues with the next tick.
"""
import os
import math
import time
import socket
import datetime
import traceback
import pickle as pkl

import numpy as np
import pandas as pd

from cav_data import *
import esscv_optimization as eo
import esscv_driver_model as dm

TEST_NAME = "esscv_test"
CALIBRATION_INTERVAL = 10       # s
SPAT_MAX_AGE = 2.0              # s: older SPaT (stopped arriving / out of RSU range) is not used
CANT_MAKE_HYSTERESIS = 1.0      # m/s: "can't make the green" only flips when clearly on the other side
MIN_CALIBRATION_ROWS = 10
DEFAULT_FF_SPD = 24.5           # m/s, used only if the OBU message has no ff_spd (RLRWS v0)
MPS_TO_MPH = 2.23694

safe_test_name = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in TEST_NAME.strip()) or "unnamed_test"
Run_data_dir = os.path.join("run_data", f"{safe_test_name}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}")
os.makedirs(Run_data_dir, exist_ok=True)
Record_data = os.path.join(Run_data_dir, "record_data.csv")
IDM_params_file = os.path.join(Run_data_dir, "idm_params_log.csv")

HOST_REC = ""
PORT_REC = 10889
s_rec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_rec.bind((HOST_REC, PORT_REC))

HOST_REC1 = ""
PORT_REC1 = 10900
s_rec1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_rec1.bind((HOST_REC1, PORT_REC1))

HOST_SEND = 'localhost'
PORT_SEND = 10890
s_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

s_rec.setblocking(False)
current_time = datetime.datetime.now()
last_time = current_time
last_calibration_time = time.time()
newestData_msg = None

idm_params = dict(dm.INIT_PARAMS)
J_final = None
plan = eo.initial_guess()      # last MPC plan (accelerations, 0.5 s steps)
prev_acc = 0.0                 # car's acceleration over the last tick (jerk term)
spd_prev, prev_tick = None, None
cant_make_prev, cant_make_key = None, None   # hysteresis state for "can't make the current green"


def append_csv_row(csv_path, row):
    write_header = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
    pd.DataFrame([row]).to_csv(csv_path, mode='a', header=write_header, index=False)


def clean_sec_to_change(sec):
    """SPaT time to change in s, or None if unusable. minEndTime counts within the hour, so the value
    computed by obu_info_block jumps by -3600 s at the top of the hour; J2735 'unknown' (36001) gives
    huge values."""
    if sec is None:
        return None
    try:
        sec = float(sec)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(sec):
        return None
    if sec < -1800:          # hour wrap
        sec += 3600
    if sec > 1800:           # unknown / not a real time
        return None
    return max(0.0, sec)


def signal_info(msg_data):
    """Signal state from the OBU message, used only if it is fresh and from the right intersection.
    Returns (status, sec_to_change, red_now, fresh, age, spat_intersection_id, expected_signal_id).
    sec_to_change is the SPaT earliest change time (minEndTime), counted down by the SPaT's age.
    Stale or foreign SPaT (e.g. the last SPaT of the previous intersection carried over to an intersection
    that sends none, as at ref_3 on 2026-11-12) gives status None: no advice."""
    get = lambda name: getattr(msg_data, name, None) if msg_data is not None else None
    status, sec = get("signal_status"), clean_sec_to_change(get("sec_to_change"))
    spat_id, rx, expected = get("spat_intersection_id"), get("spat_rx_time"), get("expected_signal_id")
    age = time.time() - rx if rx is not None else None
    fresh = (status is not None and sec is not None and age is not None and age <= SPAT_MAX_AGE
             and (expected is None or spat_id == expected))
    if not fresh:
        return None, None, False, False, age, spat_id, expected
    sec = max(0.0, sec - max(age, 0.0))
    return status, sec, status in ("red", "yellow"), True, age, spat_id, expected


def run_tick(data_orig, msg_data):
    global plan, prev_acc, spd_prev, prev_tick, idm_params, J_final, last_calibration_time
    global cant_make_prev, cant_make_key
    predicted_tl_state = data_orig.predicted_tl_state
    pos_ego = float(data_orig.pos_ego)
    spd_ego = float(data_orig.spd_ego)
    ff_spd = getattr(msg_data, "ff_spd", None) or DEFAULT_FF_SPD
    status, sec_to_change, red_now, fresh, spat_age, spat_id, expected_id = signal_info(msg_data)
    # advice only when approaching a signal within range with fresh SPaT from that intersection
    # (placeholder distances 1000 / 1500 m from the OBU are outside the range)
    advice_ok = fresh and 0 <= pos_ego <= eo.D_SIG
    # the human drives, so the jerk term uses the car's measured acceleration over the last tick
    if spd_prev is not None and prev_tick is not None:
        prev_acc = float(np.clip((spd_ego - spd_prev) / max((current_time - prev_tick).total_seconds(), 0.5),
                                 eo.MAX_DEC, eo.MAX_ACC))
    spd_prev, prev_tick = spd_ego, current_time

    # "can't make the current green" with hysteresis (reset on a new approach or when not green)
    cant_make = None
    key = (getattr(data_orig, "current_intersection", None), getattr(data_orig, "current_approach", None))
    if advice_ok and status == "green":
        previous = cant_make_prev if key == cant_make_key else None
        cant_make = eo.cant_make_green(pos_ego, ff_spd, sec_to_change, previous, CANT_MAKE_HYSTERESIS)
    cant_make_prev, cant_make_key = cant_make, key

    # 1. MPC (skipped when there is nothing to advise; capped in time, previous plan kept if it can't finish)
    t0 = time.time()
    timeline = eo.SignalTimeline(status, sec_to_change, cant_make=cant_make) if advice_ok else None
    mpc_ok, mpc_msg, mpc_fallback = None, "skipped (no advice)", False
    if advice_ok:
        warm = eo.shifted_warm_start(plan)   # previous plan, shifted to now
        try:
            new_plan, sol = eo.solve_mpc(pos_ego, spd_ego, ff_spd, timeline, warm, prev_acc)
            if eo.plan_is_usable(new_plan):
                plan, mpc_ok, mpc_msg = new_plan, bool(sol.success), str(sol.message)
            else:
                plan, mpc_ok, mpc_msg, mpc_fallback = warm, False, "unusable plan: previous plan used", True
        except eo.SolveTimeout:
            plan, mpc_ok, mpc_msg, mpc_fallback = warm, False, f"over {eo.TIME_BUDGET} s: previous plan used", True
        except Exception:
            traceback.print_exc()
            plan, mpc_ok, mpc_msg, mpc_fallback = warm, False, "solver error: previous plan used", True
    else:
        plan = eo.initial_guess()
    mpc_time = time.time() - t0
    # suggestion 1 s ahead (reaction time), clipped like the simulation
    desired_acc = float(np.clip(plan[2], eo.MAX_DEC * 0.7, eo.MAX_ACC * 0.7))

    # 2. target speed (no advice -> None; 0 for the passer stop case; else IDM inversion,
    #    capped by the MPC's stopping plan before a red, floored by its plan when restarting)
    must_stop = advice_ok and eo.must_stop(pos_ego, spd_ego, ff_spd, timeline)
    stop_speed = eo.stop_speed(pos_ego, spd_ego, plan) if must_stop else None
    restart_speed = eo.planned_speed(spd_ego, plan, dm.RESTART_AHEAD) if advice_ok else None
    target, display_source = dm.display_target(desired_acc, spd_ego, pos_ego, red_now, ff_spd, idm_params,
                                               advice_ok, stop_speed, restart_speed)
    if target is not None and not math.isfinite(target):
        target, display_source = None, "none"

    # 3. display + log
    display = {"target_mph": None if target is None else target * MPS_TO_MPH, "speed_mph": spd_ego * MPS_TO_MPH,
               "advice": target is not None}
    s_send.sendto(pkl.dumps(display), (HOST_SEND, PORT_SEND))
    shown = "  --  " if target is None else f"{target*MPS_TO_MPH:5.1f} mph"
    print(f"pos {pos_ego:7.1f} m  spd {spd_ego*MPS_TO_MPH:5.1f} mph  {status} {sec_to_change}  "
          f"MPC acc {desired_acc:+.2f}  -> target {shown} [{display_source}]  "
          f"({mpc_time:.2f}s{', fallback' if mpc_fallback else ''})")
    target_mps = float("nan") if target is None else target
    row = {
        "timestamp": current_time, "pos_ego": pos_ego, "spd_ego": spd_ego,
        "current_ref_file": getattr(data_orig, "current_ref_file", None),
        "current_intersection": getattr(data_orig, "current_intersection", None),
        "current_approach": getattr(data_orig, "current_approach", None),
        "ff_spd": ff_spd, "signal_status": status, "sec_to_change": sec_to_change,
        "sec_to_change_raw": getattr(msg_data, "sec_to_change", None) if msg_data is not None else None,
        "signal_status_raw": getattr(msg_data, "signal_status", None) if msg_data is not None else None,
        "spat_fresh": fresh, "spat_age_s": None if spat_age is None else round(spat_age, 2),
        "spat_intersection_id": spat_id, "expected_signal_id": expected_id,
        "red_now": red_now, "tl_now": float(np.asarray(predicted_tl_state).reshape(-1)[0]),
        "cant_make": cant_make,
        "mpc_success": mpc_ok, "mpc_message": mpc_msg, "mpc_fallback": mpc_fallback,
        "mpc_time_s": round(mpc_time, 3),
        "mpc_acc_0": plan[0], "mpc_acc_1": plan[1], "mpc_acc_2": plan[2], "mpc_acc_3": plan[3],
        "desired_acc": desired_acc, "must_stop": must_stop, "stop_speed": stop_speed,
        "restart_speed": restart_speed,
        "target_speed_mps": target_mps, "target_speed_mph": target_mps * MPS_TO_MPH,
        "display_source": display_source,
        "idm_a": idm_params["a"], "idm_T": idm_params["T"], "idm_s0": idm_params["s0"],
    }
    append_csv_row(Record_data, row)

    # 4. recalibrate every 10 s, blend 50/50
    now = time.time()
    if now - last_calibration_time >= CALIBRATION_INTERVAL:
        last_calibration_time = now
        try:
            rows = dm.load_calibration_rows(Record_data)
            if rows is not None and len(rows[0]) >= MIN_CALIBRATION_ROWS:
                new, J_final = dm.calibrate(rows, idm_params)
                idm_params = {k: 0.5 * idm_params[k] + 0.5 * new[k] for k in idm_params}
                print(f"[IDM] calibrated on {len(rows[0])} rows: a={idm_params['a']:.3f} "
                      f"T={idm_params['T']:.3f} s0={idm_params['s0']:.2f} (J={J_final:.4f})")
            else:
                print("[IDM] Not enough data yet.")
        except Exception:
            print("[IDM] Calibration error:")
            traceback.print_exc()
        append_csv_row(IDM_params_file, {"timestamp": current_time, **idm_params, "J_final": J_final})


while True:
    current_time = datetime.datetime.now()
    if ((current_time - last_time).total_seconds()) >= 1:
        last_time = current_time
        newestData_predicted = None
        keepReceiving = True
        while keepReceiving:
            try:
                data_predicted, fromAddr = s_rec.recvfrom(500000)
                data_msg, fromAddr = s_rec1.recvfrom(500000)
                if data_predicted:
                    newestData_predicted = data_predicted
                if data_msg:
                    newestData_msg = data_msg
            except socket.error as why:
                keepReceiving = False
                break

        if newestData_predicted:
            try:
                msg_data = pkl.loads(newestData_msg) if newestData_msg else None
                run_tick(pkl.loads(newestData_predicted), msg_data)
            except Exception:
                print("[ESSCV] tick failed, continuing:")
                traceback.print_exc()
    else:
        time.sleep(0.01)
