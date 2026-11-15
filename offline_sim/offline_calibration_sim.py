"""
Offline replay of the ILC calibration loop in warning_block_Jianshe.py.

Each row of a run's record_data.csv is treated as one 1-second tick of the live loop:
  1. the row is appended to calibrate_data if MPC_warning > 0.01 and warning_1 > 0.01
     (same gate as the live code) AND, new here, spd_ego >= MIN_SPEED
  2. every CALIBRATION_INTERVAL ticks, mainIDM() is run on the accumulated calibrate_data
     (loaded with the same load_ilc_data logic) and the result is blended 50/50
  3. one ilc_params_log row is written per tick (same columns as the live file)

calibrate_data starts from calibrate_data_reference.csv, exactly as in the live run.

Usage:
  python offline_calibration_sim.py driver3_20260814 [driver7_20260804 ...] [--min-speed 1.0] [--fix-acc] [--tag name]
Output: results/<tag>/<run>/{ilc_params_log.csv, calibration_events.csv, calibrate_data.csv}
"""
import argparse
import contextlib
import glob
import io
import os
import re
import sys
import time

import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

with contextlib.redirect_stdout(io.StringIO()):
    from calibrate_gradient_adaptive_step import mainIDM, smooth_signals
    from ILC_constants import IDM_Param

RUN_DATA = os.path.join(ROOT, "run_data")
CALIBRATE_SEED = os.path.join(ROOT, "calibrate_data_reference.csv")
CALIBRATION_INTERVAL = 10  # ticks (rows are 1 s apart)

# initial ILC parameters, same as warning_block_Jianshe.py
A0, D0, C0, T0 = 1 * 1.2, 0.1, 0.9, 0.863
V0 = 24.5
DELTA = IDM_Param().delta


def load_ilc_data(df):
    """Same as load_ilc_data() in warning_block_Jianshe.py, but takes a DataFrame."""
    pos_ego_list = df['pos_ego'].tolist()
    spd_ego_list = df['spd_ego'].tolist()
    warning_list = df['warning_2'].tolist()

    def extract_first_tl(tl_str):
        numbers = re.findall(r'[\d.]+', str(tl_str))
        return float(numbers[0]) if numbers else 1.0
    tls_list = df['predicted_tl_state'].apply(extract_first_tl).tolist()

    acc_list = [0.0]
    for i in range(1, len(spd_ego_list)):
        acc_list.append(spd_ego_list[i] - spd_ego_list[i - 1])

    spd_ego_list, acc_list = smooth_signals(spd_ego_list, acc_list, window=5)
    return warning_list, acc_list, spd_ego_list, pos_ego_list, tls_list


def load_ilc_data_accfix(df, max_gap=1.5):
    """Like load_ilc_data, but acc is only computed between rows that are consecutive in time
    (gap <= max_gap s). calibrate_data only holds rows with an active warning, so neighbouring
    rows can belong to different approaches (or to the reference seed); differencing across
    those gaps creates fake accelerations of +-10..20 m/s^2. The first row of each contiguous
    segment has no valid acc and is dropped; smoothing is done within each segment."""
    df = df.reset_index(drop=True)
    dt = pd.to_datetime(df['timestamp'], format='mixed').diff().dt.total_seconds()
    valid = (dt > 0) & (dt <= max_gap)
    seg = (~valid).cumsum()
    acc = df['spd_ego'].diff().where(valid)
    smooth = lambda s: s.rolling(5, center=True, min_periods=1).mean()
    acc_s = acc.groupby(seg).transform(smooth)
    spd_s = df['spd_ego'].groupby(seg).transform(smooth)

    def extract_first_tl(tl_str):
        numbers = re.findall(r'-?[\d.]+', str(tl_str))
        return float(numbers[0]) if numbers else 1.0
    tls = df['predicted_tl_state'].apply(extract_first_tl)

    m = valid.values
    return (df['warning_2'][m].tolist(), acc_s[m].tolist(), spd_s[m].tolist(),
            df['pos_ego'][m].tolist(), tls[m].tolist())


def find_record_file(run_dir):
    files = glob.glob(os.path.join(run_dir, "*record_data.csv"))
    if not files:
        raise FileNotFoundError(f"no record_data.csv in {run_dir}")
    return files[0]


def simulate(run_name, min_speed, out_dir, fix_acc=False):
    run_dir = os.path.join(RUN_DATA, run_name)
    rec = pd.read_csv(find_record_file(run_dir))
    seed = pd.read_csv(CALIBRATE_SEED)

    calib_rows = [r for _, r in seed.iterrows()]
    a_cal, d_cal, c_cal, T_cal, J_final = A0, D0, C0, T0, 10000
    params_log, events = [], []
    n_skipped_slow = 0
    t_start = time.time()

    for tick, row in rec.iterrows():
        if row['MPC_warning'] > 0.01 and row['warning_1'] > 0.01:
            if row['spd_ego'] >= min_speed:
                calib_rows.append(row)
            else:
                n_skipped_slow += 1

        if (tick + 1) % CALIBRATION_INTERVAL == 0:
            cal_df = pd.DataFrame(calib_rows)
            loader = load_ilc_data_accfix if fix_acc else load_ilc_data
            warning_ilc, acc_ilc, spd_ilc, spacing_ilc, tls_ilc = loader(cal_df)
            if len(warning_ilc) > 5:
                with contextlib.redirect_stdout(io.StringIO()):
                    a1, d1, c1, T1, J_final = mainIDM(V0, DELTA, warning_ilc, acc_ilc,
                                                      spd_ilc, spacing_ilc, tls_ilc)
                a_cal = a1 * 0.5 + a_cal * 0.5
                d_cal = d_cal * 0.5 + d1 * 0.5
                c_cal = c_cal * 0.5 + c1 * 0.5
                T_cal = T_cal * 0.5 + T1 * 0.5
                events.append({"timestamp": row['timestamp'], "tick": tick, "n_samples": len(warning_ilc),
                               "a_raw": a1, "d_raw": d1, "c_raw": c1, "T_raw": T1, "J_final": J_final,
                               "a_cal": a_cal, "d_cal": d_cal, "c_cal": c_cal, "T_cal": T_cal})

        params_log.append({"timestamp": row['timestamp'], "a_cal": a_cal, "d_cal": d_cal,
                           "c_cal": c_cal, "T_cal": T_cal, "J_final": J_final})

    os.makedirs(out_dir, exist_ok=True)
    pd.DataFrame(params_log).to_csv(os.path.join(out_dir, "ilc_params_log.csv"), index=False)
    pd.DataFrame(events).to_csv(os.path.join(out_dir, "calibration_events.csv"), index=False)
    pd.DataFrame(calib_rows).to_csv(os.path.join(out_dir, "calibrate_data.csv"), index=False)
    print(f"{run_name}: {len(rec)} ticks, {len(calib_rows) - len(seed)} rows appended, "
          f"{n_skipped_slow} skipped (<{min_speed} m/s), {len(events)} calibrations, "
          f"final a={a_cal:.3f} d={d_cal:.3f} c={c_cal:.3f} T={T_cal:.3f}  ({time.time() - t_start:.0f}s)")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("runs", nargs="+")
    p.add_argument("--min-speed", type=float, default=1.0)
    p.add_argument("--fix-acc", action="store_true", help="compute acc only across consecutive 1-s rows")
    p.add_argument("--tag", default=None)
    args = p.parse_args()
    tag = args.tag or f"minspd{args.min_speed:g}" + ("_accfix" if args.fix_acc else "")
    for run in args.runs:
        simulate(run, args.min_speed, os.path.join(HERE, "results", tag, run), args.fix_acc)
