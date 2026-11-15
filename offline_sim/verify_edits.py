"""
Checks for the calibration fix in warning_block_Jianshe.py / derivation_fast.py.

1. replays 3 drivers through the *live* load_ilc_data (pulled out of warning_block_Jianshe.py)
   + mainIDM, writes results/live_code/<run>/ilc_params_log.csv
2. runs load_ilc_data + mainIDM on every run's final calibrate_data.csv (crash/timing check)
3. edge cases: seed only, empty file, timestamps without microseconds
"""
import ast
import contextlib
import glob
import io
import os
import sys
import time

import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
os.chdir(ROOT)

with contextlib.redirect_stdout(io.StringIO()):
    from calibrate_gradient_adaptive_step import mainIDM
    from ILC_constants import IDM_Param

# warning_block_Jianshe.py opens sockets and loops forever on import, so pull out just the function
src = open("warning_block_Jianshe.py", encoding="utf-8").read()
fn = next(n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == "load_ilc_data")
ns = {"pd": pd}
exec(ast.get_source_segment(src, fn), ns)
load_ilc_data = ns["load_ilc_data"]

V0, DELTA = 24.5, IDM_Param().delta
A0, D0, C0, T0 = 1.2, 0.1, 0.9, 0.863
SEED = pd.read_csv("calibrate_data_reference.csv")
TMP = os.path.join(HERE, "_tmp_calibrate.csv")


def calibrate(csv_path):
    w, acc, spd, sp, tls = load_ilc_data(csv_path)
    if len(w) <= 5:
        return None, len(w)
    with contextlib.redirect_stdout(io.StringIO()):
        return mainIDM(V0, DELTA, w, acc, spd, sp, tls), len(w)


def replay(run):
    rec = pd.read_csv(glob.glob(f"run_data/{run}/*record_data.csv")[0])
    SEED.to_csv(TMP, index=False)                       # same seeding as the live script
    a, d, c, T, J = A0, D0, C0, T0, 10000
    log, worst = [], 0.0
    for tick, row in rec.iterrows():
        if row['MPC_warning'] > 0.01 and row['warning_1'] > 0.01:
            pd.DataFrame([row]).to_csv(TMP, mode='a', header=False, index=False)
        if (tick + 1) % 10 == 0:
            t0 = time.time()
            res, n = calibrate(TMP)
            worst = max(worst, time.time() - t0)
            if res:
                a1, d1, c1, T1, J = res
                a, d, c, T = a1 * .5 + a * .5, d * .5 + d1 * .5, c * .5 + c1 * .5, T * .5 + T1 * .5
        log.append({"timestamp": row['timestamp'], "a_cal": a, "d_cal": d, "c_cal": c, "T_cal": T, "J_final": J})
    out = os.path.join(HERE, "results", "live_code", run)
    os.makedirs(out, exist_ok=True)
    pd.DataFrame(log).to_csv(os.path.join(out, "ilc_params_log.csv"), index=False)
    print(f"  {run}: final a={a:.3f} d={d:.3f} c={c:.3f} T={T:.3f} J={J:.3f}  slowest calibration {worst:.2f}s")


print("1) replay with live code")
for run in ["driver3_20260814", "driver7_20260804", "driver9_20260731"]:
    replay(run)

print("2) every run_data/*/calibrate_data.csv")
for f in sorted(glob.glob("run_data/*/calibrate_data.csv")):
    t0 = time.time()
    res, n = calibrate(f)
    at_lb = res is not None and res[0] <= 0.4 and res[1] <= 0.06 and res[2] <= 0.5 and res[3] <= 0.5
    print(f"  {os.path.basename(os.path.dirname(f)):26s} n={n:4d} {time.time() - t0:5.2f}s "
          + ("(not enough data)" if res is None else f"a={res[0]:.3f} d={res[1]:.3f} c={res[2]:.3f} T={res[3]:.3f}"
             + ("  <-- all at lower bounds" if at_lb else "")))

print("3) edge cases")
SEED.to_csv(TMP, index=False)
print("  seed only:", calibrate(TMP)[1], "rows ->", "ok")
SEED.iloc[:0].to_csv(TMP, index=False)
print("  header only:", calibrate(TMP)[1], "rows ->", "ok")
df = SEED.copy()
df['timestamp'] = pd.to_datetime(df['timestamp']).dt.floor('s').astype(str)
df.loc[::2, 'timestamp'] = pd.to_datetime(SEED['timestamp'][::2]).astype(str)  # mix with/without microseconds
df.to_csv(TMP, index=False)
print("  mixed timestamp formats:", calibrate(TMP)[1], "rows ->", "ok")
os.remove(TMP)
