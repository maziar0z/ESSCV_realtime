# Session summary: ILC calibration fix (RLRWS red-light-running warning)

Handoff context for future sessions. Dates: 2026-10-06 to 2026-10-15.

## Paths and environment
- Repo (not a git repo): `D:\Research\RLRWS2\Task 4\RLRWS-Maziar_update-2\RLRWS-Maziar_update-2\` (called REPO below)
- Plot scripts and driver logs: `D:\Research\RLRWS2\Task 4\` (`plotter_3vectors.py`, `plotter_4vectors.py`, `logs\<driver>_record_data.csv`)
- Python: use `C:\Users\mazia\AppData\Local\Python\bin\python` (3.14, pandas 3.0.2, scipy, plotly, keyboard).
  Plain `python` can resolve to the Microsoft Store stub. miniconda has no pandas.
- Road-test data: `REPO\run_data\driverN_YYYYMMDD\` holds `<run>_record_data.csv`, `calibrate_data.csv` and
  `ilc_params_log.csv`. The user called these "test34_..."; the real folder names are `driverN_...`.
  The files in `Task 4\logs\` are identical copies of the record CSVs (md5 checked).

## System background
- `warning_block_Jianshe.py` is the live loop (1 Hz). Each tick:
  1. It receives a prediction over UDP port 10889 (from `traffic_prediction_block.py`) and a message over port 10900.
  2. It computes the MPC warning vector (`warning()`) and then
     `warning_showed = rev_warning_calc(v0, delta, warning_signal[2], ...)`, followed by post-filters
     (small-warning zeroing, 100 cap, far-away zeroing, near-bar zeroing, `passer` sets 35, cap at 1.5*MPC).
  3. It writes `record_data.csv`. It appends the row to `calibrate_data.csv` if `MPC_warning > 0.01 and warning_1 > 0.01`.
     calibrate_data is seeded from `calibrate_data_reference.csv`.
  4. Every 10 s (wall clock) it runs `mainIDM` (`calibrate_gradient_adaptive_step.py`) on calibrate_data, loaded with
     `load_ilc_data`, and blends the result 50/50 into (a_cal, d_cal, c_cal, T_cal). It logs to `ilc_params_log.csv`.
- Model: `v_dot = a*w - d*u^c`.
  - `w = 1 - (v/v0)^delta`, minus `((s0 + v*T)/spacing)^2` only on red within 500 m.
  - Settings: v0 = 24.5, delta = 2, s0 = 5.
  - Bounds: a [0.4, 4.5], d [0.06, 2], c [0.5, 2], T [0.5, 2].
  - Initial values (live): 1.2 / 0.1 / 0.9 / 0.863.

## Problem reported
For all 15 drivers, after about 10 min all four parameters sat exactly at their lower bounds (0.4 / 0.06 / 0.5 / 0.5).
The user suspected stopped-vehicle samples (speed < 1 m/s) in calibrate_data.

## Findings
1. **The stopped-vehicle hypothesis is false.** No run has a calibrate_data row with spd < 1 m/s; the minimum is about 4 m/s,
   because the warning gate excludes those rows. An offline replay with a >= 1 m/s filter gives the same bound-hitting result.
2. **Root cause: fake accelerations.** `load_ilc_data` computed `acc = spd[i] - spd[i-1]` across calibrate_data rows.
   Those rows are not contiguous in time: they come from different approaches, and the seed is from 2026-07-16.
   The result was fake accelerations of +19 / -12 m/s^2, which the window-5 smoothing spread over about 60 of about 150
   samples. J was about 3 and its true minimum lay at the bounds (confirmed with scipy L-BFGS-B).
3. **Gradient mismatch.** `derivation_fast.w_value` used a different formula from the `w` in `J`
   (green: `(spacing/spacing)^2`; red: `(spacing + v*T)/spacing` instead of `(s0 + v*T)/spacing`).
   The analytic gradient was 15-50% off.
4. **T derivative bug.** `T_derivative` had the wrong sign on the acc term and was not gated on tls/range.
5. **Out-of-bounds start.** `mainIDM` started at T = 2.5 > ub 2. If the line search failed at step 0, it returned T = 2.5.
6. **tls parsing.** The regex `[\d.]+` parsed tls = -1 as 1 (green).

## Fixes applied (originals in `REPO\offline_sim\backup_originals\`)
- `warning_block_Jianshe.py`, `load_ilc_data`:
  - acc is only computed between rows <= 1.5 s apart (by timestamp).
  - Smoothing (rolling 5, centered) is done per contiguous segment.
  - The first row of each segment is dropped.
  - The tls regex is now `-?[\d.]+`.
  - Timestamp parsing uses `format='mixed'`, with a fallback for older pandas.
  - Nothing else in the file changed.
- `derivation_fast.py`:
  - Added `s0 = IDM_Param().s0[0]`.
  - `w_value` now matches `J`'s.
  - `T_derivative = 2*(v_dot_hat - acc)*a*dw/dT`, with `dw/dT = -2(s0 + vT)v/spacing^2`. It returns 0 on green or beyond con_range.
- `calibrate_gradient_adaptive_step.py`, `mainIDM`: initial a, d, c, T are clamped into the bounds.
- Not changed:
  - `warning_block_Maziar.py` has its own copy of the old buggy `load_ilc_data`.
  - Existing startup bug: `warning_block_Jianshe.py` crashes with `NameError: newestData_msg` if it starts before the
    prediction block is sending (the initialisation on line 244 is commented out).

## Verification done
- Analytic gradient equals the numerical gradient exactly after the fix.
- `REPO\offline_sim\verify_edits.py`:
  - Replays 3 drivers with the live `load_ilc_data`.
  - Calibrates every `run_data\*\calibrate_data.csv` without errors, at <= 0.2 s per calibration.
  - Edge cases pass: seed only, empty file, mixed timestamps.
- End to end: a fake OBU feed sent to the real `traffic_prediction_block.py`, which fed the real `warning_block_Jianshe.py`,
  ran 2 min (run in the scratch directory). All CSVs were written and it recalibrated every 10 s.
- Results with the fix: J dropped from about 3 to about 0.25-0.37. Final parameters:
  - driver3: a=0.4, d=0.192, c=0.5, T=1.22
  - driver7: a=0.4, d=0.216, c=0.502, T=1.35
  - driver9: a=0.4, d=0.183, c=0.5, T=1.30
- **Open item:** a and c still reach their lower bounds. This is now the genuine optimum of J for this model and data.
  Candidate cause: the fit uses `warning_2` (MPC) as `u`, not the warning actually shown (`ILC_warning`).

## Tools created
- `REPO\offline_sim\offline_calibration_sim.py`: replays a run's record_data into calibrate_data plus calibration and
  writes an `ilc_params_log.csv`.
  - Options: `--min-speed`, `--fix-acc`. It has its own copies of the loaders; `--fix-acc` reproduces the fix.
  - Results in `REPO\offline_sim\results\{baseline_nofilter, minspd1, minspd1_accfix, live_code}\<run>\`.
- `REPO\offline_sim\verify_edits.py`: regression checks for the fix. It takes `load_ilc_data` from the live file via `ast`.
- `REPO\offline_sim\README.md`: findings and how to run.
- `Task 4\plotter_4vectors.py`: the `plotter_3vectors.py` plots plus a 4th line, "New ILC acc".
  - For any record CSV it replays the fixed calibration (seed, gate, 10 s interval, 50/50 blend).
  - It computes the warning each tick with the parameters in effect at that tick (params log row t-1 is used for warning t).
  - It uses `rev_warning_calc(warning_2, ...)` and the live post-filters. `load_ilc_data` and `passer` come from the live file via `ast`.
  - Options: `--params <ilc_params_log.csv>` uses a given parameter log instead; `--no-show`.
  - Outputs in `logs\`: `<stem>_new_ilc.csv`, `<stem>_acc_spd_newILC.pdf` and `.html`.
  - Validated: with the original parameter logs it reproduces the recorded `ILC_warning` exactly for drivers 9, 7 and 3 (every row).
  - Generated for drivers 9 and 7. Mean ILC warning old -> new: 34.9 -> 35.5 (driver9) and 37.5 -> 38.8 (driver7).
    The differences are mainly in the warning build-up phase.
- `plotter_3vectors.py` is unchanged.

## User preferences seen
- Wants new files rather than modifications to existing plot scripts.
- Wants originals backed up and to be told exactly which files changed.
- Wants confirmation that the live code "runs just as fine as before".
