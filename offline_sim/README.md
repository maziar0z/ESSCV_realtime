# Offline ILC calibration replay

`offline_calibration_sim.py` replays a run's `record_data.csv` through the same logic as
`warning_block_Jianshe.py`:

- Each row is one 1 s tick.
- A row is appended to calibrate_data when `MPC_warning > 0.01 and warning_1 > 0.01`
  **and** `spd_ego >= --min-speed` (default 1 m/s).
- Every 10 ticks, `mainIDM` (from `calibrate_gradient_adaptive_step.py`) runs and its output
  is blended 50/50 with the current parameters.
- The output is an `ilc_params_log.csv` with one row per tick, in the same format as the live file.
- calibrate_data is seeded from `calibrate_data_reference.csv`, as in the live run.

```
python offline_calibration_sim.py driver3_20260814 driver7_20260804 driver9_20260731            # -> results/minspd1/
python offline_calibration_sim.py driver3_20260814 driver7_20260804 driver9_20260731 --fix-acc  # -> results/minspd1_accfix/
```

Each `results/<tag>/<run>/` folder contains:

- `ilc_params_log.csv`
- `calibration_events.csv`: the raw (un-blended) mainIDM output and the sample count for each calibration
- `calibrate_data.csv`: the calibration data as it stood at the end of the run

| folder | what it is |
|---|---|
| `results/baseline_nofilter/driver3_20260814` | no speed filter. Reproduces the live `ilc_params_log.csv` (sanity check of the replay) |
| `results/minspd1/` | requested run: rows with speed < 1 m/s excluded |
| `results/minspd1_accfix/` | speed filter + acceleration computed only between time-consecutive rows |

## Findings

1. **The stopped-vehicle hypothesis does not hold.** None of the 12 driver runs has a row with
   `spd_ego < 1` in its calibrate_data, because the warning gate already excludes them
   (minimum speed in calibrate_data is about 4 m/s). So `minspd1` gives the same result as the
   live run: a=0.4, d=0.06, c=0.5, T=0.5. These are exactly the lower bounds.
2. **Root cause: fake accelerations.** `load_ilc_data` computes `acc = spd[i] - spd[i-1]` over
   calibrate_data rows. Those rows only exist while a warning is active, so neighbouring rows
   often come from different approaches, or from the July-16 reference seed. The differences
   reach +19 / -12 m/s^2. The window-5 smoothing then spreads each spike over 5 samples, which
   corrupts about 60 of about 150 samples. J is dominated by these outliers (J ~ 3), and its
   minimum lies at the lower bounds. A bounded L-BFGS-B fit on the same J also lands on the
   bounds, so the descent algorithm is not the cause.
3. **With `--fix-acc`, J drops to about 0.3-0.6.** Here acc is only taken across gaps <= 1.5 s, the
   first row of each segment is dropped, and smoothing is done within a segment. Final parameters
   for driver3 / driver7 / driver9:
   - a = 0.55 / 0.57 / 0.57
   - d = 0.18 / 0.20 / 0.18
   - T = 0.80 / 0.73 / 0.81
   - c = 0.50 / 0.50 / 0.52, still at or near its 0.5 lower bound.
4. Other issues found in the code (not fixed in the replay unless noted):
   - `derivation_fast.w_value` differs from `w_value` in `calibrate_gradient_adaptive_step.py`.
     For green, `derivation_fast` uses `(spacing/spacing)^2` (= 1) where `J` uses 0. For red,
     `derivation_fast` uses `(spacing + v*T)/spacing` where `J` uses `(s0 + v*T)/spacing`.
     As a result the analytic gradient is 15-50% off the true gradient of J. The Armijo search
     still only accepts decreases of J, so it lands near but not at the optimum.
   - `T_derivative` has the wrong sign on the `acc_ego` term and ignores tls/range.
   - The tls regex `[\d.]+` reads `-1` as `1` (green). The accfix loader uses `-?[\d.]+`.
   - The fit uses `warning_2` (MPC) as the warning, not the warning actually shown (`ILC_warning`).

## Fix applied to the live code (2026-10-13)

Originals are saved in `backup_originals/`.

- `warning_block_Jianshe.py`, `load_ilc_data`: acc is only computed between rows that are
  <= 1.5 s apart, smoothing is done per contiguous segment, the first row of each segment is
  dropped, and the tls regex keeps the `-1` sign.
- `derivation_fast.py`: `w_value` now matches `J`'s `w_value`. `T_derivative` now has the correct
  sign and is 0 on green / out of range. Analytic and numerical gradients now agree exactly.
- `calibrate_gradient_adaptive_step.py`, `mainIDM`: the start values are clamped into the bounds
  (initial T=2.5 > ub 2.0 made the line search fail at step 0 and return T=2.5).

`verify_edits.py` re-checks all of this. It replays with the live `load_ilc_data` and writes
`results/live_code/`, calibrates every `run_data/*/calibrate_data.csv`, and covers edge cases.
With the exact gradient, mainIDM reaches the true optimum of J: d ~ 0.18-0.22 and T ~ 1.2-1.35
for driver3/7/9. a and c still sit at their lower bounds (0.4 / 0.5), but that is now the real
minimum of J for this model and data, not an artefact.
