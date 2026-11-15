"""Run on the field laptop before a road test, from this folder, with the Python you will drive with:
    python preflight_check.py
Checks the Python packages, that the ESSCV modules import, that the UDP ports are free, that run_data is
writable and that matplotlib can open a window. Prints PASS/FAIL per item.
"""
import os
import sys
import socket
import importlib

results = []


def check(name, fn):
    try:
        info = fn()
        results.append((True, name, info or ""))
    except Exception as e:
        results.append((False, name, f"{type(e).__name__}: {e}"))


def version(mod):
    return lambda: getattr(importlib.import_module(mod), "__version__", "ok")


check(f"Python {sys.version.split()[0]} ({sys.executable})", lambda: "")
for mod in ["numpy", "scipy", "pandas", "matplotlib", "keyboard"]:
    check(f"import {mod}", version(mod))
check("import pycmssdk (OBU SDK, needed by obu_info_block.py)", lambda: (importlib.import_module("pycmssdk"), "ok")[1])
check("numpy imports cleanly (NumPy 1.x on Python >= 3.13 crashes)", lambda: __import__("numpy").zeros(3).sum() == 0 and "ok")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
for mod in ["cav_data", "esscv_optimization", "esscv_driver_model", "prediction", "UKF", "distance_finder"]:
    check(f"import {mod}", lambda m=mod: (importlib.import_module(m), "ok")[1])


def esscv_smoke():
    import numpy as np
    import esscv_optimization as eo
    import esscv_driver_model as dm
    tl = eo.SignalTimeline("red", 20.0)
    plan, sol = eo.solve_mpc(200.0, 20.0, 22.0, tl, eo.initial_guess(), 0.0)
    target, src = dm.display_target(float(plan[2]), 20.0, 200.0, True, 22.0, dict(dm.INIT_PARAMS), True,
                                    eo.stop_speed(200.0, 20.0, plan) if eo.must_stop(200.0, 20.0, 22.0, tl) else None,
                                    eo.planned_speed(20.0, plan, dm.RESTART_AHEAD))
    assert np.isfinite(target)
    return f"MPC solve ok, target {target * 2.23694:.1f} mph ({src})"


check("ESSCV MPC + display smoke test", esscv_smoke)


def port_free(port):
    def fn():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("", port))
        s.close()
        return "free"
    return fn


for port, who in [(10888, "traffic_prediction_block receives OBU"), (10889, "warning_block_ESSCV receives prediction"),
                  (10900, "warning_block_ESSCV receives OBU message"), (10890, "plotting_block_ESSCV receives display")]:
    check(f"UDP port {port} free ({who})", port_free(port))


def run_data_writable():
    os.makedirs("run_data", exist_ok=True)
    p = os.path.join("run_data", ".preflight")
    open(p, "w").write("ok")
    os.remove(p)
    return os.path.abspath("run_data")


check("run_data writable", run_data_writable)


def gui_backend():
    import matplotlib
    import matplotlib.pyplot as plt
    fig = plt.figure()
    plt.close(fig)
    backend = matplotlib.get_backend()
    if backend.lower() in ("agg", "pdf", "svg", "template"):
        raise RuntimeError(f"non-interactive backend '{backend}' (check MPLBACKEND)")
    return backend


check("matplotlib can open a window", gui_backend)

for ok, name, info in results:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}  {info}")
print("\nALL PASS" if all(r[0] for r in results) else "\nSOME CHECKS FAILED - see above")
