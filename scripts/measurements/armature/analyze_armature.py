#!/usr/bin/env python3
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import PLACEHOLDER_ARMATURE

SKIP_S = 0.015


def load_csv(path):
    meta = {}
    rows = []
    header_seen = False
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\n")
            if not line:
                continue
            if line.startswith("#"):
                key, _, val = line[1:].partition(":")
                meta[key.strip()] = val.strip()
                continue
            if not header_seen:
                header_seen = True
                continue
            t_s, pos, vel, tq, temp, fault_hex = line.split(",")
            rows.append((float(t_s), float(pos), float(vel), float(tq), float(temp), fault_hex))
    return meta, rows


def r_squared(y, y_pred):
    y = np.asarray(y)
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    if ss_tot == 0:
        return 1.0 if ss_res == 0 else float("nan")
    return 1.0 - ss_res / ss_tot


def analyze_file(path, skip_s):
    meta, rows = load_csv(path)
    if len(rows) < 3:
        print(f"  {os.path.basename(path)}: skipped, only {len(rows)} samples")
        return None

    t = np.array([r[0] for r in rows])
    vel = np.array([r[2] for r in rows])
    tq = np.array([r[3] for r in rows])
    faults = [r[5] for r in rows if r[5] != "00"]

    mask = t >= skip_s
    if mask.sum() < 3:
        print(f"  {os.path.basename(path)}: too few samples remain after the fixed 15 ms startup skip")
        return None

    t_m, vel_m, tq_m = t[mask], vel[mask], tq[mask]
    alpha, v0 = np.polyfit(t_m, vel_m, 1)
    pred = alpha * t_m + v0
    r2 = r_squared(vel_m, pred)
    tau_mean = float(np.mean(tq_m))
    tau_cmd = float(meta.get("torque_cmd_Nm", "nan"))

    warn = ""
    if not np.isfinite(r2) or r2 < 0.98:
        warn += f"  WARNING: low linearity (R²={r2:.4f})"
    if faults:
        warn += f"  WARNING: {len(faults)} fault samples"

    print(f"  {os.path.basename(path)}: tau_cmd={tau_cmd:+.3f}  tau_meas_avg={tau_mean:+.3f} N*m  "
          f"alpha={alpha:+.2f} rad/s^2  n={mask.sum()}  R²={r2:.4f}{warn}")

    return {
        "path": path,
        "model": meta.get("model"),
        "motor_id": meta.get("motor_id"),
        "tau_cmd": tau_cmd,
        "tau_meas": tau_mean,
        "alpha": alpha,
        "r2": r2,
        "n": int(mask.sum()),
        "fault_count": len(faults),
    }


def main():
    p = argparse.ArgumentParser(description="Calculate armature inertia from torque-step CSV files.")
    p.add_argument(
        "csv_files", nargs="+",
        help="One or more armature.py CSV paths or glob patterns for the same motor",
    )
    args = p.parse_args()

    paths = []
    for pattern in args.csv_files:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched if matched else [pattern])
    paths = [p for p in dict.fromkeys(paths)]

    print(f"Files: {len(paths)}")
    results = [r for r in (analyze_file(path, SKIP_S) for path in paths) if r is not None]

    if len(results) < 2:
        print("ERROR: At least two valid runs with different torques are required.")
        return 1

    models = {r["model"] for r in results}
    motor_ids = {r["motor_id"] for r in results}
    if len(models) > 1 or len(motor_ids) > 1:
        print(f"ERROR: Mixed models or motor IDs: models={models}, motor_ids={motor_ids}.")
        return 1

    model = next(iter(models))
    tau = np.array([r["tau_meas"] for r in results])
    alpha = np.array([r["alpha"] for r in results])

    sign_alpha = np.sign(alpha)
    sign_alpha[sign_alpha == 0] = 1.0
    design = np.column_stack([alpha, sign_alpha])

    if len(results) >= 3:
        coeffs, _, _, _ = np.linalg.lstsq(design, tau, rcond=None)
        J, friction = coeffs
        resid = tau - design @ coeffs
        dof = len(results) - 2
        sigma2 = float(np.sum(resid ** 2) / dof)
        try:
            cov = sigma2 * np.linalg.inv(design.T @ design)
            j_stderr = float(np.sqrt(cov[0, 0]))
        except np.linalg.LinAlgError:
            j_stderr = None
    else:
        coeffs, _, _, _ = np.linalg.lstsq(design, tau, rcond=None)
        J, friction = coeffs
        j_stderr = None

    pred = design @ np.array([J, friction])
    r2 = r_squared(tau, pred)

    print()
    print("Result: tau = J * alpha + friction * sign(alpha)")
    print(f"  output armature J = {J:.6f} kg*m^2"
          + (f" (±{j_stderr:.6f}, 1-sigma)" if j_stderr is not None else ""))
    print(f"  dynamic friction intercept = {friction:+.4f} N*m")
    print(f"  R²                        = {r2:.4f}")

    placeholder = PLACEHOLDER_ARMATURE.get(model)
    if placeholder:
        ratio = J / placeholder if placeholder else float("nan")
        print(f"  simulator={placeholder} kg*m^2, measured/simulator={ratio:.1f}x ({model})")

    if not np.isfinite(r2) or r2 < 0.95:
        print("WARNING: Low R²; repeat the measurement under consistent conditions.")
        print("    Use a wider torque range or repeat the measurement under the same conditions.")

    print("\nRuns:")
    print(f"  {'file':<40} {'tau_cmd':>8} {'tau_meas':>9} {'alpha':>10} {'n':>4} {'R2':>7}")
    for r in results:
        print(f"  {os.path.basename(r['path']):<40} {r['tau_cmd']:>+8.3f} {r['tau_meas']:>+9.3f} "
              f"{r['alpha']:>+10.2f} {r['n']:>4} {r['r2']:>7.4f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
