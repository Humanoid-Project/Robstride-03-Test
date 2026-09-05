#!/usr/bin/env python3




import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import PLACEHOLDER_DAMPING

SKIP_S = 0.1


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
    if len(rows) < 5:
        print(f"  {os.path.basename(path)}: skipped, only {len(rows)} samples")
        return None

    t = np.array([r[0] for r in rows])
    vel = np.array([r[2] for r in rows])
    tq = np.array([r[3] for r in rows])
    faults = [r[5] for r in rows if r[5] != "00"]

    mask = t >= skip_s
    if mask.sum() < 5:
        print(f"  {os.path.basename(path)}: too few samples remain after the fixed 0.1 s startup skip")
        return None

    vel_m, tq_m = vel[mask], tq[mask]
    omega = float(np.mean(vel_m))
    omega_std = float(np.std(vel_m))
    tau = float(np.mean(tq_m))
    speed_cmd = float(meta.get("speed_cmd_rad_s", "nan"))

    warn = ""
    if abs(omega - speed_cmd) > max(0.3, 0.15 * abs(speed_cmd)):
        warn += f"  WARNING: measured speed differs from target {speed_cmd:+.3f}"
    if faults:
        warn += f"  WARNING: {len(faults)} fault samples"

    print(f"  {os.path.basename(path)}: speed_cmd={speed_cmd:+.3f}  omega_meas={omega:+.4f}"
          f"(±{omega_std:.3f})  tau_meas={tau:+.4f} N*m  n={mask.sum()}{warn}")

    return {
        "path": path,
        "model": meta.get("model"),
        "motor_id": meta.get("motor_id"),
        "omega": omega,
        "tau": tau,
        "n": int(mask.sum()),
    }


def main():
    p = argparse.ArgumentParser(description="Calculate damping from constant-speed CSV files.")
    p.add_argument("csv_files", nargs="+", help="CSV paths or glob patterns")
    args = p.parse_args()

    paths = []
    for pattern in args.csv_files:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched if matched else [pattern])
    paths = list(dict.fromkeys(paths))

    print(f"Files: {len(paths)}")
    results = [r for r in (analyze_file(path, SKIP_S) for path in paths) if r is not None]

    if len(results) < 2:
        print("ERROR: At least two valid runs with different speeds are required.")
        return 1

    models = {r["model"] for r in results}
    motor_ids = {r["motor_id"] for r in results}
    if len(models) > 1 or len(motor_ids) > 1:
        print(f"ERROR: Mixed models or motor IDs: models={models}, motor_ids={motor_ids}.")
        return 1

    omega = np.array([r["omega"] for r in results])
    tau = np.array([r["tau"] for r in results])
    sign_omega = np.sign(omega)
    sign_omega[sign_omega == 0] = 1.0

    design = np.column_stack([omega, sign_omega])
    coeffs, _, _, _ = np.linalg.lstsq(design, tau, rcond=None)
    b, c = coeffs
    pred = design @ coeffs
    r2 = r_squared(tau, pred)

    stderr = None
    if len(results) > 2:
        resid = tau - pred
        dof = len(results) - 2
        sigma2 = float(np.sum(resid ** 2) / dof)
        try:
            cov = sigma2 * np.linalg.inv(design.T @ design)
            stderr = np.sqrt(np.diag(cov))
        except np.linalg.LinAlgError:
            stderr = None

    print()
    print("Result: tau = b * omega + c * sign(omega)")
    print(f"  output damping b = {b:.6f} N*m/(rad/s)"
          + (f" (±{stderr[0]:.6f}, 1-sigma)" if stderr is not None else ""))
    print(f"  Coulomb friction c = {c:+.4f} N*m"
          + (f"  (±{stderr[1]:.6f})" if stderr is not None else "") +
          "")
    print(f"  R²                        = {r2:.4f}")

    model = next(iter(models))
    placeholder = PLACEHOLDER_DAMPING.get(model)
    if placeholder:
        ratio = b / placeholder if placeholder else float("nan")
        print(f"  simulator={placeholder} N*m/(rad/s), measured/simulator={ratio:.1f}x ({model})")

    n_pos = int(np.sum(omega > 0))
    n_neg = int(np.sum(omega < 0))
    if n_neg == 0:
        print(f"WARNING: Only {n_pos} positive-speed runs were measured.")
    elif n_pos == 0:
        print(f"WARNING: Only {n_neg} negative-speed runs were measured.")

    if not np.isfinite(r2) or r2 < 0.9:
        print("WARNING: Low R²; verify settling and possible nonlinear damping.")

    print("\nRuns:")
    print(f"  {'file':<42} {'omega':>8} {'tau':>9} {'n':>5}")
    for r in results:
        print(f"  {os.path.basename(r['path']):<42} {r['omega']:>+8.3f} {r['tau']:>+9.4f} {r['n']:>5}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
