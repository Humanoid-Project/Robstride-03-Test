#!/usr/bin/env python3
"""항목 2 (armature) 분석 — capture_torque_step.py 로 얻은 CSV 여러 개를 묶어서
회전자+감속기 관성(J, 출력축 기준 kg*m^2 = URDF armature)을 구한다.

원리: 정지 상태에서 서로 다른 토크 tau_i 를 인가했을 때 초반 가속도 alpha_i 를
구하면, 두 시행에서 마찰항이 거의 동일하다고 볼 수 있어

    tau_i = J * alpha_i + friction   (friction: 상수항, 참고용)

이 되고, (alpha_i, tau_i) 여러 점을 선형회귀하면 기울기가 J, 절편이 마찰토크의
근사치다 (항목 4의 정지마찰 측정값과 교차검증 가능하지만, 이건 동적 상태의 근사이므로
항목 4 결과를 우선시할 것).

사용법:
    python3 analyze_armature.py data/id11_rs02_*.csv
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # cal_values/ (common.py)
from common import PLACEHOLDER_ARMATURE  # noqa: E402


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
        return 1.0
    return 1.0 - ss_res / ss_tot


def analyze_file(path, skip_s):
    meta, rows = load_csv(path)
    if len(rows) < 3:
        print(f"  {os.path.basename(path)}: 샘플이 너무 적습니다 ({len(rows)}개) — 건너뜀")
        return None

    t = np.array([r[0] for r in rows])
    vel = np.array([r[2] for r in rows])
    tq = np.array([r[3] for r in rows])
    faults = [r[5] for r in rows if r[5] != "00"]

    mask = t >= skip_s
    if mask.sum() < 3:
        print(f"  {os.path.basename(path)}: skip 이후 샘플이 부족합니다 — skip-ms 를 줄여보세요")
        return None

    t_m, vel_m, tq_m = t[mask], vel[mask], tq[mask]
    alpha, v0 = np.polyfit(t_m, vel_m, 1)
    pred = alpha * t_m + v0
    r2 = r_squared(vel_m, pred)
    tau_mean = float(np.mean(tq_m))
    tau_cmd = float(meta.get("torque_cmd_Nm", "nan"))

    warn = ""
    if r2 < 0.98:
        warn += f"  ⚠ R²={r2:.4f} (선형성 낮음, 재측정 권장)"
    if faults:
        warn += f"  ⚠ 폴트 {len(faults)}건 포함(마지막 샘플 부근)"

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
    p = argparse.ArgumentParser(description="토크 스텝 CSV들을 묶어 armature(J) 계산")
    p.add_argument("csv_files", nargs="+", help="capture_torque_step.py 출력 CSV들 (glob 가능)")
    p.add_argument("--skip-ms", type=float, default=15.0,
                   help="각 시행 시작부에서 건너뛸 시간(ms), 전류 상승 과도구간 제외용. 기본 15")
    args = p.parse_args()

    paths = []
    for pattern in args.csv_files:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched if matched else [pattern])
    paths = [p for p in dict.fromkeys(paths)]  # 중복 제거, 순서 유지

    print(f"입력 파일 {len(paths)}개\n")
    results = [r for r in (analyze_file(path, args.skip_ms / 1000.0) for path in paths) if r is not None]

    if len(results) < 2:
        print("\n유효한 시행이 2개 미만입니다. J를 구하려면 서로 다른 토크로 최소 2회(권장 3~4회) 필요합니다.")
        return 1

    models = {r["model"] for r in results}
    motor_ids = {r["motor_id"] for r in results}
    if len(models) > 1 or len(motor_ids) > 1:
        print(f"\n경고: 입력 파일들의 model/motor_id가 섞여 있습니다 (models={models}, motor_ids={motor_ids}). "
              "같은 모터로 측정한 파일만 같이 넘기세요.")

    model = next(iter(models))
    tau = np.array([r["tau_meas"] for r in results])
    alpha = np.array([r["alpha"] for r in results])

    # tau = J*alpha + friction*sign(alpha). analyze_damping.py와 동일한 부호인식 방식 —
    # 지금까지는 항상 같은 부호(+) 토크만 써서 friction이 상수 절편과 수학적으로 동일했지만,
    # 나중에 음의 토크도 섞어 쓰면(대칭성 확인 등) friction 부호가 방향에 따라 뒤집혀야
    # 맞으므로 미리 일반화해둔다(같은 부호 데이터에서는 결과가 기존과 완전히 동일함).
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

    print("\n" + "=" * 60)
    print("결과 (tau = J * alpha + friction * sign(alpha) 회귀)")
    print("=" * 60)
    print(f"  J (armature, 출력축 기준) = {J:.6f} kg*m^2"
          + (f"  (±{j_stderr:.6f}, 1-sigma)" if j_stderr is not None else "  (2점 회귀, 불확실도 계산 불가 — 3점 이상 권장)"))
    print(f"  friction 절편(동적 근사)  = {friction:+.4f} N*m  (참고용, 항목4 정지마찰과 별개)")
    print(f"  R²                        = {r2:.4f}")

    placeholder = PLACEHOLDER_ARMATURE.get(model)
    if placeholder:
        ratio = J / placeholder if placeholder else float("nan")
        print(f"\n  현재 시뮬레이터 추정값({model}) = {placeholder} kg*m^2  →  실측/추정 비율 = {ratio:.1f}x")

    if r2 < 0.95:
        print("\n  ⚠ R²가 낮습니다. 시행 간 마찰이 크게 달랐거나(속도 구간이 서로 다름) 노이즈가 큽니다.")
        print("    -> 토크값들을 더 넓게 벌리거나, skip-ms를 조절하거나, 같은 조건에서 반복 측정 권장.")

    print("\n각 시행:")
    print(f"  {'file':<40} {'tau_cmd':>8} {'tau_meas':>9} {'alpha':>10} {'n':>4} {'R2':>7}")
    for r in results:
        print(f"  {os.path.basename(r['path']):<40} {r['tau_cmd']:>+8.3f} {r['tau_meas']:>+9.3f} "
              f"{r['alpha']:>+10.2f} {r['n']:>4} {r['r2']:>7.4f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
