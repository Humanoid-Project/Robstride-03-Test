#!/usr/bin/env python3
"""항목 3 (damping) 분석 — capture_velocity_hold.py 로 얻은 CSV 여러 개를 묶어서
점성 damping 계수(b, 출력축 기준 N*m/(rad/s) = URDF damping)를 구한다.

원리: 정속 상태(가속도=0)에서는 뉴턴 2법칙이

    tau_hold(omega) = b * omega + c * sign(omega)

로 단순화된다(c: 방향에 따라 부호가 바뀌는 쿨롱/정지마찰 성분, 참고용).
여러 속도(부호 섞어도 됨)에서 [omega_i, sign(omega_i)] 를 설계행렬로 최소자승
회귀하면 b(모든 방향 공통, 물리적으로 하나여야 함)와 c를 동시에 구할 수 있다.
한쪽 방향 데이터만 있으면 sign(omega)이 상수라 그냥 단순회귀(1차식)와 같아진다.

사용법:
    python3 analyze_damping.py data/id11_rs02_*.csv
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # cal_values/ (common.py)
from common import PLACEHOLDER_DAMPING  # noqa: E402


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
    if len(rows) < 5:
        print(f"  {os.path.basename(path)}: 샘플이 너무 적습니다 ({len(rows)}개) — 건너뜀")
        return None

    t = np.array([r[0] for r in rows])
    vel = np.array([r[2] for r in rows])
    tq = np.array([r[3] for r in rows])
    faults = [r[5] for r in rows if r[5] != "00"]

    mask = t >= skip_s
    if mask.sum() < 5:
        print(f"  {os.path.basename(path)}: skip 이후 샘플이 부족합니다 — skip-s 를 줄여보세요")
        return None

    vel_m, tq_m = vel[mask], tq[mask]
    omega = float(np.mean(vel_m))
    omega_std = float(np.std(vel_m))
    tau = float(np.mean(tq_m))
    speed_cmd = float(meta.get("speed_cmd_rad_s", "nan"))

    warn = ""
    if abs(omega - speed_cmd) > max(0.3, 0.15 * abs(speed_cmd)):
        warn += f"  ⚠ 목표({speed_cmd:+.3f})와 실측 평균 속도 차이가 큼"
    if faults:
        warn += f"  (참고: fault 바이트 {len(faults)}건, 의미 불명 — 무시)"

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
    p = argparse.ArgumentParser(description="정속 구간 CSV들을 묶어 damping(b) 계산")
    p.add_argument("csv_files", nargs="+", help="capture_velocity_hold.py 출력 CSV들 (glob 가능)")
    p.add_argument("--skip-s", type=float, default=0.1,
                   help="각 시행 기록 시작부에서 건너뛸 시간(초), 잔여 정착 여유용. 기본 0.1")
    args = p.parse_args()

    paths = []
    for pattern in args.csv_files:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched if matched else [pattern])
    paths = list(dict.fromkeys(paths))

    print(f"입력 파일 {len(paths)}개\n")
    results = [r for r in (analyze_file(path, args.skip_s) for path in paths) if r is not None]

    if len(results) < 2:
        print("\n유효한 시행이 2개 미만입니다. 서로 다른 속도로 최소 2회(권장 3~4회, 방향도 섞어서) 필요합니다.")
        return 1

    models = {r["model"] for r in results}
    motor_ids = {r["motor_id"] for r in results}
    if len(models) > 1 or len(motor_ids) > 1:
        print(f"\n경고: 입력 파일들의 model/motor_id가 섞여 있습니다 (models={models}, motor_ids={motor_ids}). "
              "같은 모터로 측정한 파일만 같이 넘기세요.")

    omega = np.array([r["omega"] for r in results])
    tau = np.array([r["tau"] for r in results])
    sign_omega = np.sign(omega)
    sign_omega[sign_omega == 0] = 1.0  # 정지에 가까운 시행은 사실상 없어야 하지만 방어적으로 처리

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

    print("\n" + "=" * 60)
    print("결과 (tau = b * omega + c * sign(omega) 회귀)")
    print("=" * 60)
    print(f"  b (damping, 출력축 기준) = {b:.6f} N*m/(rad/s)"
          + (f"  (±{stderr[0]:.6f}, 1-sigma)" if stderr is not None else "  (점 부족, 불확실도 계산 불가 — 3점 이상 권장)"))
    print(f"  c (방향별 쿨롱마찰 근사) = {c:+.4f} N*m"
          + (f"  (±{stderr[1]:.6f})" if stderr is not None else "") +
          "  (참고용, 항목4 정지마찰과 별개)")
    print(f"  R²                        = {r2:.4f}")

    model = next(iter(models))
    placeholder = PLACEHOLDER_DAMPING.get(model)
    if placeholder:
        ratio = b / placeholder if placeholder else float("nan")
        print(f"\n  현재 시뮬레이터 추정값({model}) = {placeholder} N*m/(rad/s)  →  실측/추정 비율 = {ratio:.1f}x")

    n_pos = int(np.sum(omega > 0))
    n_neg = int(np.sum(omega < 0))
    if n_neg == 0:
        print(f"\n  참고: 전부 양(+) 방향 속도만 측정됨({n_pos}개). c는 그 방향의 쿨롱마찰 근사치일 뿐이고,")
        print("        반대 방향도 같은 크기인지는 이 데이터만으로는 알 수 없음 — 방향 섞어서 재측정 권장.")
    elif n_pos == 0:
        print(f"\n  참고: 전부 음(-) 방향 속도만 측정됨({n_neg}개). 반대 방향도 측정해 비교 권장.")

    if r2 < 0.9:
        print("\n  ⚠ R²가 낮습니다. b가 정말 속도에 무관한 상수인지(비선형 점성마찰 가능성),")
        print("    또는 시행별 정착이 불완전했는지 확인 필요.")

    print("\n각 시행:")
    print(f"  {'file':<42} {'omega':>8} {'tau':>9} {'n':>5}")
    for r in results:
        print(f"  {os.path.basename(r['path']):<42} {r['omega']:>+8.3f} {r['tau']:>+9.4f} {r['n']:>5}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
