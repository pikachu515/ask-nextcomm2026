"""정보 고르기 모의실험 (개념 검증)

네 갈래 교차로 하나에서 8초 동안을 0.1초 단위로 흉내 낸다.
  - 차 24대(갈래마다 6대)가 정지선으로 다가온다. 직진, 좌회전, 우회전 중 하나를 한다.
  - 보행자 12명이 네 횡단보도를 건넌다.
  - 일부 갈래에는 안쪽 차로에 대형 차량이 서 있어 바깥 차로 차의 시야를 가린다.
  - 길가 센서는 보행자를 위에서 보고(일부 놓침), 무선으로 차에 알린다.

차가 스스로 못 보는 보행자가 자기 길의 횡단보도에 있는데, 브레이크를 밟아야 하는 시점
(마감)까지 0.5초 안에 본 정보를 받지 못하면 '놓침'으로 센다.

무선 자원은 0.1초마다 정해진 양(B)만 쓸 수 있다.
  - 한 차에 따로 보내기: 1칸, 수신 성공 98%
  - 모두에게 방송: c_bc칸 (가장 약한 차에 맞춰야 해서 비쌈), 수신 성공 90%

비교 방식
  bcast_all  모든 보행자를 돌아가며 방송
  cpm_like   표준과 비슷한 규칙(약식): 0.5초가 지났거나 1m 넘게 움직인 보행자만 방송
  ask        가림·충돌까지 남은 시간·정보의 나이로 누구에게 무엇이 필요한지 고르고,
             필요한 차가 많으면 방송, 적으면 따로 보냄

실행
  python selection_sim.py              # 시드 100개
  python selection_sim.py --seeds 20
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

HERE = Path(__file__).resolve().parent

# 모두 가정값이다. README.md 의 표와 같아야 한다.
PARAMS = dict(
    T=8.0, dt=0.1,
    K_per_arm=6, P=12,
    d0=(25.0, 140.0),        # 처음 정지선까지 거리 [m]
    v_kmh=(30.0, 50.0),
    p_truck=0.5,             # 갈래마다 안쪽 차로에 대형 차량이 서 있을 확률
    own_range=80.0,          # 차 자기 센서로 보는 거리 [m]
    p_rsu_miss=0.05,         # 길가 센서가 보행자를 놓칠 확률 (보행자마다)
    rsu_lat=0.15,            # 길가 센서가 보고 처리해 보내기까지 [s] (정보의 나이 시작값)
    ttl=0.5,                 # 정보를 써도 되는 기한 [s]
    B=8,                     # 0.1초마다 쓸 수 있는 무선 자원 [칸]
    c_bc=4,                  # 방송 한 건의 비용 [칸]
    p_uc=0.98, p_bc=0.90,    # 수신 성공 확률
    tau=0.3, a=2.0, margin=2.0,
    ped_v=1.3,
    horizon=8.0,             # 충돌까지 남은 시간이 이보다 길면 관련 없다고 본다 [s]
    refresh=0.3,             # ASK가 같은 차에 같은 보행자를 다시 보내는 간격 [s]
    cpm_dt=0.5, cpm_move=1.0,
    p_occ_err=0.10,          # 길가 센서가 "그 차에서 보이는가"를 잘못 판단할 확률 (매 순간)
    p_route_err=0.10,        # 길가 센서가 차의 진행 방향(직진/좌/우)을 잘못 짐작할 확률 (차마다)
)

HALF = 7.0                   # 도로 반폭 [m]
CW = (9.0, 12.0)             # 횡단보도 띠 (중심에서 거리) [m]
LANE_OUT, LANE_IN = 5.25, 1.75


def rot(arm):
    """갈래 0=남쪽에서 북쪽으로, 1=서→동, 2=북→남, 3=동→서. 지역 좌표를 세계 좌표로 돌리는 행렬."""
    c, s = [(1, 0), (0, -1), (-1, 0), (0, 1)][arm]
    return np.array([[c, -s], [s, c]], float)


def seg_hits_rect(p0, p1, rects):
    """선분 p0→p1 이 직사각형(축 정렬, [xmin,xmax,ymin,ymax]) 가운데 하나라도 지나는가. 여러 선분을 한 번에."""
    hit = np.zeros(p0.shape[0], bool)
    d = p1 - p0
    for xmin, xmax, ymin, ymax in rects:
        t0 = np.zeros(p0.shape[0]); t1 = np.ones(p0.shape[0]); ok = np.ones(p0.shape[0], bool)
        for k, lo, hi in ((0, xmin, xmax), (1, ymin, ymax)):
            dk = d[:, k]; pk = p0[:, k]
            par = np.abs(dk) < 1e-12
            ok &= ~(par & ((pk < lo) | (pk > hi)))
            with np.errstate(divide="ignore", invalid="ignore"):
                ta = (lo - pk) / dk; tb = (hi - pk) / dk
            tmin = np.where(par, -np.inf, np.minimum(ta, tb)); tmax = np.where(par, np.inf, np.maximum(ta, tb))
            t0 = np.maximum(t0, tmin); t1 = np.minimum(t1, tmax)
        hit |= ok & (t0 <= t1)
    return hit


def build(seed, p):
    rng = np.random.default_rng(seed)
    K = 4 * p["K_per_arm"]
    arm = np.repeat(np.arange(4), p["K_per_arm"])
    move = rng.integers(0, 3, K)                         # 0 직진, 1 우회전, 2 좌회전
    v = rng.uniform(*p["v_kmh"], K) / 3.6
    d0 = np.sort(rng.uniform(*p["d0"], K).reshape(4, -1), axis=1).ravel()
    exit_arm = np.where(move == 0, (arm + 2) % 4, np.where(move == 1, (arm + 3) % 4, (arm + 1) % 4))
    # 대형 차량: 안쪽 차로, 정지선 바로 뒤 (지역 좌표 y=-(CW1+1) ~ -(CW1+11))
    rects = []
    for g in range(4):
        if rng.random() < p["p_truck"]:
            corners = np.array([[LANE_IN - 1.25, -CW[1] - 12], [LANE_IN + 1.25, -CW[1] - 2]])
            w = corners @ rot(g).T
            rects.append([w[:, 0].min(), w[:, 0].max(), w[:, 1].min(), w[:, 1].max()])
    # 보행자: 횡단보도마다 3명, 가로로 걷는다
    P = p["P"]
    ped_arm = np.repeat(np.arange(4), P // 4)
    ped_x0 = rng.uniform(-HALF, HALF, P)
    ped_dir = rng.choice([-1.0, 1.0], P)
    ped_y = rng.uniform(*CW, P)
    rsu_sees = rng.random(P) > p["p_rsu_miss"]
    return dict(K=K, arm=arm, move=move, v=v, d0=d0, exit_arm=exit_arm, rects=rects,
                ped_arm=ped_arm, ped_x0=ped_x0, ped_dir=ped_dir, ped_y=ped_y, rsu_sees=rsu_sees,
                rng_tx=np.random.default_rng(seed + 50_000))


def positions(sc, t, p):
    """시각 t의 차와 보행자 위치 (세계 좌표)."""
    K, P = sc["K"], len(sc["ped_arm"])
    car = np.zeros((K, 2))
    for i in range(K):
        y = -(CW[1] + sc["d0"][i]) + sc["v"][i] * t
        car[i] = rot(sc["arm"][i]) @ np.array([LANE_OUT, min(y, -CW[1])])
    x = sc["ped_x0"] + sc["ped_dir"] * p["ped_v"] * t
    x = (x + HALF) % (4 * HALF)
    x = np.where(x > 2 * HALF, 4 * HALF - x, x) - HALF        # 횡단보도 끝에서 되돌아 걷는다
    ped = np.zeros((P, 2))
    for j in range(P):
        ped[j] = rot(sc["ped_arm"][j]) @ np.array([x[j], -sc["ped_y"][j]])
    return car, ped


def pair_info(sc, p):
    """차-보행자 쌍마다 길이 겹치는지, 마감(브레이크를 밟을 시점)이 언제인지."""
    K, P = sc["K"], len(sc["ped_arm"])
    conflict = np.zeros((K, P), bool)
    dist = np.full((K, P), np.inf)
    for i in range(K):
        for j in range(P):
            if sc["ped_arm"][j] == sc["arm"][i]:
                conflict[i, j] = True; dist[i, j] = sc["d0"][i]
            elif sc["ped_arm"][j] == sc["exit_arm"][i]:
                conflict[i, j] = True; dist[i, j] = sc["d0"][i] + 2 * CW[1] + (6.0 if sc["move"][i] == 2 else 0.0)
    v = sc["v"][:, None]
    d_stop = v * p["tau"] + v * v / (2 * p["a"]) + p["margin"]
    t_dl = (dist - d_stop) / v
    ttc = dist / v
    return conflict, t_dl, ttc


def run_policy(seed, policy, **override):
    """정보를 보내고, 마감 시점에 쓸 수 있는 정보를 받았는지 센다. 평가는 실제 값, ASK의 판단은 추정값으로 한다."""
    p = {**PARAMS, **override}
    sc = build(seed, p)
    K, P = sc["K"], len(sc["ped_arm"])
    conflict, t_dl, ttc = pair_info(sc, p)
    rng_e = np.random.default_rng(seed + 70_000)
    guess = dict(sc)
    wrong = rng_e.random(K) < p["p_route_err"]
    guess["exit_arm"] = np.where(wrong, (sc["exit_arm"] + rng_e.integers(1, 3, K)) % 4, sc["exit_arm"])
    guess["move"] = np.where(wrong & (guess["exit_arm"] == (sc["arm"] + 1) % 4), 2, sc["move"])
    conf_est, t_dl_est, ttc_est = pair_info(guess, p)
    steps = int(round(p["T"] / p["dt"]))
    ii, jj = np.meshgrid(np.arange(K), np.arange(P), indexing="ij")
    rx_log = np.zeros((steps, K, P), bool)
    last_rx = np.full((K, P), -np.inf)
    last_bc = np.full(P, -np.inf)
    last_bc_pos = np.full((P, 2), np.nan)
    rr = 0
    units = 0
    occ_at = np.zeros((steps, K, P), bool)
    rng = sc["rng_tx"]
    seen = sc["rsu_sees"]
    useful = wasted = 0

    for s in range(steps):
        t = s * p["dt"]
        car, ped = positions(sc, t, p)
        blocked = seg_hits_rect(car[ii.ravel()], ped[jj.ravel()], sc["rects"]).reshape(K, P)
        far = np.linalg.norm(car[:, None, :] - ped[None, :, :], axis=2) > p["own_range"]
        occ = blocked | far
        occ_at[s] = occ
        budget = p["B"]
        sends = []
        if policy == "bcast_all":
            for _ in range(budget // p["c_bc"]):
                for _try in range(P):
                    j = rr % P; rr += 1
                    if seen[j]:
                        sends.append((p["c_bc"], None, j)); break
        elif policy == "cpm_like":
            due = [j for j in range(P) if seen[j] and (t - last_bc[j] >= p["cpm_dt"] - 1e-9 or
                   np.isnan(last_bc_pos[j, 0]) or np.linalg.norm(ped[j] - last_bc_pos[j]) > p["cpm_move"])]
            due.sort(key=lambda j: last_bc[j])
            for j in due[: budget // p["c_bc"]]:
                sends.append((p["c_bc"], None, j)); last_bc[j] = t; last_bc_pos[j] = ped[j]
        else:
            remain = t_dl_est - t
            occ_est = occ ^ (rng_e.random((K, P)) < p["p_occ_err"])
            valid = seen[None, :] & conf_est & (remain > 0)
            R = np.clip(1 - ttc_est / p["horizon"], 0, 1) * conf_est
            O = occ_est.astype(float)
            N = (t - last_rx >= p["refresh"]).astype(float)
            U = (remain > p["rsu_lat"]).astype(float)
            eta = 0.1
            V = np.where(valid, R * (eta + (1 - eta) * O) * N * U, 0.0)
            urgent = valid & occ_est & (remain < 1.5) & (N > 0)
            score = V + urgent * 10.0
            for j in np.argsort(-score.max(axis=0)):
                need = np.flatnonzero(score[:, j] > 0.05)
                if need.size == 0 or budget < 1:
                    continue
                need = need[np.argsort(-score[need, j])]
                if need.size >= p["c_bc"] and budget >= p["c_bc"]:
                    sends.append((p["c_bc"], None, j)); budget -= p["c_bc"]
                else:
                    for i in need[: int(budget)]:
                        sends.append((1, [i], j)); budget -= 1
        for cost, who, j in sends:
            units += cost
            if who is None:
                ok = rng.random(K) < p["p_bc"]
            else:
                ok = np.zeros(K, bool)
                for i in who:
                    ok[i] = rng.random() < p["p_uc"]
            rx_log[s, ok, j] = True
            last_rx[ok, j] = t
            rel = ok & conflict[:, j] & occ[:, j] & ((t_dl[:, j] - t) > 0)
            useful += int(rel.sum()); wasted += int((ok & ~rel).sum())

    crit = miss = 0
    win = int(round((p["ttl"] - p["rsu_lat"]) / p["dt"]))   # 마감 전 이 칸 수 안에 받았어야 한다
    for i in range(K):
        for j in range(P):
            td = t_dl[i, j]
            if not (conflict[i, j] and seen[j] and 0.3 <= td < p["T"]):
                continue
            s = int(td / p["dt"])
            if not occ_at[s, i, j]:
                continue
            crit += 1
            if not rx_log[max(0, s - win): s + 1, i, j].any():
                miss += 1
    return dict(crit=crit, miss=miss, units_per_s=units / p["T"], useful=useful, wasted=wasted)


POLICIES = [("bcast_all", "모두 돌아가며 방송"), ("cpm_like", "표준과 비슷한 규칙"), ("ask", "ASK: 필요한 차에 고르기")]


def set_font():
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in ("Malgun Gothic", "AppleGothic", "NanumGothic", "Noto Sans CJK KR"):
        if name in installed:
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False


def summarize(seeds, **override):
    out = {}
    for k, _ in POLICIES:
        rs = [run_policy(s, k, **override) for s in seeds]
        crit = sum(r["crit"] for r in rs); miss = sum(r["miss"] for r in rs)
        per_seed = [100 * r["miss"] / r["crit"] for r in rs if r["crit"] > 0]
        half = 1.984 * np.std(per_seed, ddof=1) / math.sqrt(len(per_seed)) if len(per_seed) > 1 else 0.0
        out[k] = dict(crit=crit, miss=miss, miss_pct=100 * miss / max(1, crit), miss_pct_ci95=half,
                      units_per_s=float(np.mean([r["units_per_s"] for r in rs])),
                      useful_share=100 * sum(r["useful"] for r in rs) / max(1, sum(r["useful"] + r["wasted"] for r in rs)))
    return out


def main():
    ap = argparse.ArgumentParser(description="정보 고르기 모의실험")
    ap.add_argument("--seeds", type=int, default=100)
    ap.add_argument("--sweep-seeds", type=int, default=30)
    args = ap.parse_args()
    out = HERE / "results"
    out.mkdir(exist_ok=True)
    set_font()

    res = summarize(range(args.seeds))
    rows = []
    print(f"가정값 기준 · 시드 {args.seeds}개 · 자원 {PARAMS['B']}칸/0.1초 · 방송 비용 {PARAMS['c_bc']}칸")
    print(f"{'방식':<20}{'꼭 받아야 할 쌍':>12}{'놓친 비율':>10}{'쓴 자원[칸/s]':>14}{'받은 것 중 쓸모':>14}")
    for k, label in POLICIES:
        r = res[k]
        rows.append(dict(policy=k, label=label, n_seeds=args.seeds, B=PARAMS["B"], c_bc=PARAMS["c_bc"],
                         critical_pairs=r["crit"], missed=r["miss"], miss_pct=round(r["miss_pct"], 2),
                         miss_pct_ci95=round(r["miss_pct_ci95"], 2), units_per_s=round(r["units_per_s"], 1),
                         useful_share_pct=round(r["useful_share"], 1)))
        print(f"{label:<20}{r['crit']:>12}{r['miss_pct']:>9.1f}%{r['units_per_s']:>14.1f}{r['useful_share']:>13.1f}%")
    # 붐비는 교차로 (차 40대, 보행자 20명)
    busy = summarize(range(args.seeds), K_per_arm=10, P=20)
    for k, label in POLICIES:
        r = busy[k]
        rows.append(dict(policy=k + "_busy", label=label + " (붐빔: 차 40대, 보행자 20명)", n_seeds=args.seeds, B=PARAMS["B"],
                         c_bc=PARAMS["c_bc"], critical_pairs=r["crit"], missed=r["miss"], miss_pct=round(r["miss_pct"], 2),
                         miss_pct_ci95=round(r["miss_pct_ci95"], 2), units_per_s=round(r["units_per_s"], 1),
                         useful_share_pct=round(r["useful_share"], 1)))
        print(f"{label + ' (붐빔)':<20}{r['crit']:>12}{r['miss_pct']:>9.1f}%{r['units_per_s']:>14.1f}{r['useful_share']:>13.1f}%")
    with (out / "selection_summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)

    # 길가 센서의 짐작이 틀리는 정도에 따라
    err_rows = []
    for e in (0.0, 0.05, 0.10, 0.20, 0.30):
        rr = summarize(range(args.sweep_seeds), p_occ_err=e, p_route_err=e)
        err_rows.append(dict(est_err=e, ask_miss_pct=round(rr["ask"]["miss_pct"], 2),
                             bcast_all_miss_pct=round(rr["bcast_all"]["miss_pct"], 2)))
    print("짐작 오차 →", [(r["est_err"], r["ask_miss_pct"]) for r in err_rows])
    with (out / "selection_error_sweep.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(err_rows[0]))
        w.writeheader(); w.writerows(err_rows)

    sweep = []
    for B in (4, 8, 12, 16, 24):
        for c_bc in (2, 4):
            rr = summarize(range(args.sweep_seeds), B=B, c_bc=c_bc)
            for k, label in POLICIES:
                sweep.append(dict(B=B, c_bc=c_bc, policy=k, label=label, miss_pct=round(rr[k]["miss_pct"], 2),
                                  units_per_s=round(rr[k]["units_per_s"], 1)))
        print("자원", B, [(s["c_bc"], s["policy"], s["miss_pct"]) for s in sweep if s["B"] == B])
    with (out / "selection_sweep.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(sweep[0]))
        w.writeheader(); w.writerows(sweep)

    colors = {"bcast_all": "#9aa3ad", "cpm_like": "#6b7f99", "ask": "#14284b"}
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.9))
    for ax, c_bc in zip(axes, (4, 2)):
        for k, label in POLICIES:
            xs = [s["B"] for s in sweep if s["policy"] == k and s["c_bc"] == c_bc]
            ys = [s["miss_pct"] for s in sweep if s["policy"] == k and s["c_bc"] == c_bc]
            ax.plot(xs, ys, marker="o", color=colors[k], label=label, lw=2 if k == "ask" else 1.3)
        ax.set_xlabel("0.1초마다 쓸 수 있는 무선 자원 [칸]")
        ax.set_ylabel("꼭 받아야 할 정보를 놓친 비율 [%]")
        ax.set_title(f"방송 한 건 = {c_bc}칸" + (" (기본 가정)" if c_bc == PARAMS["c_bc"] else " (방송이 싼 경우)"), fontsize=10.5)
        ax.spines[["top", "right"]].set_visible(False)
    axes[0].legend(frameon=False, fontsize=9)
    fig.suptitle(f"가려진 보행자 정보를 제때 받았는가 (가정값 기준, 시드 {args.sweep_seeds}개)", fontsize=11)
    fig.tight_layout()
    fig.savefig(out / "fig1_selection_sweep.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
