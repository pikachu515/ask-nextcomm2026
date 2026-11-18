"""정전 시 사전 알림 모의실험 (개념 검증)

요청 폭주 모의실험(sim/surge)과 같은 도시, 같은 30분, 같은 난수를 쓴다.
다른 점은 하나다. 비상 전원이 있는 교차로가 "신호 꺼짐"을 먼저 알리면,
그 알림을 받은 차의 확인형 질문 일부가 아예 생기지 않는다.

비교 방식
  human    원격 관제사만    요청 폭주 모의실험과 같다
  machine  +교차로 응답     요청 폭주 모의실험과 같다. 확인형 질문은 교차로와 기지국 AI가 먼저 답한다
  notify   +먼저 알림       machine에 먼저 알리기를 더한다

질문
  1) 원격 관제사에게 가는 확인 요청이 얼마나 주는가
  2) 차로를 막은 시간이 얼마나 주는가
  3) 비상 전원이 있는 교차로 비율에 따라 효과가 어떻게 바뀌는가

실행
  python blackout_sim.py              # 시드 100개
  python blackout_sim.py --seeds 20   # 빠른 확인
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "surge"))
from surge_sim import PARAMS as SURGE_PARAMS  # noqa: E402  같은 도시 조건을 그대로 쓴다

# 먼저 알리기에 관한 가정값. README.md 의 표와 같아야 한다.
NOTIFY = dict(
    p_power=0.5,    # 비상 전원이 있어 정전 중에도 알림을 보낼 수 있는 교차로 비율
    p_reach=0.9,    # 알림이 차에 닿는 비율 (방송 수신 실패 10%)
    p_settle=0.7,   # 알림을 받은 차의 확인형 질문 가운데 알림만으로 풀리는 비율
)
PARAMS = {**SURGE_PARAMS, **NOTIFY}

POLICIES = [
    ("human", "원격 관제사만", dict()),
    ("machine", "+교차로 응답", dict(ai=True)),
    ("notify", "+먼저 알림", dict(ai=True, notify=True)),
]

DT = 2.0
DRIVING, WAIT_MACHINE, WAIT_HUMAN, IN_SERVICE = 0, 1, 2, 3


def simulate(seed, ai=False, notify=False, **override):
    p = {**PARAMS, **override}
    rng = np.random.default_rng(seed)               # surge_sim 과 같은 난수 흐름
    rng_n = np.random.default_rng(seed + 100_000)   # 먼저 알리기에만 쓰는 난수 (다른 방식의 운을 바꾸지 않는다)
    n, m = p["N"], p["M"]
    steps = int(p["T_end"] / DT)

    status = np.zeros(n, dtype=np.int8)
    blocking = np.zeros(n, dtype=bool)
    t_req = np.zeros(n)
    work = np.zeros(n)
    machine_done = np.zeros(n)
    machine_ok = np.zeros(n, dtype=bool)

    n_block = np.zeros(steps)
    asked = to_machine = to_human = settled_by_notice = 0

    for i in range(steps):
        t = i * DT
        event = p["ev_start"] <= t < p["ev_end"]
        u = rng.random((5, n))
        e = rng.exponential(1.0, n)
        un = rng_n.random(n)

        lam = p["lam_normal"] * (p["surge"] if event else 1.0)
        new = np.flatnonzero((status == DRIVING) & (u[0] < lam * DT))
        if new.size:
            p_conf = p["p_confirm_event"] if event else p["p_confirm_normal"]
            confirm = u[1][new] < p_conf
            if notify and event:
                # 알림이 이미 답한 질문은 보내지 않는다. 차는 멈추지 않고 스스로 지나간다
                quiet = confirm & (un[new] < p["p_power"] * p["p_reach"] * p["p_settle"])
                settled_by_notice += int(quiet.sum())
                new, confirm = new[~quiet], confirm[~quiet]
            asked += new.size
            blocking[new] = u[2][new] < p["p_block"]
            t_req[new] = t
            work[new] = e[new] * np.where(confirm, p["svc_confirm"], p["svc_complex"])
            if ai:
                status[new] = np.where(confirm, WAIT_MACHINE, WAIT_HUMAN)
                machine_done[new] = t + p["ai_time"]
                machine_ok[new] = u[3][new] < p["ai_resolve"]
                to_machine += int(confirm.sum())
                to_human += int((~confirm).sum())
            else:
                status[new] = WAIT_HUMAN
                to_human += new.size

        if ai:
            done = np.flatnonzero((status == WAIT_MACHINE) & (t >= machine_done))
            if done.size:
                ok = done[machine_ok[done]]
                status[ok] = DRIVING
                blocking[ok] = False
                fail = done[~machine_ok[done]]
                status[fail] = WAIT_HUMAN
                to_human += fail.size

        free = m - int((status == IN_SERVICE).sum())
        if free > 0:
            waiting = np.flatnonzero(status == WAIT_HUMAN)
            if waiting.size:
                order = np.argsort(t_req[waiting], kind="stable")
                status[waiting[order[:free]]] = IN_SERVICE

        serving = np.flatnonzero(status == IN_SERVICE)
        if serving.size:
            speed = 1.0 if not event else p["q_congested"]
            work[serving] -= DT * speed
            finished = serving[work[serving] <= 0]
            status[finished] = DRIVING
            blocking[finished] = False

        n_block[i] = (blocking & (status != DRIVING)).sum()

    return dict(
        n_block=n_block,
        block_veh_min=float(n_block.sum() * DT / 60.0),
        asked=asked, to_machine=to_machine, to_human=to_human, settled=settled_by_notice,
    )


def mean_ci(values):
    v = np.asarray(values, dtype=float)
    half = 1.984 * v.std(ddof=1) / math.sqrt(v.size) if v.size > 1 else 0.0
    return float(v.mean()), float(half)


def set_font():
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in ("Malgun Gothic", "AppleGothic", "NanumGothic", "Noto Sans CJK KR"):
        if name in installed:
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False


def main():
    ap = argparse.ArgumentParser(description="정전 시 사전 알림 모의실험")
    ap.add_argument("--seeds", type=int, default=100)
    ap.add_argument("--sweep-seeds", type=int, default=30)
    args = ap.parse_args()
    out = HERE / "results"
    out.mkdir(exist_ok=True)
    set_font()

    seeds = range(args.seeds)
    runs = {k: [simulate(s, **fl) for s in seeds] for k, _, fl in POLICIES}
    base_h = np.array([r["to_human"] for r in runs["human"]], float)
    base_b = np.array([r["block_veh_min"] for r in runs["human"]])
    mach_h = np.array([r["to_human"] for r in runs["machine"]], float)
    mach_b = np.array([r["block_veh_min"] for r in runs["machine"]])

    rows = []
    print(f"가정값 기준 · 시드 {args.seeds}개 · 비상 전원 교차로 {PARAMS['p_power']:.0%}")
    print(f"{'방식':<10}{'질문 수':>8}{'관제사에게 간 요청':>14}{'관제사만 대비':>10}{'차로 점유 차량·분':>16}{'관제사만 대비':>10}{'+교차로 응답 대비':>14}")
    for k, label, _ in POLICIES:
        h = np.array([r["to_human"] for r in runs[k]], float)
        b = np.array([r["block_veh_min"] for r in runs[k]])
        row = dict(
            policy=k, label=label, n_seeds=args.seeds,
            asked_mean=round(mean_ci([r["asked"] for r in runs[k]])[0], 1),
            settled_by_notice_mean=round(mean_ci([r["settled"] for r in runs[k]])[0], 1),
            to_human_mean=round(mean_ci(h)[0], 1), to_human_ci95=round(mean_ci(h)[1], 1),
            to_human_red_vs_human_pct=round(mean_ci(100 * (1 - h / base_h))[0], 1),
            to_human_red_vs_machine_pct=round(mean_ci(100 * (1 - h / mach_h))[0], 1),
            block_veh_min_mean=round(mean_ci(b)[0], 1), block_veh_min_ci95=round(mean_ci(b)[1], 1),
            block_red_vs_human_pct=round(mean_ci(100 * (1 - b / base_b))[0], 1),
            block_red_vs_machine_pct=round(mean_ci(100 * (1 - b / mach_b))[0], 1),
        )
        rows.append(row)
        print(f"{label:<10}{row['asked_mean']:>8.0f}{row['to_human_mean']:>14.0f}{row['to_human_red_vs_human_pct']:>9.1f}%"
              f"{row['block_veh_min_mean']:>16.0f}{row['block_red_vs_human_pct']:>9.1f}%{row['block_red_vs_machine_pct']:>13.1f}%")

    with (out / "blackout_summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)

    # 비상 전원 비율 바꿔 보기
    powers = [0.0, 0.25, 0.5, 0.75, 1.0]
    sw = range(args.sweep_seeds)
    sweep = []
    mach_sw = float(np.mean([simulate(s, ai=True)["block_veh_min"] for s in sw]))  # 같은 시드의 비교 기준
    for pw in powers:
        rs = [simulate(s, ai=True, notify=True, p_power=pw) for s in sw]
        sweep.append(dict(p_power=pw,
                          asked_mean=round(float(np.mean([r["asked"] for r in rs])), 1),
                          to_human_mean=round(float(np.mean([r["to_human"] for r in rs])), 1),
                          block_veh_min_mean=round(float(np.mean([r["block_veh_min"] for r in rs])), 1),
                          block_red_vs_machine_pct=round(100 * (1 - float(np.mean([r["block_veh_min"] for r in rs])) / mach_sw), 1)))
    with (out / "blackout_power_sweep.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(sweep[0]))
        w.writeheader(); w.writerows(sweep)
    print("비상 전원 비율 →", [(s["p_power"], s["to_human_mean"], s["block_veh_min_mean"]) for s in sweep])

    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8))
    labels = [lb for _, lb, _ in POLICIES]
    colors = ["#9aa3ad", "#6b7f99", "#14284b"]
    vals_h = [r["to_human_mean"] for r in rows]
    vals_b = [r["block_veh_min_mean"] for r in rows]
    for ax, vals, title in ((axes[0], vals_h, "원격 관제사에게 간 요청 수 (90분)"), (axes[1], vals_b, "차로 점유 차량·분")):
        bars = ax.bar(labels, vals, color=colors, width=0.6)
        for bb, v in zip(bars, vals):
            ax.text(bb.get_x() + bb.get_width() / 2, v * 1.02, f"{v:,.0f}", ha="center", fontsize=10)
        ax.set_title(title, fontsize=11)
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle(f"정전 시 사전 알림 (비상 전원 교차로 {PARAMS['p_power']:.0%}, 가정값 기준, 시드 {args.seeds}개 평균)", fontsize=11)
    fig.tight_layout()
    fig.savefig(out / "fig1_blackout_compare.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.plot([s["p_power"] * 100 for s in sweep], [s["block_veh_min_mean"] for s in sweep], marker="o", color="#14284b", label="+먼저 알림")
    ax.axhline(mach_sw, color="#6b7f99", ls="--", label="+교차로 응답 (알림 없음)")
    ax.set_xlabel("비상 전원이 있는 교차로 비율 [%]")
    ax.set_ylabel("차로 점유 차량·분")
    ax.set_title(f"비상 전원이 많을수록 효과가 커집니다 (가정값 기준, 시드 {args.sweep_seeds}개)", fontsize=10.5)
    ax.legend(frameon=False)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(out / "fig2_power_sweep.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
