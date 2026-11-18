"""마감시각 순서 모의실험 (개념 검증)

교차로 몇 곳을 맡는 기지국 하나와 그 옆 AI 서버 하나를 흉내 낸다.
차가 사진을 올려 "지나가도 되나요?"를 물으면 요청은 두 줄을 차례로 지난다.

  ① 올려 보내는 통신 (모든 차가 나눠 쓰는 무선 구간)
  ② 기지국 옆 AI 서버의 처리 줄
  ③ 짧은 답을 내려 보냄

차마다 답이 꼭 와야 하는 시각(마감시각)이 다르다. 속도와 정지선까지 거리로 계산한다.
  d_stop = v·τ + v²/(2a) + m
  D_net  = max(0, (L - d_stop)/v - T_verify)
마감 안에 답이 오지 않으면 차는 정지선 앞에 멈춰야 한다.

비교 방식
  fifo        통신도 AI도 온 순서대로
  edf_net     통신만 마감 순서 (AI는 온 순서)
  edf_ai      AI만 마감 순서 (통신은 온 순서)
  edf_both    통신과 AI 모두 같은 마감 순서 (ASK)
  edf_admit   edf_both + 마감 안에 못 끝날 요청은 줄에 넣지 않고 바로 "확인 불가"로 돌려보냄 (ASK)

실행
  python deadline_sim.py              # 시드 100개
  python deadline_sim.py --seeds 20   # 빠른 확인
"""
from __future__ import annotations

import argparse
import csv
import heapq
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
    T_sim=300.0,          # 한 번 실험 길이 [s]
    lam=11.0,             # 요청 도착률 [건/s] (AI 서버 이용률 약 0.88)
    v_kmh=(30.0, 60.0),   # 차 속도 범위 [km/h]
    L=(40.0, 140.0),      # 물을 때 정지선까지 거리 범위 [m]
    tau=0.3,              # 반응·작동 지연 [s]
    a=2.0,                # 부드러운 감속도 [m/s²]
    a_acc=1.5,            # 다시 출발할 때 가속도 [m/s²]
    margin=2.0,           # 안전 여유 거리 [m]
    t_verify=0.05,        # 차가 답을 확인하는 시간 [s]
    size_kb=(100.0, 300.0),   # 올리는 사진 크기 범위 [KB]
    ul_mbps=22.0,         # 혼잡할 때 이 요청들에 돌아가는 올림 속도 [Mbit/s] (통신 이용률 약 0.91)
    air=0.010,            # 무선 구간 고정 지연 [s]
    ai_mean=0.080,        # AI 처리 시간 평균 [s] (로그정규)
    ai_cv=0.5,            # AI 처리 시간 변동계수
    dl=0.010,             # 답을 내려 보내는 시간 [s]
    own_check=1.0,        # "확인 불가"를 받고 멈춘 차가 스스로 확인하는 시간 [s]
)

POLICIES = [
    ("fifo", "온 순서대로", dict(net="fifo", ai="fifo", admit=False)),
    ("edf_net", "통신만 마감 순서", dict(net="edf", ai="fifo", admit=False)),
    ("edf_ai", "AI만 마감 순서", dict(net="fifo", ai="edf", admit=False)),
    ("edf_both", "둘 다 마감 순서", dict(net="edf", ai="edf", admit=False)),
    ("edf_admit", "둘 다 + 미리 거르기", dict(net="edf", ai="edf", admit=True)),
]


def make_requests(seed, p):
    """같은 시드면 모든 방식이 같은 요청을 받는다."""
    rng = np.random.default_rng(seed)
    t, reqs = 0.0, []
    while True:
        t += rng.exponential(1.0 / p["lam"])
        if t > p["T_sim"]:
            break
        v = rng.uniform(*p["v_kmh"]) / 3.6
        L = rng.uniform(*p["L"])
        d_stop = v * p["tau"] + v * v / (2 * p["a"]) + p["margin"]
        D = max(0.0, (L - d_stop) / v - p["t_verify"])
        s_ul = p["air"] + rng.uniform(*p["size_kb"]) * 8 / 1000 / p["ul_mbps"]
        sig = math.sqrt(math.log(1 + p["ai_cv"] ** 2))
        s_ai = rng.lognormal(math.log(p["ai_mean"]) - sig * sig / 2, sig)
        reqs.append(dict(t=t, v=v, D=D, dl=t + D, s_ul=s_ul, s_ai=s_ai))
    return reqs


def run(reqs, p, net, ai, admit):
    """두 줄(통신, AI)을 차례로 지나는 사건 기반 계산. 각 줄은 서버 하나, 처리 중에는 끼어들지 않는다."""
    mean_ai = p["ai_mean"]
    key = {"fifo": lambda r: r["t"], "edf": lambda r: r["dl"]}
    events = [(r["t"], 0, i) for i, r in enumerate(reqs)]   # (시각, 단계, 요청 번호)
    heapq.heapify(events)
    q = {1: [], 2: []}           # 단계별 기다리는 줄
    busy = {1: False, 2: False}
    seq = 0
    done = {}                    # 요청 번호 → (답이 도착한 시각, 거절 여부)

    def start(stage, now):
        nonlocal seq
        while q[stage] and not busy[stage]:
            _, _, i = heapq.heappop(q[stage])
            r = reqs[i]
            if admit:
                rest = (r["s_ul"] if stage == 1 else 0.0) + mean_ai + p["dl"]
                # 지금 시작해도 차가 다 서기 전에 답이 못 간다 → 어차피 설 차이므로 바로 "확인 불가".
                # 조금만 늦을 요청은 거르지 않는다 (감속 중에 답이 오면 차는 다시 간다)
                if now + rest > r["dl"] + r["v"] / p["a"]:
                    done[i] = (now + p["dl"], True)
                    continue
            busy[stage] = True
            s = r["s_ul"] if stage == 1 else r["s_ai"]
            seq += 1
            heapq.heappush(events, (now + s, stage, i))

    def enqueue(stage, i):
        nonlocal seq
        pol = net if stage == 1 else ai
        seq += 1
        heapq.heappush(q[stage], (key[pol](reqs[i]), seq, i))

    while events:
        now, stage, i = heapq.heappop(events)
        if stage == 0:            # 도착
            enqueue(1, i)
            start(1, now)
        elif stage == 1:          # 통신 끝 → AI 줄로
            busy[1] = False
            enqueue(2, i)
            start(1, now)
            start(2, now)
        else:                     # AI 끝 → 답을 내려 보냄
            busy[2] = False
            done[i] = (now + p["dl"], False)
            start(2, now)
    return done


def evaluate(reqs, done, p):
    """마감 안에 답을 받은 비율과, 멈춰서 늘어난 통과 시간을 계산한다."""
    n = len(reqs)
    feas = [i for i, r in enumerate(reqs) if r["D"] > 0.1]   # 처음부터 마감이 거의 없는 차는 따로 센다
    on_time = late = refused = full = 0
    lost = []
    for i in feas:
        r = reqs[i]
        t_ans, ref = done[i]
        v = r["v"]
        if not ref and t_ans <= r["dl"]:
            on_time += 1
            lost.append(0.0)
            continue
        # 마감 지점(r["dl"])에서 브레이크를 밟기 시작한다. 다 서기 전에 답이 오면 그때 다시 가속한다
        t_brake = r["dl"]
        t_full = v / p["a"]                                      # 완전히 서기까지 걸리는 시간
        stop_cost = v / (2 * p["a"]) + v / (2 * p["a_acc"])     # 완전히 멈췄다 다시 출발하며 잃는 시간
        if ref:
            refused += 1
            full += 1
            lost.append(stop_cost + p["own_check"])              # 서서 스스로 확인하고 출발
            continue
        late += 1
        dt_b = t_ans - t_brake
        if dt_b < t_full:                                        # 감속하던 중에 답이 옴
            dv = p["a"] * dt_b
            lost.append((p["a"] * dt_b * dt_b / 2 + dv * dv / (2 * p["a_acc"])) / v)
        else:                                                    # 정지선 앞에 완전히 섰다가 답을 기다림
            full += 1
            lost.append(stop_cost + (dt_b - t_full))
    nf = max(1, len(feas))
    resp = [done[i][0] - reqs[i]["t"] for i in feas if not done[i][1]]
    return dict(
        n=n, n_feasible=len(feas),
        on_time_pct=100 * on_time / nf, late_pct=100 * late / nf, refused_pct=100 * refused / nf,
        stopped_pct=100 * full / nf,          # 정지선 앞에 완전히 선 차
        slowed_pct=100 * (late + refused) / nf,   # 마감을 넘겨 감속한 차 (완전히 선 차 포함)
        lost_s_mean=float(np.mean(lost)) if lost else 0.0,
        resp_p50=float(np.percentile(resp, 50)) if resp else 0.0,
        resp_p99=float(np.percentile(resp, 99)) if resp else 0.0,
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


def run_all(seeds, **override):
    p = {**PARAMS, **override}
    res = {k: [] for k, _, _ in POLICIES}
    for s in seeds:
        reqs = make_requests(s, p)
        for k, _, fl in POLICIES:
            res[k].append(evaluate(reqs, run(reqs, p, **fl), p))
    return res


def main():
    ap = argparse.ArgumentParser(description="마감시각 순서 모의실험")
    ap.add_argument("--seeds", type=int, default=100)
    ap.add_argument("--sweep-seeds", type=int, default=30)
    args = ap.parse_args()
    out = HERE / "results"
    out.mkdir(exist_ok=True)
    set_font()

    res = run_all(range(args.seeds))
    rho = PARAMS["lam"] * PARAMS["ai_mean"]
    print(f"가정값 기준 · 시드 {args.seeds}개 · 요청 {PARAMS['lam']}건/s · AI 서버 이용률 약 {rho:.2f}")
    print(f"{'방식':<16}{'제때 답':>9}{'늦은 답':>9}{'미리 거름':>9}{'완전히 선 차':>9}{'잃은 시간[s]':>12}{'응답 P99[s]':>12}")
    rows = []
    for k, label, _ in POLICIES:
        r = res[k]
        row = dict(policy=k, label=label, n_seeds=args.seeds, lam=PARAMS["lam"], ai_util=round(rho, 2))
        for m in ("on_time_pct", "late_pct", "refused_pct", "slowed_pct", "stopped_pct", "lost_s_mean", "resp_p50", "resp_p99"):
            mu, h = mean_ci([x[m] for x in r])
            row[m] = round(mu, 3 if m.startswith("resp") else 2)
            row[m + "_ci95"] = round(h, 3 if m.startswith("resp") else 2)
        rows.append(row)
        print(f"{label:<16}{row['on_time_pct']:>8.1f}%{row['late_pct']:>8.1f}%{row['refused_pct']:>8.1f}%"
              f"{row['stopped_pct']:>8.1f}%{row['lost_s_mean']:>12.2f}{row['resp_p99']:>12.2f}")
    with (out / "deadline_summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)

    # 부하 바꿔 보기
    lams = [6.0, 8.0, 10.0, 11.0, 12.0, 13.0, 14.0]
    sweep = []
    for lam in lams:
        rr = run_all(range(args.sweep_seeds), lam=lam)
        for k, label, _ in POLICIES:
            sweep.append(dict(lam=lam, ai_util=round(lam * PARAMS["ai_mean"], 2), policy=k, label=label,
                              on_time_pct=round(float(np.mean([x["on_time_pct"] for x in rr[k]])), 2),
                              late_pct=round(float(np.mean([x["late_pct"] for x in rr[k]])), 2),
                              lost_s_mean=round(float(np.mean([x["lost_s_mean"] for x in rr[k]])), 3)))
        print("부하", lam, [(s["policy"], s["on_time_pct"]) for s in sweep if s["lam"] == lam])
    with (out / "deadline_load_sweep.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(sweep[0]))
        w.writeheader(); w.writerows(sweep)

    colors = {"fifo": "#9aa3ad", "edf_net": "#b9c3cf", "edf_ai": "#7d8fa6", "edf_both": "#3f6c9b", "edf_admit": "#14284b"}
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.9))
    labels = [lb.replace(" 마감 순서", "\n마감 순서").replace(" + ", "\n+ ") for _, lb, _ in POLICIES]
    on = [r["on_time_pct"] for r in rows]
    lost = [r["lost_s_mean"] for r in rows]
    ax = axes[0]
    bars = ax.bar(labels, on, color=[colors[k] for k, _, _ in POLICIES], width=0.62)
    for b, v in zip(bars, on):
        ax.text(b.get_x() + b.get_width() / 2, v * 1.01, f"{v:.1f}%", ha="center", fontsize=9.5)
    ax.set_ylim(0, 108)
    ax.set_title(f"마감 안에 답을 받은 비율 (이용률 {rho:.2f}, 시드 {args.seeds}개)", fontsize=10.5)
    ax.tick_params(axis="x", labelsize=8.5)
    ax.spines[["top", "right"]].set_visible(False)
    ax = axes[1]
    for k, label, _ in POLICIES:
        if k not in ("fifo", "edf_both", "edf_admit"):
            continue
        xs = [s["ai_util"] for s in sweep if s["policy"] == k and s["ai_util"] >= 0.8]
        ys = [s["lost_s_mean"] for s in sweep if s["policy"] == k and s["ai_util"] >= 0.8]
        ax.plot(xs, ys, marker="o", color=colors[k], label=label, lw=2 if k == "edf_admit" else 1.4)
    ax.set_xlabel("AI 서버 이용률")
    ax.set_title(f"늦은 답으로 차 한 대가 잃은 시간 [초] (시드 {args.sweep_seeds}개)", fontsize=10.5)
    ax.legend(frameon=False, fontsize=9)
    ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle("통신과 AI를 같은 마감 순서로 (가정값 기준)", fontsize=11)
    fig.tight_layout()
    fig.savefig(out / "fig1_deadline_compare.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    for k, label, _ in POLICIES:
        xs = [s["ai_util"] for s in sweep if s["policy"] == k]
        ys = [s["on_time_pct"] for s in sweep if s["policy"] == k]
        ax.plot(xs, ys, marker="o", color=colors[k], label=label, lw=2 if k in ("edf_both", "edf_admit") else 1.3)
    ax.set_xlabel("AI 서버 이용률 (1을 넘으면 처리량보다 요청이 많음)")
    ax.set_ylabel("마감 안에 답을 받은 비율 [%]")
    ax.set_title(f"부하가 클수록 차이가 커집니다 (가정값 기준, 시드 {args.sweep_seeds}개)", fontsize=10.5)
    ax.legend(frameon=False, fontsize=8.5)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(out / "fig2_load_sweep.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
