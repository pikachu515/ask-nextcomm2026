"""요청 폭주 시뮬레이션 (개념 검증)

무인 자율주행차 N대가 달리는 도시에서 정전 같은 사건이 터져 도움 요청이
한꺼번에 몰리는 30분을 2초 단위로 흉내 낸다. 통신 물리 계층이나 실제 차량
거동은 다루지 않는다. 입력값은 모두 가정이며 README.md 의 표에 정리돼 있다.

비교 방식 (아래로 갈수록 기능이 하나씩 더해진다)
  human    사람만        관제사가 온 순서대로 처리. 혼잡하면 영상이 느려 처리도 느려진다
  machine  +교차로 응답  확인형 질문은 교차로 장비와 기지국 AI가 먼저 답한다 (ASK 2단계)
  qos      +통신 보장    관제사가 맡은 차량의 영상 품질을 지킨다 (ASK 3단계)
  full     +우선·대피    도로를 막은 차량부터 배정하고, 기한을 넘기면 갓길로 비킨다 (ASK 3·4단계)

실행
  python surge_sim.py              # 시드 100개, 결과는 results/ 에 저장
  python surge_sim.py --seeds 20   # 빠른 확인용
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

# 모든 값은 가정이다. 근거가 생기면 이 표와 README.md 를 함께 고친다.
PARAMS = dict(
    N=200,                  # 운행 차량 수 (광주 실증도시 규모)
    M=5,                    # 관제사 수 (관제사 1명당 차량 40대)
    T_end=5400,             # 전체 시간 [s] (90분)
    ev_start=600,           # 사건 시작 [s]
    ev_end=2400,            # 사건 종료 [s]
    lam_normal=0.5 / 3600,  # 평소 차량 1대의 요청 빈도 [1/s] (2시간에 1번)
    surge=10.0,             # 사건 중 요청 빈도 배수
    p_confirm_normal=0.5,   # 평소 확인형 질문 비율
    p_confirm_event=0.7,    # 사건 중 확인형 질문 비율
    p_block=0.6,            # 요청 차량이 차로를 막고 있을 확률
    svc_confirm=20.0,       # 확인형 질문의 관제사 평균 처리 시간 [s]
    svc_complex=90.0,       # 복잡한 질문의 관제사 평균 처리 시간 [s]
    ai_resolve=0.8,         # 교차로 장비·기지국 AI가 확인형 질문을 해결하는 비율
    ai_time=4.0,            # 교차로 장비·기지국 AI의 응답 시간 [s]
    q_congested=0.5,        # 혼잡 시 영상 품질 비율 (처리 속도가 이만큼 느려진다)
    deadline=60.0,          # 대피 판단 기한 [s]
    p_pull=0.7,             # 대피 성공 확률
)

POLICIES = [
    ("human", "사람만", dict()),
    ("machine", "+교차로 응답", dict(ai=True)),
    ("qos", "+통신 보장", dict(ai=True, qos=True)),
    ("full", "+우선·대피", dict(ai=True, qos=True, prio=True, pull=True)),
]

DT = 2.0  # 시뮬레이션 간격 [s]

# 차량 상태
DRIVING, WAIT_MACHINE, WAIT_HUMAN, IN_SERVICE = 0, 1, 2, 3


def simulate(seed, ai=False, qos=False, prio=False, pull=False, **override):
    """한 번의 90분을 흉내 내고 시간별 기록과 요약 지표를 돌려준다."""
    p = {**PARAMS, **override}
    rng = np.random.default_rng(seed)
    n, m = p["N"], p["M"]
    steps = int(p["T_end"] / DT)

    status = np.zeros(n, dtype=np.int8)
    blocking = np.zeros(n, dtype=bool)   # 차로를 막고 있는가
    t_req = np.zeros(n)                  # 요청 시각
    work = np.zeros(n)                   # 남은 관제사 작업량 [s]
    machine_done = np.zeros(n)
    machine_ok = np.zeros(n, dtype=bool)
    pull_tried = np.zeros(n, dtype=bool)

    n_block = np.zeros(steps)
    n_stall = np.zeros(steps)
    resolved = []

    for i in range(steps):
        t = i * DT
        event = p["ev_start"] <= t < p["ev_end"]

        # 공통 난수: 방식과 상관없이 매 스텝 같은 개수의 난수를 뽑는다.
        # 같은 시드라면 같은 차량이 같은 시각에 같은 운을 만나므로,
        # 방식 간 차이는 '선택'에서만 생긴다.
        u = rng.random((5, n))
        e = rng.exponential(1.0, n)

        # 1) 새 요청: 달리고 있는 차량에서만 생긴다
        lam = p["lam_normal"] * (p["surge"] if event else 1.0)
        new = np.flatnonzero((status == DRIVING) & (u[0] < lam * DT))
        if new.size:
            p_conf = p["p_confirm_event"] if event else p["p_confirm_normal"]
            confirm = u[1][new] < p_conf
            blocking[new] = u[2][new] < p["p_block"]
            t_req[new] = t
            work[new] = e[new] * np.where(confirm, p["svc_confirm"], p["svc_complex"])
            pull_tried[new] = False
            if ai:
                status[new] = np.where(confirm, WAIT_MACHINE, WAIT_HUMAN)
                machine_done[new] = t + p["ai_time"]
                machine_ok[new] = u[3][new] < p["ai_resolve"]
            else:
                status[new] = WAIT_HUMAN

        # 2) 교차로 장비·기지국 AI의 응답: 해결하면 다시 달리고, 못 풀면 사람에게 넘긴다
        if ai:
            done = np.flatnonzero((status == WAIT_MACHINE) & (t >= machine_done))
            if done.size:
                ok = done[machine_ok[done]]
                resolved.extend((t - t_req[ok]).tolist())
                status[ok] = DRIVING
                blocking[ok] = False
                status[done[~machine_ok[done]]] = WAIT_HUMAN

        # 3) 갓길 대피: 기한을 넘긴 차로 점유 차량이 한 번 시도한다
        if pull:
            cand = np.flatnonzero(
                (status == WAIT_HUMAN) & blocking & ~pull_tried & (t - t_req >= p["deadline"])
            )
            if cand.size:
                pull_tried[cand] = True
                blocking[cand[u[4][cand] < p["p_pull"]]] = False

        # 4) 빈 관제사 배정: 온 순서대로, 또는 차로를 막은 차량부터
        free = m - int((status == IN_SERVICE).sum())
        if free > 0:
            waiting = np.flatnonzero(status == WAIT_HUMAN)
            if waiting.size:
                if prio:
                    order = np.lexsort((t_req[waiting], ~blocking[waiting]))
                else:
                    order = np.argsort(t_req[waiting], kind="stable")
                status[waiting[order[:free]]] = IN_SERVICE

        # 5) 관제사 처리: 혼잡하면 영상 품질만큼 느려진다 (통신 보장이면 정상 속도)
        serving = np.flatnonzero(status == IN_SERVICE)
        if serving.size:
            speed = 1.0 if (qos or not event) else p["q_congested"]
            work[serving] -= DT * speed
            finished = serving[work[serving] <= 0]
            if finished.size:
                resolved.extend((t + DT - t_req[finished]).tolist())
                status[finished] = DRIVING
                blocking[finished] = False

        n_stall[i] = (status != DRIVING).sum()
        n_block[i] = (blocking & (status != DRIVING)).sum()

    r = np.array(resolved) if resolved else np.array([0.0])
    return dict(
        n_block=n_block,
        n_stall=n_stall,
        block_veh_min=float(n_block.sum() * DT / 60.0),  # 차로 점유 차량·분
        peak_block=float(n_block.max()),
        peak_stall=float(n_stall.max()),
        p95_resolve_s=float(np.percentile(r, 95)),
    )


def mean_ci(values):
    """평균과 95% 신뢰구간 반폭. 시드가 30개 이상일 때의 근삿값이다."""
    v = np.asarray(values, dtype=float)
    half = 1.984 * v.std(ddof=1) / math.sqrt(v.size) if v.size > 1 else 0.0
    return float(v.mean()), float(half)


def main():
    parser = argparse.ArgumentParser(description="ASK 요청 폭주 시뮬레이션 (개념 검증)")
    parser.add_argument("--seeds", type=int, default=100, help="반복 횟수 (기본 100)")
    parser.add_argument("--sweep-seeds", type=int, default=30, help="민감도 그림의 반복 횟수")
    args = parser.parse_args()

    out = Path(__file__).resolve().parent / "results"
    out.mkdir(exist_ok=True)
    # 한글 글꼴: 설치돼 있는 것 가운데 먼저 찾은 것을 쓴다 (윈도우, 맥, 리눅스 순)
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in ("Malgun Gothic", "AppleGothic", "NanumGothic", "Noto Sans CJK KR"):
        if name in installed:
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False

    seeds = range(args.seeds)
    runs = {key: [simulate(s, **flags) for s in seeds] for key, _, flags in POLICIES}
    base = np.array([r["block_veh_min"] for r in runs["human"]])

    rows = []
    print(f"가정값 기준 개념 검증 · 시드 {args.seeds}개 · 차량 {PARAMS['N']}대 · 관제사 {PARAMS['M']}명")
    print(f"{'방식':<12}{'차로 점유 차량·분':>18}{'사람만 대비 감소':>18}{'동시 정지 최대':>14}{'95% 해결[s]':>12}")
    for key, label, _ in POLICIES:
        block = np.array([r["block_veh_min"] for r in runs[key]])
        b_mean, b_half = mean_ci(block)
        red_mean, red_half = mean_ci(100.0 * (1.0 - block / base))  # 같은 시드끼리 비교
        stall_mean, _ = mean_ci([r["peak_stall"] for r in runs[key]])
        peak_block_mean, _ = mean_ci([r["peak_block"] for r in runs[key]])
        p95_mean, _ = mean_ci([r["p95_resolve_s"] for r in runs[key]])
        rows.append(dict(
            policy=key, label=label, n_seeds=args.seeds,
            block_veh_min_mean=round(b_mean, 1), block_veh_min_ci95=round(b_half, 1),
            reduction_pct_mean=round(red_mean, 1), reduction_pct_ci95=round(red_half, 1),
            peak_stalled_mean=round(stall_mean, 1), peak_blocking_mean=round(peak_block_mean, 1),
            p95_resolve_s_mean=round(p95_mean, 0),
        ))
        print(f"{label:<12}{b_mean:>12.0f} ±{b_half:<5.0f}{red_mean:>12.1f} ±{red_half:<4.1f}"
              f"{stall_mean:>12.1f}{p95_mean:>12.0f}")

    with (out / "surge_summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    # 그림 1: 시간에 따른 차로 점유 차량 수
    fig, ax = plt.subplots(figsize=(8, 4.2))
    minutes = np.arange(int(PARAMS["T_end"] / DT)) * DT / 60.0
    for key, label, _ in POLICIES:
        ax.plot(minutes, np.mean([r["n_block"] for r in runs[key]], axis=0), label=label, linewidth=2)
    ax.axvspan(PARAMS["ev_start"] / 60, PARAMS["ev_end"] / 60, color="0.9", zorder=0)
    ax.text((PARAMS["ev_start"] + PARAMS["ev_end"]) / 120, ax.get_ylim()[1] * 0.95,
            "사건 구간 (요청 10배)", ha="center", va="top", color="0.4")
    ax.set_xlabel("시간 [분]")
    ax.set_ylabel("차로를 막고 선 차량 수 [대]")
    ax.set_title(f"요청이 몰릴 때 차로를 막고 선 차량 수 (가정값 기준, 시드 {args.seeds}개 평균)")
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out / "fig1_blocking_over_time.png", dpi=150)
    plt.close(fig)

    # 그림 2: 교차로 장비·기지국 AI의 해결 비율에 따른 총 점유 시간
    ratios = [0.0, 0.2, 0.4, 0.6, 0.8, 0.95]
    sweep_seeds = range(args.sweep_seeds)
    fig, ax = plt.subplots(figsize=(8, 4.2))
    for key in ("machine", "full"):
        label = next(lb for k, lb, _ in POLICIES if k == key)
        flags = next(fl for k, _, fl in POLICIES if k == key)
        ys = [np.mean([simulate(s, **flags, ai_resolve=x)["block_veh_min"] for s in sweep_seeds])
              for x in ratios]
        ax.plot(ratios, ys, marker="o", linewidth=2, label=label)
    ax.axhline(base.mean(), color="0.5", linestyle="--", label="사람만")
    ax.set_xlabel("교차로 장비·기지국 AI가 확인형 질문을 해결하는 비율")
    ax.set_ylabel("차로 점유 차량·분")
    ax.set_title("기계가 답하는 비율과 차로 점유 시간 (가정값 기준)")
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out / "fig2_resolve_ratio_sweep.png", dpi=150)
    plt.close(fig)

    print(f"\n저장: {out / 'surge_summary.csv'}, fig1_blocking_over_time.png, fig2_resolve_ratio_sweep.png")


if __name__ == "__main__":
    main()
