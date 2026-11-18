"""SUMO 교차로 교통 영향 모의실험 (개념 검증)

교차로 하나에 남북으로 오가는 차를 흘려보낸다(한 방향 한 차로).
일부 차는 정지선 앞에서 "지나가도 되나요?"를 묻는다. 답이 오는 시간은
마감시각 모의실험(sim/deadline)의 방식별 결과에서 마감이 비슷한 요청을 골라 쓴다.

  - 마감 안에 답이 오면 차는 그대로 지나간다.
  - 늦으면 정지선 앞에 선다. 답이 오면 다시 출발한다.
  - "확인 불가"로 미리 걸러지면 정지선 앞에 서서 스스로 1초 확인하고 출발한다.

묻지 않은 차도 앞차가 서면 뒤에서 함께 선다. 이것을 SUMO로 본다.

실행
  python crossing_sumo.py              # 시드 20개
  python crossing_sumo.py --seeds 5    # 빠른 확인
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import sumo
import traci
from matplotlib import font_manager

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "deadline"))
import deadline_sim as dl  # noqa: E402

BIN = Path(sumo.SUMO_HOME) / "bin"
NET = HERE / "net"

# 모두 가정값이다. README.md 의 표와 같아야 한다.
PARAMS = dict(
    T=600,              # 차를 들여보내는 시간 [s]
    flow=600,           # 방향마다 시간당 차량 수
    p_ask=0.3,          # 정지선 앞에서 묻는 차의 비율
    L=(60.0, 140.0),    # 물을 때 정지선까지 거리 [m]
    own_check=1.0,      # "확인 불가"를 받고 선 차가 스스로 확인하는 시간 [s]
    pool_seeds=20,      # 답 지연을 가져올 마감시각 모의실험의 시드 수
    loads=(12.0, 13.0), # 마감시각 모의실험의 요청 도착률 [건/s]. AI 서버 이용률 0.96, 1.04
)

POLICIES = [("ideal", "기다림 없음 (기준)")] + [(k, lb) for k, lb, _ in dl.POLICIES]


def build_net():
    NET.mkdir(exist_ok=True)
    (NET / "n.nod.xml").write_text("""<nodes>
  <node id="C" x="0" y="0" type="priority"/>
  <node id="N" x="0" y="400" type="priority"/>
  <node id="S" x="0" y="-400" type="priority"/>
  <node id="E" x="200" y="0" type="priority"/>
  <node id="W" x="-200" y="0" type="priority"/>
</nodes>""", encoding="utf-8")
    (NET / "n.edg.xml").write_text("""<edges>
  <edge id="NC" from="N" to="C" numLanes="1" speed="13.89" priority="3"/>
  <edge id="CS" from="C" to="S" numLanes="1" speed="13.89" priority="3"/>
  <edge id="SC" from="S" to="C" numLanes="1" speed="13.89" priority="3"/>
  <edge id="CN" from="C" to="N" numLanes="1" speed="13.89" priority="3"/>
  <edge id="EC" from="E" to="C" numLanes="1" speed="13.89" priority="1"/>
  <edge id="CW" from="C" to="W" numLanes="1" speed="13.89" priority="1"/>
</edges>""", encoding="utf-8")
    subprocess.run([str(BIN / "netconvert"), "-n", str(NET / "n.nod.xml"), "-e", str(NET / "n.edg.xml"),
                    "-o", str(NET / "cross.net.xml"), "--no-turnarounds", "true", "--no-warnings", "true"],
                   check=True, capture_output=True)


def write_routes(seed, p):
    rng = np.random.default_rng(seed)
    lines = ['<routes>',
             '  <vType id="av" accel="1.5" decel="2.0" emergencyDecel="6.0" sigma="0.3" length="4.8" minGap="2.5" '
             'speedFactor="normc(1,0.08,0.8,1.2)"/>',
             '  <route id="ns" edges="NC CS"/>', '  <route id="sn" edges="SC CN"/>']
    veh = []
    for r in ("ns", "sn"):
        t = 0.0
        while True:
            t += rng.exponential(3600.0 / p["flow"])
            if t > p["T"]:
                break
            veh.append((t, r))
    veh.sort()
    for i, (t, r) in enumerate(veh):
        lines.append(f'  <vehicle id="v{i}" type="av" route="{r}" depart="{t:.2f}" departSpeed="max" departLane="best"/>')
    lines.append('</routes>')
    path = NET / f"r{seed}.rou.xml"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def response_pool(p, lam):
    """마감시각 모의실험에서 방식마다 (마감, 응답 시간, 거절 여부)를 모은다."""
    q = {**dl.PARAMS, "lam": lam}
    pools = {k: [] for k, _, _ in dl.POLICIES}
    for s in range(p["pool_seeds"]):
        reqs = dl.make_requests(10_000 + s, q)
        for k, _, fl in dl.POLICIES:
            done = dl.run(reqs, q, **fl)
            for i, r in enumerate(reqs):
                if r["D"] > 0.1:
                    t_ans, ref = done[i]
                    pools[k].append((r["D"], t_ans - r["t"], ref))
    out = {}
    for k, rows in pools.items():
        a = np.array(rows, float)
        a = a[np.argsort(a[:, 0])]
        out[k] = a
    return out


def deadline_of(v, L):
    q = dl.PARAMS
    d_stop = v * q["tau"] + v * v / (2 * q["a"]) + q["margin"]
    return max(0.0, (L - d_stop) / v - q["t_verify"])


def run(seed, policy, pool, p):
    rou = write_routes(seed, p)
    trip = NET / f"trip_{policy}_{seed}.xml"
    traci.start([str(BIN / "sumo"), "-n", str(NET / "cross.net.xml"), "-r", str(rou), "--seed", str(seed),
                 "--tripinfo-output", str(trip), "--no-step-log", "true", "--no-warnings", "true",
                 "--step-length", "0.1", "--time-to-teleport", "-1"])
    rng = np.random.default_rng(seed + 900)
    ask_L, asked, ctrl, stop_set = {}, set(), {}, set()
    n_ask = n_stop = 0
    max_queue = 0
    lane_len = traci.lane.getLength("NC_0")
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        now = traci.simulation.getTime()
        for vid in traci.simulation.getDepartedIDList():
            if rng.random() < p["p_ask"]:
                ask_L[vid] = rng.uniform(*p["L"])
        for vid in list(ask_L):
            if vid in asked:
                continue
            try:
                road = traci.vehicle.getRoadID(vid)
            except traci.TraCIException:
                ask_L.pop(vid); continue
            if road not in ("NC", "SC"):
                continue
            dist = lane_len - traci.vehicle.getLanePosition(vid)
            if dist > ask_L[vid]:
                continue
            asked.add(vid); n_ask += 1
            if policy == "ideal":
                continue
            v = max(traci.vehicle.getSpeed(vid), 1.0)
            D = deadline_of(v, dist)
            a = pool[policy]
            j = min(int(np.searchsorted(a[:, 0], D)), len(a) - 1)
            lo, hi = max(0, j - 20), min(len(a), j + 20)        # 마감이 비슷한 요청 가운데 하나
            _, resp, ref = a[rng.integers(lo, hi)]
            on_time = (not ref) and resp <= D
            if on_time:
                continue
            # 늦거나 거절되면, 마감 지점(지금부터 D초 뒤)에서 브레이크를 밟기 시작한다
            stop_set.add(vid)
            ctrl[vid] = dict(brake=now + D, answer=now + resp, refused=bool(ref), state="wait", stopped_at=None)
        # 제어: 마감 지점에서 감속 → 정지 유지 → 답이 오면(거절이면 스스로 확인 뒤) 출발
        for vid in list(ctrl):
            c = ctrl[vid]
            try:
                if c["state"] == "wait":
                    if c["answer"] <= c["brake"] and not c["refused"]:
                        ctrl.pop(vid); stop_set.discard(vid); continue      # 브레이크 전에 답이 옴
                    if now >= c["brake"]:
                        v = traci.vehicle.getSpeed(vid)
                        traci.vehicle.slowDown(vid, 0.0, max(0.1, v / dl.PARAMS["a"]))
                        c["state"] = "braking"; n_stop += 1
                elif c["state"] == "braking":
                    if traci.vehicle.getSpeed(vid) < 0.1:
                        traci.vehicle.setSpeed(vid, 0.0)
                        c["state"] = "held"; c["stopped_at"] = now
                    elif not c["refused"] and now >= c["answer"]:
                        traci.vehicle.setSpeed(vid, -1); ctrl.pop(vid); continue   # 서기 전에 답이 옴
                    else:
                        traci.vehicle.setSpeed(vid, max(0.0, traci.vehicle.getSpeed(vid) - dl.PARAMS["a"] * 0.1))
                if c["state"] == "held":
                    go = now >= (c["stopped_at"] + p["own_check"] if c["refused"] else c["answer"])
                    if go:
                        traci.vehicle.setSpeed(vid, -1); ctrl.pop(vid)
            except traci.TraCIException:
                ctrl.pop(vid, None)
        q = traci.edge.getLastStepHaltingNumber("NC") + traci.edge.getLastStepHaltingNumber("SC")
        max_queue = max(max_queue, q)
    traci.close()

    tl, tl_other, n = [], [], 0
    for t in ET.parse(trip).getroot().iter("tripinfo"):
        loss = float(t.get("timeLoss"))
        tl.append(loss)
        if t.get("id") not in stop_set:
            tl_other.append(loss)
    trip.unlink(missing_ok=True)
    rou.unlink(missing_ok=True)
    return dict(n=len(tl), n_ask=n_ask, n_stop=n_stop, time_loss=float(np.mean(tl)),
                time_loss_p95=float(np.percentile(tl, 95)), time_loss_others=float(np.mean(tl_other)),
                total_loss_veh_min=float(np.sum(tl) / 60.0), max_queue=max_queue)


def set_font():
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in ("Malgun Gothic", "AppleGothic", "NanumGothic", "Noto Sans CJK KR"):
        if name in installed:
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False


def main():
    ap = argparse.ArgumentParser(description="SUMO 교통 모의실험")
    ap.add_argument("--seeds", type=int, default=10)
    args = ap.parse_args()
    p = PARAMS
    out = HERE / "results"
    out.mkdir(exist_ok=True)
    set_font()
    build_net()
    rows = []
    for lam in p["loads"]:
        pool = response_pool(p, lam)
        util = lam * dl.PARAMS["ai_mean"]
        res = {k: [run(s, k, pool, p) for s in range(args.seeds)] for k, _ in POLICIES}
        base = np.array([r["total_loss_veh_min"] for r in res["ideal"]])
        print(f"가정값 기준 · AI 서버 이용률 {util:.2f} · 시드 {args.seeds}개 · 방향마다 {p['flow']}대/시 · 묻는 차 {p['p_ask']:.0%}")
        print(f"{'방식':<18}{'감속한 차':>8}{'평균 손실[s]':>12}{'안 물은 차 손실[s]':>16}{'추가 손실[차량·분]':>18}{'최대 정지 줄':>11}")
        for k, label in POLICIES:
            r = res[k]
            extra = np.array([x["total_loss_veh_min"] for x in r]) - base
            half = 1.984 * extra.std(ddof=1) / math.sqrt(len(extra)) if len(extra) > 1 else 0.0
            row = dict(ai_util=round(util, 2), policy=k, label=label, n_seeds=args.seeds,
                       vehicles=round(np.mean([x["n"] for x in r]), 1), asked=round(np.mean([x["n_ask"] for x in r]), 1),
                       braked=round(np.mean([x["n_stop"] for x in r]), 1),
                       time_loss_mean_s=round(np.mean([x["time_loss"] for x in r]), 2),
                       time_loss_p95_s=round(np.mean([x["time_loss_p95"] for x in r]), 2),
                       time_loss_not_asked_s=round(np.mean([x["time_loss_others"] for x in r]), 2),
                       extra_loss_veh_min=round(float(extra.mean()), 2), extra_loss_ci95=round(half, 2),
                       max_halting=round(np.mean([x["max_queue"] for x in r]), 1))
            rows.append(row)
            print(f"{label:<18}{row['braked']:>8.1f}{row['time_loss_mean_s']:>12.2f}{row['time_loss_not_asked_s']:>16.2f}"
                  f"{row['extra_loss_veh_min']:>18.2f}{row['max_halting']:>11.1f}")
    with (out / "sumo_summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)

    colors = ["#d0d5db", "#9aa3ad", "#b9c3cf", "#7d8fa6", "#3f6c9b", "#14284b"]
    fig, axes = plt.subplots(1, len(p["loads"]), figsize=(11, 3.9))
    for ax, lam in zip(np.atleast_1d(axes), p["loads"]):
        util = round(lam * dl.PARAMS["ai_mean"], 2)
        sub = [r for r in rows if r["ai_util"] == util and r["policy"] != "ideal"]
        labels = [r["label"].replace(" 마감 순서", "\n마감 순서").replace(" + ", "\n+ ") for r in sub]
        vals = [r["extra_loss_veh_min"] for r in sub]
        bars = ax.bar(labels, vals, color=colors[1:], width=0.62)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v * 1.02 + 0.05, f"{v:.1f}", ha="center", fontsize=9.5)
        ax.set_title(f"AI 서버 이용률 {util}: 기다림 없음 대비 늘어난 손실 [차량·분]", fontsize=10)
        ax.tick_params(axis="x", labelsize=8)
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle(f"SUMO 교차로 10분: 늦은 답으로 감속·정지한 차와 뒤차의 손실 (가정값 기준, 시드 {args.seeds}개 평균)", fontsize=11)
    fig.tight_layout()
    fig.savefig(out / "fig1_sumo_compare.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
