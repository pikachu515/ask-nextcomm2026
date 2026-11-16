#!/usr/bin/env python3
"""ASK + SafeLink: transparent, *abstract* proof-of-concept simulator.

NOT a 3GPP NR, PHY/MAC, safety validation, or real radio measurement.
Compares FIFO, deadline-aware A, always-duplicate P1, reactive A->B,
and proposed deadline + predicted-quality-selective duplication.

Run: python ASK_SafeLink_demo.py
Outputs: ASK_SafeLink_metrics.csv and ASK_SafeLink_comparison.png
Requires: Python 3; matplotlib optional for the PNG.
"""
from dataclasses import dataclass
from collections import deque
import csv, heapq, math, random, argparse
from pathlib import Path

SEED = 20261112
REACTIVE_SETUP_DELAY = .08 # seconds; an assumption, never a 3GPP-defined value
DURATION = 30.0
V = 50 / 3.6 # m/s
BRAKE_A = 2.0 # m/s2, scenario assumption, not an actual automated-driving limit
STOP_DIST = V*V/(2*BRAKE_A)
MARGIN = 0.14 # s. Remaining local decision margin; illustrative only

@dataclass(frozen=True)
class Message:
    id:int; t:float; tier:int; distance:float; deadline:float; max_age:float
    u_a:float; u_b:float; u_a_delay:float; u_b_delay:float
    # These uniforms are fixed across policies for fair comparisons.


def radio_state(t):
    # Synthetic 5G-Uu telemetry: 13-18s congestion, 21-22.5s correlated fade.
    if 21 <= t < 22.5: return (3.0, 0.70, 0.030)
    if 13 <= t < 18: return (9.0, 0.15, 0.022)
    if 12 <= t < 13: return (12.0, 0.07, 0.015)
    return (18.0, 0.025, 0.012)


def direct_state(t):
    # B = *separately equipped roadside unit* with PC5-like direct path.
    # Joint outage models common-cause risk; paths are not fully independent.
    if 21 <= t < 22.5: return (0.50, 0.045)
    if 13 <= t < 18: return (0.065, 0.035)
    return (0.035, 0.030)


def generate():
    rng=random.Random(SEED)
    messages=[]
    t=0.0; idx=0
    while t < DURATION:
        t += rng.expovariate(54) # ~54 safety/normal messages per second
        if t>=DURATION: break
        r=rng.random()
        tier=1 if r<.27 else (2 if r<.76 else 3)
        if tier==1: distance=rng.uniform(53,74)
        elif tier==2: distance=rng.uniform(60,105)
        else: distance=rng.uniform(82,155)
        # A vehicle has to obtain information BEFORE it loses comfortable stopping distance.
        deadline=max(.055, min(1.4,(distance-STOP_DIST)/V - MARGIN))
        age_max={1:0.23,2:0.65,3:2.0}[tier]
        messages.append(Message(idx,t,tier,distance,deadline,age_max,rng.random(),rng.random(),rng.random(),rng.random()))
        idx+=1
    # Other users also load A's shared response scheduler (not counted as safety messages).
    background=[]
    t=0.0
    while t < DURATION:
        t += rng.expovariate(27)
        if t<DURATION: background.append((t,idx)); idx+=1
    return messages,background


def select(m,policy):
    if policy=='FIFO_A': return ('A',)
    if policy=='PRIO_EDF_A': return ('A',)
    if policy=='ALL_P1_DUP': return ('A','B') if m.tier==1 else ('A',)
    if policy=='REACTIVE': return ('A',) # potentially B after A times out
    if policy=='SAFE_LINK':
        sinr,loss,extra=radio_state(m.t)
        # Before the actual failure: use only *current* quality telemetry / congestion.
        # A learned predictor could replace this explicit rule in future work.
        predicted_risk = (sinr<13 or 13 <= m.t < 18) # abstract quality/traffic signal
        hard_deadline = m.deadline < .20
        if m.tier == 1:
            return ('A','B') if (predicted_risk or hard_deadline) else ('A',)
        if m.tier==2 and sinr<7: return ('B',)
        return ('A',)
    raise ValueError(policy)


def simulate(policy,messages,background):
    rng=random.Random(SEED+13)
    chosen={m.id:select(m,policy) for m in messages}
    # A's response queue represents uplink + edge + downlink; only the downlink
    # queue is explicit. A's uplink is a simple delay/loss abstraction.
    jobs=[]
    for m in messages:
        if 'A' in chosen[m.id]:
            jobs.append((m.t+.016, m.id, m.tier, m.t+m.deadline, m))
    for t,jid in background:
        jobs.append((t+.016,jid,4,t+10,None))
    jobs.sort(key=lambda x:(x[0],x[1]))
    arrivals={}; q=[]; queue_seq=0; last_end=0.0
    # Non-preemptive priority queue: a job already transmitting is not interrupted.
    i=0; time=0.0
    while i<len(jobs) or q:
        if not q and i<len(jobs): time=max(time,jobs[i][0])
        while i<len(jobs) and jobs[i][0]<=time+1e-12:
            ready,jid,tier,deadline,m=jobs[i]
            key=(queue_seq,) if policy in ('FIFO_A',) else (tier, deadline, queue_seq)
            heapq.heappush(q,(key,queue_seq,m)); queue_seq+=1; i+=1
        if not q: continue
        _,_,m=heapq.heappop(q)
        # Simulated shared downlink service rate: congestion slows responses.
        service=.010 if not (13<=time<18) else .022
        finish=time+service
        if m is not None:
            _,drop,extra=radio_state(finish)
            if m.u_a>=drop:
                arrivals.setdefault(m.id,[]).append((finish+.016+extra+.026*m.u_a_delay,'A'))
        time=finish
    # B is a direct RSU -> vehicle path, with independent send but correlated outage.
    txcost=0
    for m in messages:
        txcost += len(chosen[m.id])
        if 'B' in chosen[m.id]:
            loss,offset=direct_state(m.t)
            if m.u_b>=loss:
                arrivals.setdefault(m.id,[]).append((m.t+offset+.028*m.u_b_delay,'B'))
        if policy=='REACTIVE' and m.tier==1:
            # B activated only if A has not arrived within 120 ms.
            timeout=.12
            setup_delay=REACTIVE_SETUP_DELAY  # assumed cold-path activation overhead
            a_on_time=any(path=='A' and t<=m.t+timeout for t,path in arrivals.get(m.id,[]))
            if not a_on_time:
                txcost+=1
                loss,offset=direct_state(m.t+timeout+setup_delay)
                if m.u_b>=loss:
                    arrivals.setdefault(m.id,[]).append((m.t+timeout+setup_delay+offset+.028*m.u_b_delay,'B'))
    # Metrics: a message counts only if delivered before its physical decision
    # deadline AND before the information's max acceptable age.
    p1=[m for m in messages if m.tier==1]
    good=0; lat=[]; rejected_expired=0; no_delivery=0
    for m in p1:
        times=sorted(t for t,_ in arrivals.get(m.id,[]))
        if not times: no_delivery+=1
        if times and times[0]-m.t > m.max_age: rejected_expired+=1
        delivered=[t for t in times if t-m.t <= min(m.deadline,m.max_age)]
        if delivered:
            good+=1;lat.append((delivered[0]-m.t)*1000)
    return dict(policy=policy,urgent_messages=len(p1),on_time_pct=100*good/len(p1),
                cost_units=txcost, extra_sends=txcost-len(messages),
                expired_first_arrivals=rejected_expired,no_delivery=no_delivery,
                median_urgent_latency_ms=round(sorted(lat)[len(lat)//2],1) if lat else float('nan'))


def main():
    global SEED, REACTIVE_SETUP_DELAY
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed',type=int,default=SEED)
    parser.add_argument('--reactive-setup-ms',type=float,default=80.0)
    args=parser.parse_args()
    SEED=args.seed
    REACTIVE_SETUP_DELAY=args.reactive_setup_ms/1000.0
    messages,background=generate()
    policies=['FIFO_A','PRIO_EDF_A','REACTIVE','ALL_P1_DUP','SAFE_LINK']
    results=[simulate(p,messages,background) for p in policies]
    out=Path(__file__).resolve().parent/'results'; out.mkdir(exist_ok=True)
    csv_path=out/'ASK_SafeLink_metrics.csv'
    with csv_path.open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=results[0].keys());w.writeheader();w.writerows(results)
    print(f'Illustrative only: {len(messages)} messages; P1={results[0]["urgent_messages"]}; reactive_setup={args.reactive_setup_ms:g}ms')
    print('Policy             P1 on-time(%)   all sends   extra sends')
    for r in results: print(f'{r["policy"]:18s} {r["on_time_pct"]:11.2f} {r["cost_units"]:11d} {r["extra_sends"]:12d}')
    try:
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(9,5))
        names=[r['policy'].replace('_',' ') for r in results]
        ax.scatter([r['cost_units'] for r in results],[r['on_time_pct'] for r in results],s=80)
        for name,r in zip(names,results): ax.annotate(name,(r['cost_units'],r['on_time_pct']),xytext=(6,7),textcoords='offset points',fontsize=9)
        ax.set_xlabel('Message transmissions (A and B, arbitrary cost units)')
        ax.set_ylabel('Urgent P1 valid on-time delivery (%)')
        ax.set_title('ASK + SafeLink — conceptual network experiment (NOT 5G PHY)')
        ax.grid(alpha=.25);fig.tight_layout()
        fig.savefig(out/'ASK_SafeLink_comparison.png',dpi=170)
        print('Saved:', csv_path.name, 'ASK_SafeLink_comparison.png')
    except ImportError:
        print('Saved:', csv_path.name, '(install matplotlib for plot)')

if __name__=='__main__': main()
