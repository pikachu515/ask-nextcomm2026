#!/usr/bin/env python3
"""Reproducible repeated-seed sensitivity study for an abstract link model.
This does NOT implement real 5G NR/PC5 or a learned AI predictor.
Run from same directory as ASK_SafeLink_demo.py: python ASK_SafeLink_eval.py
"""
import csv, math, statistics
from pathlib import Path
import safelink_demo as sim

policies = ['FIFO_A','PRIO_EDF_A','REACTIVE','ALL_P1_DUP','SAFE_LINK']
setups_ms = [0,40,80,160]
seeds = range(20261001,20261101)  # 100 fixed seeds
rows=[]
for ms in setups_ms:
    sim.REACTIVE_SETUP_DELAY=ms/1000
    for seed in seeds:
        sim.SEED=seed
        messages, background=sim.generate()
        for policy in policies:
            result=sim.simulate(policy,messages,background)
            rows.append({'setup_ms':ms,'seed':seed,**result})

def report(rows,path):
    with path.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]))
        w.writeheader();w.writerows(rows)

folder=Path(__file__).resolve().parent/'results'; folder.mkdir(exist_ok=True)
report(rows,folder/'ASK_SafeLink_repeated_raw.csv')
summary=[]
for ms in setups_ms:
    for p in policies:
        sub=[r for r in rows if r['setup_ms']==ms and r['policy']==p]
        vals=[float(r['on_time_pct']) for r in sub]
        c=[float(r['cost_units']) for r in sub]
        mean=statistics.mean(vals); sd=statistics.stdev(vals)
        ci=1.984*sd/math.sqrt(len(vals)) # t approx df=99
        summary.append({'setup_ms':ms,'policy':p,'n_seeds':len(sub),
            'mean_on_time_pct':round(mean,3),'ci95_low_pct':round(mean-ci,3),'ci95_high_pct':round(mean+ci,3),
            'mean_cost_units':round(statistics.mean(c),1),'mean_extra_sends':round(statistics.mean([float(r['extra_sends']) for r in sub]),1)})
report(summary,folder/'ASK_SafeLink_repeated_summary.csv')
print('MODEL-BASED, NOT REAL 5G/6G MEASUREMENTS; 100 independent pseudo-random seeds')
for r in summary:
    if r['setup_ms'] in [0,80]: print(r)
try:
    import matplotlib.pyplot as plt
    base=[r for r in summary if r['setup_ms']==80]
    fig,axs=plt.subplots(1,2,figsize=(12.6,4.3))
    labels=['FIFO','PRIO+EDF','REACTIVE','ALWAYS DUP','SAFE LINK']
    xs=list(range(5))
    axs[0].bar(xs,[r['mean_on_time_pct'] for r in base],color=['#9DAAB3','#7297B1','#8B98A3','#3A8F87','#225D74'])
    axs[0].set_xticks(xs,labels,rotation=22,ha='right')
    axs[0].set_ylabel('P1 on-time valid delivery (%)')
    axs[0].set_ylim(0,105)
    axs[0].set_title('Same synthetic environment, 100 random seeds')
    axs[1].bar(xs,[r['mean_cost_units'] for r in base],color=['#9DAAB3','#7297B1','#8B98A3','#3A8F87','#225D74'])
    axs[1].set_xticks(xs,labels,rotation=22,ha='right')
    axs[1].set_ylabel('Total send attempts (proxy only)')
    axs[1].set_title('Higher reliability may require more transmissions')
    for ax in axs: ax.grid(axis='y',alpha=.16);ax.set_axisbelow(True)
    fig.suptitle('ASK + SafeLink CONCEPT ONLY — no 3GPP RF/MAC simulation',fontsize=12,weight='bold')
    fig.tight_layout()
    fig.savefig(folder/'ASK_SafeLink_repeated_results.png',dpi=180,bbox_inches='tight')
    plt.close(fig)
except ImportError: pass
