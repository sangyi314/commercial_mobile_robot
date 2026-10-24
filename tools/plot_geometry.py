#!/usr/bin/env python3
"""Render one immutable offline report. PNG creation is exclusive."""
import argparse
from pathlib import Path
import sys
sys.dont_write_bytecode = True
from geometry_inputs import read, schema


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    r = read(a.report); schema(r, 'evaluation_report')
    if a.output.exists() or a.output.is_symlink(): raise ValueError('OUTPUT_ALREADY_EXISTS')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout='constrained')
    for pair in r['pairs']:
        ax = axes[0]
        for interval in pair['intervals']:
            if not interval['valid']:
                ax.axvspan(interval['interval_ns'][0]/1e9, interval['interval_ns'][1]/1e9, color='gray', alpha=.3)
        points = pair['aligned_points']
        # Draw only declared valid intervals, never join through gaps.
        by_time = {x['time_ns']: x for x in points}
        for interval in pair['intervals']:
            if interval['valid']:
                l, h = interval['interval_ns']
                ax.plot([l/1e9,h/1e9],[by_time[l]['clearance_m'],by_time[h]['clearance_m']],color='#2864a6',linewidth=1.4)
        ax.scatter([x['time_ns']/1e9 for x in points],[x['clearance_m'] for x in points],s=2,color='#2864a6')
        m = pair['linear_model_min']
        if m: ax.scatter([m['time_ns']/1e9],[m['clearance_m']],color='#ba3131',s=45,zorder=4,label='Linear-model minimum')
        for i,(name, samples) in enumerate(pair['normalized_samples'].items()):
            color=['#2864a6','#d17a16'][i%2]
            selected=[s for s in samples if r['scope_ns'][0]<=s['t_ns']<=r['scope_ns'][1]]
            axes[1].scatter([s['center_xyz'][0] for s in selected],[s['center_xyz'][1] for s in selected],s=3,color=color,label=name)
    axes[0].axhline(0,color='black',linestyle='--',linewidth=.8)
    axes[0].set(xlabel='Gazebo source simulation time (s)',ylabel='Circle signed clearance (m)',title='Local pair / '+r['coverage']+' coverage')
    axes[0].grid(alpha=.2);axes[0].legend(fontsize=8)
    axes[1].set(xlabel='World X (m)',ylabel='World Y (m)',title='Recorded actual circle centers');axes[1].axis('equal');axes[1].grid(alpha=.2);axes[1].legend(fontsize=8)
    fig.suptitle(r['run_id']+'\nCircle-envelope / piecewise-linear model; physical collision UNKNOWN',fontsize=10)
    with a.output.open('xb') as f:fig.savefig(f,format='png',dpi=160)
    plt.close(fig)
    print(a.output.resolve())


if __name__ == '__main__': main()
