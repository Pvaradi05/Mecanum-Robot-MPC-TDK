"""Charts of controller runs from their CSV logs (tooling only, not part of the controller).

    ros2 run mecanum_mpc_common plot_runs [--controller nmpc] <run_dir> ...       # one per run
    ros2 run mecanum_mpc_common plot_runs [--controller nmpc] --joined <out_stem> <run_dir> ...

Per run (<run_dir>/<shape>_<heading>_chart.png/.pdf): tracking and cross-track error, heading
error, body velocity v_x, v_y and yaw rate (measured and reference), and the four wheel torques
laid out like the wheels. Joined (<out_stem>.png/.pdf): one column per path, fixed and tangent
heading as two colours, rows: tracking error, heading error, wheel torque, v_x, v_y, yaw rate,
each smoothed with a 1 s centred window (moving RMS for errors and torque, moving mean for
velocities) so the two headings stay readable on top of each other; the per-run charts are raw.
Each <run_dir> holds one CSV written by a controller node (<shape>_<heading>_<speed>_<time>.csv).
Summary metrics (as evaluate, from t = 3 s) go to <out_stem>_summary.csv.
"""
import argparse
import csv
import glob
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .conventions import TITLES  # noqa: E402
from .evaluate import dist_to_polyline, load, metrics, wrap  # noqa: E402

TITLE = TITLES['lmpc_v2']      # set from --controller

SKIP = 3.0                     # s, start of the metric window (as in the earlier evaluations)
SHAPES = ('circle', 'ellipse', 'figure8', 'square')
NAMES = {'circle': 'Circle', 'ellipse': 'Ellipse', 'figure8': 'Figure-8', 'square': 'Square'}
WHEEL_NAMES = ('front left', 'front right', 'rear left', 'rear right')

# reference palette (dataviz skill), light mode: categorical slots 1-2 and the chart ink
BLUE, ORANGE = '#2a78d6', '#eb6834'
INK, INK2, MUTED = '#0b0b0b', '#52514e', '#898781'
GRID, BASE = '#e1e0d9', '#c3c2b7'
HEADING_COLOR = {'fixed': BLUE, 'tangent': ORANGE}

plt.rcParams.update({
    'font.size': 9, 'axes.titlesize': 10, 'axes.labelsize': 9,
    'axes.edgecolor': BASE, 'axes.linewidth': 0.6, 'axes.labelcolor': INK2,
    'axes.titlecolor': INK, 'axes.titleweight': 'bold', 'axes.titlelocation': 'left',
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.5, 'grid.linestyle': '-',
    'axes.spines.top': False, 'axes.spines.right': False,
    'xtick.color': MUTED, 'ytick.color': MUTED, 'xtick.labelcolor': INK2,
    'ytick.labelcolor': INK2, 'xtick.major.size': 0, 'ytick.major.size': 0,
    'legend.frameon': False, 'legend.fontsize': 8.5, 'figure.facecolor': 'white',
    'savefig.facecolor': 'white', 'lines.solid_capstyle': 'round',
})


def find_csv(run_dir):
    found = sorted(glob.glob(os.path.join(run_dir, '*.csv')))
    if len(found) != 1:
        raise SystemExit(f'{run_dir}: expected one CSV log, found {len(found)}')
    return found[0]


def run_data(run_dir):
    """Logged signals plus derived ones, and the run's name parts."""
    path = find_csv(run_dir)
    name = os.path.basename(path).split('_')
    shape, heading, speed = name[0], name[1], float(name[2])
    d = load(path)
    m = metrics(d, SKIP)
    on_path = d['t'] <= m['t_end']
    pos, ref = np.c_[d['X'], d['Y']], np.c_[d['X_r'], d['Y_r']]
    d['e_track'] = 1e3 * np.hypot(*(pos - ref).T)
    d['e_cross'] = 1e3 * dist_to_polyline(pos, ref[on_path])
    d['e_psi'] = np.degrees(wrap(d['psi'] - d['psi_r']))
    for sfx, psi in (('', d['psi']), ('_r', d['psi_r'])):           # body-frame velocity
        c, s = np.cos(psi), np.sin(psi)
        d['vx' + sfx] = c * d['X_dot' + sfx] + s * d['Y_dot' + sfx]
        d['vy' + sfx] = -s * d['X_dot' + sfx] + c * d['Y_dot' + sfx]
        d['w' + sfx] = d['psi_dot' + sfx]
    d['M'] = 1e3 * np.c_[d['M1'], d['M2'], d['M3'], d['M4']]         # mN m
    return dict(d=d, m=m, shape=shape, heading=heading, speed=speed, csv=path)


def _zero(ax):
    ax.axhline(0.0, color=BASE, lw=0.8, zorder=1)


def _path_end(ax, t_end, label=False):
    ax.axvline(t_end, color=MUTED, lw=0.8, zorder=1)
    if label:
        ax.annotate('path end', (t_end, 1.0), xycoords=('data', 'axes fraction'),
                    xytext=(-3, -2), textcoords='offset points', ha='right', va='top',
                    fontsize=7.5, color=MUTED)


def smooth(t, y, window=1.0, rms=False):
    """Centred moving mean (or RMS) over `window` seconds; shorter at the ends."""
    n = max(1, int(round(window / np.median(np.diff(t))))) | 1
    k = np.ones(n)
    num = np.convolve(y ** 2 if rms else y, k, 'same') / np.convolve(np.ones_like(y), k, 'same')
    return np.sqrt(num) if rms else num


def _title(fig, r, x=0.06):
    m = r['m']
    fig.text(x, 0.975, f'{TITLE}  ·  {NAMES[r["shape"]]}  ·  {r["heading"]} heading  ·  '
             f'{r["speed"]:g} m/s', fontsize=13, fontweight='bold', color=INK, va='top')
    fig.text(x, 0.945, f'Tracking error RMS {m["rms"]:.2f} mm (max {m["max"]:.2f})   ·   '
             f'cross-track RMS {m["ct_rms"]:.2f} mm   ·   heading RMS {m["psi_rms"]:.2f}°   ·   '
             f'peak wheel torque {1e3 * m["torque_max"]:.1f} mN·m   ·   QP failures '
             f'{m["not_solved"]}      (metrics from t = {SKIP:g} s to the end of the path)',
             fontsize=9, color=INK2, va='top')


def plot_run(run_dir):
    r = run_data(run_dir)
    d = r['d']
    t = d['t']
    fig = plt.figure(figsize=(13, 8.6))
    outer = fig.add_gridspec(1, 2, width_ratios=(1.45, 1), wspace=0.16,
                             left=0.06, right=0.985, top=0.885, bottom=0.07)
    left = outer[0].subgridspec(5, 1, hspace=0.42)
    axs = [fig.add_subplot(left[0])]
    axs += [fig.add_subplot(left[i], sharex=axs[0]) for i in range(1, 5)]

    ax = axs[0]
    ax.plot(t, d['e_track'], color=BLUE, lw=0.8, label='tracking error (to the reference point)')
    ax.plot(t, d['e_cross'], color=ORANGE, lw=0.8, label='cross-track error (to the path)')
    ax.set_title('Position error')
    ax.set_ylabel('mm')
    ax.set_ylim(bottom=0)
    ax.legend(loc='lower right', bbox_to_anchor=(1.0, 1.0), ncol=2, borderaxespad=0.2)

    ax = axs[1]
    _zero(ax)
    ax.plot(t, d['e_psi'], color=BLUE, lw=0.8)
    ax.set_title('Heading error  ψ − ψ_r')
    ax.set_ylabel('deg')

    for ax, key, title, unit in ((axs[2], 'vx', 'Body velocity  v_x (forward)', 'm/s'),
                                 (axs[3], 'vy', 'Body velocity  v_y (left)', 'm/s'),
                                 (axs[4], 'w', 'Yaw rate  ω', 'rad/s')):
        _zero(ax)
        ax.plot(t, d[key + '_r'], color=MUTED, lw=1.6, label='reference')
        ax.plot(t, d[key], color=BLUE, lw=0.8, label='measured')
        ax.set_title(title)
        ax.set_ylabel(unit)
    axs[2].legend(loc='lower right', bbox_to_anchor=(1.0, 1.0), ncol=2, borderaxespad=0.2)
    for i, ax in enumerate(axs):
        _path_end(ax, r['m']['t_end'], label=i == 1)
    for ax in axs[:-1]:
        ax.tick_params(labelbottom=False)
    axs[-1].set_xlabel('time  [s]')

    right = outer[1].subgridspec(2, 2, hspace=0.25, wspace=0.12)
    lim = 1.08 * np.abs(d['M']).max()
    tq = []
    for i in range(4):
        ax = fig.add_subplot(right[i // 2, i % 2], sharex=axs[0],
                             sharey=tq[0] if tq else None)
        _zero(ax)
        _path_end(ax, r['m']['t_end'])
        ax.plot(t, d['M'][:, i], color=BLUE, lw=0.6)
        ax.set_title(f'M{i + 1}  ·  {WHEEL_NAMES[i]}')
        ax.set_ylim(-lim, lim)
        if i % 2:
            ax.tick_params(labelleft=False)
        else:
            ax.set_ylabel('wheel torque  [mN·m]')
        if i < 2:
            ax.tick_params(labelbottom=False)
        else:
            ax.set_xlabel('time  [s]')
        tq.append(ax)
    _title(fig, r)
    stem = os.path.join(run_dir, f'{r["shape"]}_{r["heading"]}_chart')
    for ext in ('png', 'pdf'):
        fig.savefig(f'{stem}.{ext}', dpi=180)
    plt.close(fig)
    return r


def plot_joined(runs, out_stem, window=1.0):
    shapes = [s for s in SHAPES if any(r['shape'] == s for r in runs)]
    rows = (('e_track', True, 'Tracking error  (moving RMS)', 'mm'),
            ('e_psi', True, 'Heading error  (moving RMS)', 'deg'),
            ('M', True, 'Wheel torque, all four wheels  (moving RMS)', 'mN·m'),
            ('vx', False, 'Body velocity v_x, forward  (moving mean)', 'm/s'),
            ('vy', False, 'Body velocity v_y, left  (moving mean)', 'm/s'),
            ('w', False, 'Yaw rate ω  (moving mean)', 'rad/s'))
    fig, axs = plt.subplots(len(rows), len(shapes), figsize=(3.3 * len(shapes) + 0.9, 13),
                            sharex='col', sharey='row', squeeze=False,
                            gridspec_kw=dict(hspace=0.42, wspace=0.07, left=0.06, right=0.99,
                                             top=0.89, bottom=0.045))
    for j, s in enumerate(shapes):
        col = sorted((r for r in runs if r['shape'] == s), key=lambda r: r['heading'])
        for r in col:
            d = r['d']
            for i, (key, rms, _, _) in enumerate(rows):
                y = d[key]
                if key == 'M':                      # RMS over the four wheels, then over time
                    y = np.sqrt((y ** 2).mean(axis=1))
                axs[i, j].plot(d['t'], smooth(d['t'], y, window, rms),
                               color=HEADING_COLOR[r['heading']], lw=1.3)
        for i, ax in enumerate(axs[:, j]):
            _path_end(ax, col[0]['m']['t_end'], label=i == 0)
        pos = axs[0, j].get_position()
        fig.text(pos.x0, 0.915, NAMES[s], fontsize=12, fontweight='bold', color=INK,
                 va='bottom')
        axs[-1, j].set_xlabel('time  [s]')
    for i, (_, rms, title, unit) in enumerate(rows):
        for ax in axs[i]:
            if rms:
                ax.set_ylim(bottom=0)
            else:
                _zero(ax)
        axs[i, 0].set_title(title, fontsize=9.5)
        axs[i, 0].set_ylabel(unit)
    handles = [plt.Line2D([], [], color=c, lw=2.0) for c in HEADING_COLOR.values()]
    fig.legend(handles, [f'{h} heading' for h in HEADING_COLOR], loc='upper right',
               bbox_to_anchor=(0.99, 0.985), ncol=2, fontsize=10)
    speeds = sorted({r['speed'] for r in runs})
    fig.text(0.06, 0.985, f'{TITLE} on every path', fontsize=14, fontweight='bold',
             color=INK, va='top')
    fig.text(0.06, 0.958, f'Gazebo, one lap at {", ".join(f"{v:g}" for v in speeds)} m/s, '
             'untuned weights. Each row shares its y-axis, so the paths compare directly.\n'
             f'Signals smoothed over a {window:g} s centred window; the raw signals are in the '
             'per-run charts.', fontsize=9.5, color=INK2, va='top', linespacing=1.5)
    for ext in ('png', 'pdf'):
        fig.savefig(f'{out_stem}.{ext}', dpi=170)
    plt.close(fig)


def write_summary(runs, path):
    cols = ('path', 'heading', 'speed_m_s', 'track_rms_mm', 'track_max_mm', 'cross_rms_mm',
            'cross_max_mm', 'heading_rms_deg', 'heading_max_deg', 'torque_peak_mNm',
            'torque_rms_mNm', 'qp_ms_p50', 'qp_ms_p95', 'qp_failures')
    order = {s: i for i, s in enumerate(SHAPES)}
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in sorted(runs, key=lambda r: (order.get(r['shape'], 9), r['heading'])):
            m = r['m']
            w.writerow([r['shape'], r['heading'], r['speed']] + [f'{v:.3f}' for v in (
                m['rms'], m['max'], m['ct_rms'], m['ct_max'], m['psi_rms'], m['psi_max'],
                1e3 * m['torque_max'], 1e3 * m['torque_rms'], *m['qp_ms'])] + [m['not_solved']])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('run_dirs', nargs='+')
    ap.add_argument('--joined', metavar='OUT_STEM', help='also write the joined chart')
    ap.add_argument('--controller', default='lmpc_v2', choices=sorted(TITLES))
    a = ap.parse_args()
    global TITLE
    TITLE = TITLES[a.controller]
    runs = []
    for rd in a.run_dirs:
        runs.append(plot_run(rd))
        print(f'{rd}: chart written')
    if a.joined:
        plot_joined(runs, a.joined)
        write_summary(runs, f'{a.joined}_summary.csv')
        print(f'{a.joined}.png/.pdf and {a.joined}_summary.csv written')


if __name__ == '__main__':
    main()
