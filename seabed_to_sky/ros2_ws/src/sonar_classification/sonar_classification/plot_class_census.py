#!/usr/bin/env python3
"""
plot_class_census — turn the debug class-census CSV into one plot with one
curve per class.

    ros2 run sonar_classification plot_class_census [CSV] [-o OUT.png] [--dark]
                                                    [--metric count|mass|both]
                                                    [--show]

The CSV is written by classification_node when ``debug_census`` is enabled: one
row every ``census_period_sec`` seconds, one column per class of classes.yaml.
This script reads the class axis straight out of the header, so adding or
renaming a class needs no change here.

    count  how many voxels each class currently OWNS (its MAP label, eq 15).
           This is the convergence history of the map, and the default.
    mass   how much soft evidence each class has ATTRACTED, sum_v m_{v,c}.
           It moves earlier and more smoothly: a class can accumulate evidence
           for a long time before it wins enough voxels to appear in the count
           curve, so a flat count curve plus a rising mass curve means
           "detected but always losing the argmax", not "not detected".

``--metric both`` draws the two as stacked panels sharing the time axis —
never as two y-scales on one plot, which would let the arbitrary relative
scaling of the axes imply a relationship that is not in the data.
"""

import argparse
import csv
import os
import sys

import numpy as np

# Categorical hues in FIXED slot order — never cycled, never generated. Colour
# follows the class, not its rank, so a class keeps its hue across runs and
# across filtered views.
_SERIES_LIGHT = ('#2a78d6', '#eb6834', '#1baf7a', '#eda100',
                 '#e87ba4', '#008300', '#4a3aa7', '#e34948')
_SERIES_DARK = ('#3987e5', '#d95926', '#199e70', '#c98500',
                '#d55181', '#008300', '#9085e9', '#e66767')

_THEME = {
    'light': dict(series=_SERIES_LIGHT, surface='#fcfcfb',
                  primary='#0b0b0b', secondary='#52514e', grid='#e2e1dd',
                  other='#8a8983'),
    'dark': dict(series=_SERIES_DARK, surface='#1a1a19',
                 primary='#ffffff', secondary='#c3c2b7', grid='#33322f',
                 other='#8a8983'),
}

#: Past eight series a ninth hue would be invented, so the tail folds into one
#: aggregate "other" curve instead. Ranked by peak value, so the classes that
#: actually shape the map keep their identity.
_MAX_SERIES = 8


def read_census(path: str):
    """(t, sweeps, names, counts, mass) from a census CSV.

    Comment lines (``#``) carry provenance and are skipped; the real header row
    defines the class axis.
    """
    with open(path, 'r', newline='') as fh:
        rows = [r for r in csv.reader(fh)
                if r and not r[0].lstrip().startswith('#')]
    if len(rows) < 2:
        raise SystemExit(f'{path}: no census rows yet — was the run long '
                         'enough for at least two ticks?')

    header = rows[0]
    fixed = 4                                  # t_sec, stamp_iso, sweeps, n_voxels
    n_cls = (len(header) - fixed) // 2
    names = header[fixed:fixed + n_cls]

    data = np.array([[r[0]] + r[2:] for r in rows[1:]], dtype=np.float64)
    t = data[:, 0]
    sweeps = data[:, 1]
    counts = data[:, 3:3 + n_cls]
    mass = data[:, 3 + n_cls:3 + 2 * n_cls]
    return t, sweeps, names, counts, mass


def _fold(names, values):
    """Cap the series count, folding the smallest into a single 'other' curve."""
    if len(names) <= _MAX_SERIES:
        return list(names), values, None
    order = np.argsort(-values.max(axis=0))
    keep = np.sort(order[:_MAX_SERIES - 1])
    rest = np.setdiff1d(np.arange(len(names)), keep)
    folded = values[:, rest].sum(axis=1)
    return ([names[i] for i in keep], values[:, keep],
            (f'other ({len(rest)} classes)', folded))


def _panel(ax, t, names, values, theme, ylabel, title):
    """One time-series panel: thin lines, recessive frame, ink-coloured labels."""
    names, values, other = _fold(names, values)
    colors = theme['series']

    handles = []
    for i, nm in enumerate(names):
        (ln,) = ax.plot(t, values[:, i], lw=2.0, color=colors[i],
                        solid_capstyle='round', label=nm)
        handles.append(ln)
    if other is not None:
        (ln,) = ax.plot(t, other[1], lw=2.0, color=theme['other'],
                        solid_capstyle='round', label=other[0])
        names = names + [other[0]]
        handles.append(ln)

    # Direct labels only while they stay legible — a label per curve past four
    # collides more than it clarifies, and the legend already carries identity.
    if len(names) <= 4 and t.size:
        series = [values[:, i] for i in range(values.shape[1])]
        if other is not None:
            series.append(other[1])
        span = max(t[-1] - t[0], 1e-6)
        for nm, col, ys in zip(names, list(colors[:len(names)]), series):
            ax.plot([t[-1]], [ys[-1]], marker='o', ms=5, color=col, zorder=5)
            ax.annotate(nm, xy=(t[-1], ys[-1]),
                        xytext=(0.012 * span, 0), textcoords='offset points',
                        color=theme['primary'], fontsize=9,
                        va='center', ha='left', clip_on=False)

    ax.set_ylabel(ylabel, color=theme['secondary'], fontsize=10)
    if title:
        ax.set_title(title, color=theme['primary'], fontsize=11,
                     loc='left', pad=10)
    ax.grid(True, axis='y', color=theme['grid'], lw=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(theme['grid'])
    ax.tick_params(colors=theme['secondary'], labelsize=9)
    ax.set_ylim(bottom=0)
    return handles, names


def plot(path: str, out: str, metric: str = 'count', dark: bool = False,
         show: bool = False) -> str:
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    theme = _THEME['dark' if dark else 'light']
    t, sweeps, names, counts, mass = read_census(path)

    panels = ({'count': [('count', counts, 'voxels with this MAP label')],
               'mass': [('mass', mass, 'accumulated soft evidence')],
               'both': [('count', counts, 'voxels with this MAP label'),
                        ('mass', mass, 'accumulated soft evidence')]}[metric])

    fig, axes = plt.subplots(len(panels), 1, figsize=(10, 4.4 * len(panels)),
                             sharex=True, constrained_layout=True)
    axes = np.atleast_1d(axes)
    fig.patch.set_facecolor(theme['surface'])

    handles = labels = None
    for ax, (_, values, ylabel) in zip(axes, panels):
        ax.set_facecolor(theme['surface'])
        handles, labels = _panel(
            ax, t, names, values, theme, ylabel,
            title=None)

    axes[-1].set_xlabel('time since census start [s]',
                        color=theme['secondary'], fontsize=10)
    fig.suptitle(f'Per-class voxel census  —  {os.path.basename(path)}\n'
                 f'{int(sweeps[-1])} sweeps over {t[-1]:.0f} s',
                 color=theme['primary'], fontsize=12, ha='left', x=0.01)

    # Identity is never colour alone: the legend is always present for two or
    # more series, direct labels above are the redundant cue where they fit.
    leg = fig.legend(handles, labels, loc='outside lower center',
                     ncol=min(len(labels), 5), frameon=False, fontsize=9)
    for text in leg.get_texts():
        text.set_color(theme['primary'])

    fig.savefig(out, dpi=160, facecolor=theme['surface'])
    if show:
        plt.show()
    plt.close(fig)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Plot the sonar_classification class census: one curve per '
                    'class, on one plot.')
    ap.add_argument('csv', nargs='?', default='~/ros2_ws/class_census.csv',
                    help='census CSV written by classification_node '
                         '(default: %(default)s)')
    ap.add_argument('-o', '--out', default=None,
                    help='output PNG (default: alongside the CSV)')
    ap.add_argument('--metric', choices=('count', 'mass', 'both'),
                    default='count',
                    help='voxel counts (default), accumulated soft evidence, '
                         'or both as stacked panels')
    ap.add_argument('--dark', action='store_true',
                    help='render on the dark surface')
    ap.add_argument('--show', action='store_true',
                    help='also open an interactive window')
    args = ap.parse_args(argv)

    path = os.path.abspath(os.path.expanduser(args.csv))
    if not os.path.exists(path):
        raise SystemExit(
            f'{path}: not found. Enable the census first:\n'
            '  ros2 launch sonar_classification sonar_classification.launch.py '
            'debug_census:=true')
    out = args.out or os.path.splitext(path)[0] + f'_{args.metric}.png'
    out = os.path.abspath(os.path.expanduser(out))

    written = plot(path, out, metric=args.metric, dark=args.dark,
                   show=args.show)
    print(written)
    return 0


if __name__ == '__main__':
    sys.exit(main())
