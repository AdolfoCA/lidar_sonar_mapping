"""
census — the debug class-census log: how many voxels each class owns over time.

Every ``period`` seconds the classification node counts the voxels whose MAP
label (eq 15) is each class and appends one row here. The result is a tidy CSV
whose columns are exactly the classes of classes.yaml, so
``plot_class_census`` can draw one curve per class on a single plot: the
convergence history of the map.

The file carries a ``#`` comment header recording the class axis, the voxel
resolution and the evidence floor in force, because a census is only
comparable against another census taken under the same settings.

Columns
-------
    t_sec            seconds since the logger started
    stamp_iso        wall-clock timestamp of the row
    sweeps           sweeps processed so far (the map's actual "age")
    n_voxels         voxels counted (those passing the evidence floor)
    <class>          voxel count with MAP label <class>, one column per class
    mass_<class>     accumulated soft evidence sum_v m_{v,c}, one per class

Counts answer "how much of the map does this class own"; masses answer "how
much evidence has this class actually attracted", which moves smoothly and
earlier — a class can accumulate mass for a long time before it wins enough
voxels to appear in the count curve. Both are logged because reading them
together is what tells you whether a flat count curve means "not detected" or
"detected but always losing the argmax".

Pure Python — no ROS — so it can be driven from a replay script as easily as
from the node.
"""

import csv
import datetime
import os


class ClassCensusLog:
    """Append-only CSV of per-class voxel counts, one row per census tick."""

    def __init__(self, path: str, class_names, resolution: float,
                 min_evidence: float, period: float):
        self.path = os.path.abspath(os.path.expanduser(str(path)))
        self.class_names = list(class_names)
        self._rows = 0

        parent = os.path.dirname(self.path)
        if parent:
            os.makedirs(parent, exist_ok=True)

        # Truncated on open: a census is a single run's history, and silently
        # appending to a previous run's file would splice two unrelated maps
        # into one curve.
        with open(self.path, 'w', newline='') as fh:
            fh.write('# sonar_classification class census\n')
            fh.write(f'# started       : {datetime.datetime.now().isoformat(timespec="seconds")}\n')
            fh.write(f'# classes       : {", ".join(self.class_names)}\n')
            fh.write(f'# voxel size [m]: {resolution}\n')
            fh.write(f'# evidence floor: {min_evidence} (voxels with M_v below this are not counted)\n')
            fh.write(f'# period [s]    : {period}\n')
            csv.writer(fh).writerow(self.header())

    def header(self):
        return (['t_sec', 'stamp_iso', 'sweeps', 'n_voxels']
                + list(self.class_names)
                + [f'mass_{n}' for n in self.class_names])

    def write(self, t_sec: float, sweeps: int, n_voxels: int, counts, mass) -> None:
        """Append one census row and flush it — the log must survive a kill."""
        row = ([f'{t_sec:.3f}',
                datetime.datetime.now().isoformat(timespec='seconds'),
                int(sweeps), int(n_voxels)]
               + [int(c) for c in counts]
               + [f'{float(m):.6g}' for m in mass])
        with open(self.path, 'a', newline='') as fh:
            csv.writer(fh).writerow(row)
        self._rows += 1

    @property
    def rows(self) -> int:
        return self._rows
