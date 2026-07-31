"""Shared fixtures: a minimal on-disk class inventory the tests can load.

Building the ClassSet through ``load_classes`` rather than by hand means every
test also exercises the validator, and the YAML schema stays honest.
"""

import textwrap

import pytest

from sonar_classification.argus.classes import load_classes

MINIMAL_YAML = textwrap.dedent("""
    global:
      sonar_tilt_deg: 15.0
      elev_aperture_deg: 20.0
      beamwidth_deg: 1.0
      absorption_alpha_db_per_m: 1.74
      calibration_offset_db: 0.0
      tvg: {mode: none, u: 0.0, w: 0.0}
      lidar: {eps_xy: 0.30, k_min: 3}

    factors: {position: true, amplitude: true, lidar: true, shape: true}

    classes:
      # zero-extent: the Gaussian member of the height family
      - name: seabed
        position:  {l: 0.0, u: 0.0, tau: 0.15}
        amplitude: {mu: 4800, sigma: 3000}
        lidar:
          pi: 0.04
          clearance: {kind: box, l: 1.0, u: 6.0, sigma: 1.0}
        shape:
          rho: 0.05
          extent:    {median_m: 1.00, spread_factor: 4.0}
          thickness: {median_m: 0.15, spread_factor: 3.0}

      # extended, bed-anchored
      - name: vegetation
        position:  {l: 0.0, u: 1.5, tau: 0.35}
        amplitude: {mu: 10700, sigma: 6700}
        lidar:
          pi: 0.04
          clearance: {kind: box, l: 1.0, u: 6.0, sigma: 1.0}
        shape:
          rho: 0.80
          extent:    {median_m: 1.20, spread_factor: 2.5}
          thickness: {median_m: 0.60, spread_factor: 2.0}

      # surface-piercing, laterally extended
      - name: wall
        position:  {l: 0.0, u: 8.0, tau: 0.30}
        amplitude: {mu: 27000, sigma: 17000}
        lidar:
          pi: 0.90
          clearance: {kind: exp, gamma: 1.0}
        shape:
          rho: 0.90
          extent:    {median_m: 6.00, spread_factor: 2.0}
          thickness: {median_m: 0.10, spread_factor: 2.0}

      # surface-piercing, laterally compact — identical to wall except extent
      - name: pillar
        position:  {l: 0.0, u: 8.0, tau: 0.30}
        amplitude: {mu: 27000, sigma: 17000}
        lidar:
          pi: 0.75
          clearance: {kind: exp, gamma: 1.0}
        shape:
          rho: 0.90
          extent:    {median_m: 0.50, spread_factor: 2.0}
          thickness: {median_m: 0.10, spread_factor: 2.0}
    """)


def write_classes(tmp_path, yaml_text=MINIMAL_YAML, **overrides):
    """Write a class inventory to ``tmp_path`` and load it."""
    path = tmp_path / 'classes.yaml'
    path.write_text(yaml_text)
    cs = load_classes(str(path))
    if overrides:
        # ClassSet is frozen; tests that need an ablation rewrite the YAML.
        raise TypeError('use yaml_text to vary parameters, not overrides')
    return cs


@pytest.fixture
def cs(tmp_path):
    return write_classes(tmp_path)
