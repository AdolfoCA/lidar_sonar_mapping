#!/usr/bin/env python3
"""
train_intensity_correction.py
=============================
Learn a per-beam intensity correction ("denoise" the noisy azimuth beams) from the CSV
written by collect_intensity_sections.py.

Steps
-----
1. Time-aggregate the CSV -> a stationary per-section intensity obs[range_sec, beam_sec].
   (The per-beam GAIN artifact is stationary; the scene averages out over a long run.)
2. Clean reference per range: ref[r] = median over CLEAN beam sections (frac_band==0,
   inside the FOV) of obs[r, :].
3. Calibration map (BASELINE): factor[r,b] = ref[r] / obs[r,b]   (~1 clean, ~0.5 noisy).
   Apply as: corrected_intensity = raw_intensity * factor(beam, range).
4. Neural net (sklearn MLPRegressor): features [bearing_deg, range_m, tvg_db, gain_db]
   -> log(factor). A smooth, continuous correction usable at any (bearing,range) and,
   if you later train over several missions with DIFFERENT tvg/gain, across settings too.
5. Save calibration_map.npz, the NN (joblib), a per-beam factor CSV, and before/after plots.

TVG / gain
----------
The per-beam artifact is INDEPENDENT of TVG: a per-RANGE gain common to all beams cancels
in the per-beam ratio ref[r]/obs[r,b]. slope/gain are kept because they are (a) the recorded
acquisition settings, (b) NN features so a model trained across missions with different
settings can generalize, (c) usable for an optional range-compensated-backscatter output.
Defaults: --tvg-slope 6, --gain 35.

Usage
-----
  python3 train_intensity_correction.py --csv ~/ros2_ws/intensity_sections.csv \
      --tvg-slope 6 --gain 35 --out-dir ~/ros2_ws/intensity_model
"""
import argparse
import os

import numpy as np
import pandas as pd


def tvg_db(range_m, slope, gain):
    """Applied TVG gain [dB] at range R. Linear model gain + slope*R (BlueView-style).
    Swap here if your unit's curve differs (e.g. gain + 20*log10(R) + 2*alpha*R)."""
    return gain + slope * np.asarray(range_m, dtype=np.float64)


def aggregate_csv(csv_path, chunksize=4000):
    """Streaming MEAN per section column -> handles multi-GB CSVs without loading it all.
    (Averaging thousands of frames cancels the scene, leaving the stationary per-beam gain.)
    Returns (section_cols, mean[n_sec], n_range, n_beam, n_frames)."""
    sums = None
    count = 0
    sec_cols = nR = nB = None
    for chunk in pd.read_csv(csv_path, chunksize=chunksize):
        if sec_cols is None:
            sec_cols = [c for c in chunk.columns if c.startswith('r') and '_b' in c]
            sums = np.zeros(len(sec_cols), np.float64)
            nR, nB = int(chunk['n_range'].iloc[0]), int(chunk['n_beam'].iloc[0])
        v = chunk[sec_cols].to_numpy(np.float64)
        sums += np.nansum(v, axis=0)
        count += v.shape[0]
    return sec_cols, sums / max(count, 1), nR, nB, count


def main():
    pa = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    pa.add_argument('--csv', required=True)
    pa.add_argument('--meta', default=None, help='default <csv>.meta.csv')
    pa.add_argument('--out-dir', default=None, help='default <csv dir>/intensity_model')
    pa.add_argument('--tvg-slope', type=float, default=6.0)
    pa.add_argument('--gain', type=float, default=35.0)
    pa.add_argument('--bearing-gate', type=float, default=41.0, help='deg, clean ref FOV')
    pa.add_argument('--clip', type=float, nargs=2, default=[0.2, 5.0], help='factor clip lo hi')
    pa.add_argument('--hidden', type=int, nargs='*', default=[64, 64])
    a = pa.parse_args()

    meta_path = a.meta or (a.csv + '.meta.csv')
    out = a.out_dir or os.path.join(os.path.dirname(os.path.abspath(a.csv)), 'intensity_model')
    os.makedirs(out, exist_ok=True)

    # ---- 1. aggregate + align meta ----
    sec_cols, obs, nR, nB, frames = aggregate_csv(a.csv)
    print(f"aggregated {frames} frames, {len(sec_cols)} sections ({nR} range bins x {nB} beams)")
    meta = pd.read_csv(meta_path).set_index('section_id').loc[sec_cols].reset_index()
    ri = meta['ri'].to_numpy(int)
    bi = meta['bi'].to_numpy(int)
    bearing = 0.5 * (meta['bearing_lo_deg'] + meta['bearing_hi_deg']).to_numpy(float)
    rng = 0.5 * (meta['range_lo_m'] + meta['range_hi_m']).to_numpy(float)
    frac_band = meta['frac_band'].to_numpy(float)
    n_rsec, n_bsec = int(ri.max()) + 1, int(bi.max()) + 1

    # ---- 2. clean reference per range ----
    clean = (frac_band == 0) & (np.abs(bearing) <= a.bearing_gate) & np.isfinite(obs) & (obs > 0)
    ref = np.full(n_rsec, np.nan)
    for r in range(n_rsec):
        m = clean & (ri == r)
        if m.sum() >= 3:
            ref[r] = np.median(obs[m])
    ref_sec = ref[ri]

    # ---- 3. calibration factor (baseline) ----
    valid = np.isfinite(obs) & (obs > 0) & np.isfinite(ref_sec)
    factor = np.ones_like(obs)
    factor[valid] = np.clip(ref_sec[valid] / np.maximum(obs[valid], 1e-6), a.clip[0], a.clip[1])
    np.savez(os.path.join(out, 'calibration_map.npz'),
             section_id=np.array(sec_cols), ri=ri, bi=bi, bearing_deg=bearing, range_m=rng,
             frac_band=frac_band, obs=obs, ref=ref, factor=factor,
             n_range=nR, n_beam=nB, n_rsec=n_rsec, n_bsec=n_bsec,
             tvg_slope=a.tvg_slope, gain=a.gain)

    band_sel = frac_band > 0.5
    print(f"per-beam gain (obs/ref): band median={np.nanmedian((obs/ref_sec)[band_sel]):.2f}x  "
          f"clean median={np.nanmedian((obs/ref_sec)[clean]):.2f}x  "
          f"-> calibration factor band median={np.nanmedian(factor[band_sel]):.2f}")

    # ---- 4. neural net: features -> log(factor) ----
    from sklearn.neural_network import MLPRegressor
    from sklearn.preprocessing import StandardScaler
    from sklearn.pipeline import make_pipeline
    import joblib
    feat = np.column_stack([bearing, rng, tvg_db(rng, a.tvg_slope, a.gain),
                            np.full_like(rng, a.gain)])
    y = np.log(factor)
    model = make_pipeline(
        StandardScaler(),
        MLPRegressor(hidden_layer_sizes=tuple(a.hidden), activation='relu',
                     max_iter=3000, early_stopping=True, n_iter_no_change=30, random_state=0))
    model.fit(feat[valid], y[valid])
    pred = np.exp(model.predict(feat))
    rmse = float(np.sqrt(np.mean((np.log(pred[valid]) - y[valid]) ** 2)))
    joblib.dump({'model': model, 'tvg_slope': a.tvg_slope, 'gain': a.gain,
                 'features': ['bearing_deg', 'range_m', 'tvg_db', 'gain_db'],
                 'clip': a.clip},
                os.path.join(out, 'nn_correction.joblib'))
    print(f"NN trained: log-factor RMSE vs calibration = {rmse:.3f} "
          f"({'NN ~= calibration (artifact is simple/multiplicative)' if rmse < 0.1 else 'NN captures structure beyond a flat factor'})")

    # ---- 5. per-beam factor CSV + plots ----
    pb = pd.DataFrame({'bi': np.arange(n_bsec)})
    pb['bearing_deg'] = [np.median(bearing[bi == b]) for b in range(n_bsec)]
    pb['frac_band'] = [np.max(frac_band[bi == b]) for b in range(n_bsec)]
    pb['cal_factor'] = [np.median(factor[(bi == b) & valid]) if ((bi == b) & valid).any() else 1.0
                        for b in range(n_bsec)]
    pb['nn_factor'] = [np.median(pred[bi == b]) for b in range(n_bsec)]
    pb.to_csv(os.path.join(out, 'per_beam_factor.csv'), index=False)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    gain_before = (obs / ref_sec)
    fig, ax = plt.subplots(2, 1, figsize=(12, 8))
    # per-beam gain before vs after (median over ranges)
    gb = [np.nanmedian(gain_before[bi == b]) for b in range(n_bsec)]
    ga = [np.nanmedian((obs * factor / ref_sec)[bi == b]) for b in range(n_bsec)]
    gn = [np.nanmedian((obs * pred / ref_sec)[bi == b]) for b in range(n_bsec)]
    bc = pb['bearing_deg'].to_numpy()
    ax[0].axhline(1.0, color='k', lw=0.8, ls='--')
    ax[0].plot(bc, gb, label='before (raw/ref)')
    ax[0].plot(bc, ga, label='after calibration')
    ax[0].plot(bc, gn, label='after NN')
    for b in range(n_bsec):
        if pb['frac_band'][b] > 0.5:
            ax[0].axvspan(bc[b] - 0.1, bc[b] + 0.1, color='red', alpha=0.05)
    ax[0].set_xlabel('bearing [deg]'); ax[0].set_ylabel('per-beam gain (x ref)')
    ax[0].set_title('Per-beam gain: noisy beams ~2x before; ~1 after correction'); ax[0].legend()
    # correction factor: calibration scatter vs NN
    ax[1].scatter(bearing, factor, s=4, alpha=0.3, label='calibration factor (per section)')
    ax[1].scatter(bearing, pred, s=4, alpha=0.3, label='NN factor')
    ax[1].axhline(1.0, color='k', lw=0.8, ls='--')
    ax[1].set_xlabel('bearing [deg]'); ax[1].set_ylabel('correction factor'); ax[1].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'intensity_correction.png'), dpi=120)
    print(f"saved -> {out}/  (calibration_map.npz, nn_correction.joblib, "
          f"per_beam_factor.csv, intensity_correction.png)")


if __name__ == '__main__':
    main()
