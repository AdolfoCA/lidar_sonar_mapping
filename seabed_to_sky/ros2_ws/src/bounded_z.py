"""
Elevation uncertainty in the sonar image plane
===============================================
LEFT  panel: WITHOUT seabed prior -> delta_phi = 2*Phi = 20 deg (constant)
RIGHT panel: WITH seabed prior    -> delta_phi* = phi_hi* - phi_lo*

Both panels share the same colorbar (0 to 20 deg = full FOV).
The contrast between a flat 20 deg and a narrow ~1 deg window
is the visual argument.

BlueView M900 @ 2250 kHz
  gamma = 30 deg, h = 6 m, h_h = 1 m
  theta in [-65, +65] deg
  phi   in [-10, +10] deg
  r     in [0.5, 10] m
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── Parameters ────────────────────────────────────────────────────────────────
GAMMA_DEG  = 30.0
H          =  6.0
H_H        =  2.0
PHI_MAX    = 10.0          # elevation half-FOV (deg)
R_MIN, R_MAX         =  0.5, 10.0
THETA_MIN, THETA_MAX = -65.0, 65.0

Nr, Nth = 700, 700
r_arr  = np.linspace(R_MIN,    R_MAX,    Nr)
th_arr = np.linspace(THETA_MIN, THETA_MAX, Nth)
R_g, TH_g = np.meshgrid(r_arr, th_arr)   # (Nth, Nr)

gamma = np.deg2rad(GAMMA_DEG)
TH    = np.deg2rad(TH_g)

# ── Amplitude and phase ───────────────────────────────────────────────────────
AMP   = R_g * np.sqrt(np.sin(gamma)**2 * np.cos(TH)**2 + np.cos(gamma)**2)
delta = np.arctan2(-np.sin(gamma) * np.cos(TH), np.cos(gamma))

# ── Valid mask: beam reaches seabed ──────────────────────────────────────────
valid = AMP >= H

# ── phi* bounds (degrees) ────────────────────────────────────────────────────
phi_lo = np.full_like(AMP, np.nan)
phi_hi = np.full_like(AMP, np.nan)

phi_lo[valid] = np.rad2deg(np.arcsin(-H       / AMP[valid]) - delta[valid])
phi_hi[valid] = np.rad2deg(np.arcsin((-H+H_H) / AMP[valid]) - delta[valid])

# Clip to physical FOV
phi_lo = np.clip(phi_lo, -PHI_MAX, PHI_MAX)
phi_hi = np.clip(phi_hi, -PHI_MAX, PHI_MAX)

d_phi_prior = phi_hi - phi_lo          # Δφ* with prior  (deg)
d_phi_free  = np.full_like(AMP, 2*PHI_MAX)  # Δφ without prior = 20 deg (constant)

# Transpose for plotting
d_phi_free_p  = d_phi_free.T
d_phi_prior_p = np.where(valid.T, d_phi_prior.T, np.nan)
valid_p       = valid.T

# r zoom
r_valid_min = r_arr[np.any(valid_p, axis=1)].min()
r_margin    = 0.2

# Shared colorbar: 0 to full FOV (20 deg)
vmax   = 2 * PHI_MAX
levels = np.linspace(0, vmax, 80)

# ── Figure: single panel ──────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(9, 7))
fig.patch.set_facecolor("#1a1a2e")
ax.set_facecolor("#0d0d1a")
for sp in ax.spines.values():
    sp.set_edgecolor("#444466")
ax.tick_params(colors="white", labelsize=10)
ax.grid(True, color="#252540", lw=0.5, ls="--")

max_prior = float(np.nanmax(d_phi_prior_p))
fig.suptitle(
    f"Δϕ*  |  γ = {GAMMA_DEG}°,   h = {H} m,   h_h = {H_H} m   "
    f"[BlueView M900 @ 2250 kHz]\n"
    f"Max Δϕ* = {max_prior:.2f}°   (full FOV = {2*PHI_MAX:.0f}°)",
    color="white", fontsize=12, y=0.97
)

im = ax.contourf(th_arr, r_arr, d_phi_prior_p,
                  levels=levels, cmap="plasma", extend="neither")

# Invalid region
inv = (~valid_p).astype(float)
inv[inv == 0] = np.nan
ax.contourf(th_arr, r_arr, inv,
             levels=[0.5, 1.5], colors=["#0d0d1a"], alpha=1.0)
ax.contour(th_arr, r_arr, (~valid_p).astype(float),
            levels=[0.5], colors=["#ff5555"],
            linewidths=1.5, linestyles="--")

ax.set_ylim(r_valid_min - r_margin, R_MAX)
ax.set_xlabel("θ  azimuth (deg)", color="white", fontsize=11)
ax.set_ylabel("r  range (m)",     color="white", fontsize=11)

cb = fig.colorbar(im, ax=ax, pad=0.02)
cb.set_label("Δϕ*  (degrees)", color="white", fontsize=11)
cb.ax.yaxis.set_tick_params(color="white")
plt.setp(cb.ax.yaxis.get_ticklabels(), color="white")

plt.tight_layout()
plt.show()