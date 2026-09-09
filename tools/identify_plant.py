#!/usr/bin/env python3
"""
identify_plant.py — BIAB kettle plant identification and PID tuning
--------------------------------------------------------------------
Loads a bibby CSV log of a heat-then-cool test, drives the lumped kettle
model with the power that was actually delivered, and fits the model's
physical parameters to the measured temperature by least squares:

    m·c · dT/dt = P(t) − k_loss · (T − T_amb)          (kettle)
    T_meas(t)   = T(t − L)                             (probe + filter delay)

    m·c     [J/°C]   thermal mass of the batch (≈ 4186 J/°C per litre)
    k_loss  [W/°C]   heat-loss coefficient to the room
    L       [s]      effective dead time (probe lag + filter group delay)

No step is assumed: any power trace works, so no analysis window has to be
chosen — the whole log is used.  The heating phase pins down m·c (the ramp
slope) and L (the corners at power on/off); the cooling phase pins down
k_loss (the decay rate toward ambient).  Together they fix the FOPDT
equivalents used by the classical tuning rules:

    K = 1 / k_loss  [°C/W]      τ = m·c / k_loss  [s]      L  [s]

Two things the lumped model does not contain are measured separately and
reported: the dead time read straight off the power-on corner (the least-
squares L is unreliable when the probe sits in the element flow), and the
mixing offset — how far above the bulk the probe reads while the elements
run, visible as a fast drop right after power-off.

The ambient temperature is NOT estimated: pass the room temperature you
measured with a separate thermometer as --ambient.

The bibby control law works in WATTS: the PID output is a power demand that
the equal-flux splitter turns into per-element duties.  All gains printed
here are therefore watts-based:

    kp  [W / °C]
    ki  [W / °C per sample]   (the integrator accumulates raw error/sample)
    kd  [W·sample / °C]       (the derivative is a raw per-sample delta)

Requirements:
    pip install numpy scipy matplotlib pandas

Usage:
    python3 identify_plant.py <log.csv> --ambient <°C> [OPTIONS]

Options:
    --ambient C     Room / ambient temperature during the test, °C. Required.
    --volume-l L    Water volume you put in.  Compares the fitted m·c against
                    the water alone to show whether the delivered watts are
                    really what bibby.ini says (mains voltage, element
                    tolerance) — the usual reason the two disagree.
    --dt      SEC   Resample period for the fit (default 1.0 s).  The kettle
                    moves on a minutes time-scale; 1 s loses nothing.
    --rule    RULE  Tuning rules to print: simc | zn | cc | all (default: simc).
                    ZN/CC reaction-curve rules are meaningless on a lag-
                    dominant plant (L ≪ τ) and print absurd gains there.
    --lambda  SEC   SIMC closed-loop time constant τc, seconds.  Default:
                    max(3·L, 120 s) — smooth rather than tight.
    -o FILE         Save plot to FILE instead of displaying interactively.

Workflow
--------
1.  Collect a heat/cool log (README §9.4): manual mode, ~80 % power until
    mash temperature, then 0 W and let it cool for a while.  Note the room
    temperature.
2.  Run this script on the whole log with --ambient.
3.  Copy the recommended [pid] and [feedforward] blocks into bibby.ini.
4.  Validate with a setpoint step in auto mode (README §9.5).
"""

import argparse
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.signal import lfilter


# ── CSV loading ───────────────────────────────────────────────────────────────

def load_csv(path: str) -> pd.DataFrame:
  df = pd.read_csv(path, comment='#')
  df.columns = df.columns.str.strip()
  required = ['t_monotonic_s', 'temp_filt_c', 'p_delivered_w']
  missing = [c for c in required if c not in df.columns]
  if missing:
    sys.exit(f"CSV is missing required columns: {missing} "
             "(pre-rebuild duty-format logs are not supported)")
  df = df[required].apply(pd.to_numeric, errors='coerce').dropna()
  df = df.sort_values('t_monotonic_s').reset_index(drop=True)
  if len(df) < 100:
    sys.exit(f"Only {len(df)} usable rows in the log.")
  if float(df['p_delivered_w'].abs().max()) < 1.0:
    sys.exit("p_delivered_w is zero throughout — the SSRs never fired. "
             "Nothing to identify.")
  return df


def resample(df: pd.DataFrame, dt: float):
  """Bin-average onto a uniform grid of period dt (seconds).

  Power is averaged inside each bin (exact for energy); temperature is
  averaged too (a negligible extra smoothing).  Empty bins — the logger
  drops to one row per slow tick during a fault — are filled: power is held
  from the previous bin, temperature is interpolated.
  """
  t = df['t_monotonic_s'].values - df['t_monotonic_s'].values[0]
  T = df['temp_filt_c'].values.astype(float)
  P = df['p_delivered_w'].values.astype(float)
  idx = np.floor(t / dt).astype(int)
  n = int(idx[-1]) + 1
  cnt = np.bincount(idx, minlength=n).astype(float)
  have = cnt > 0
  Tg = np.full(n, np.nan)
  Pg = np.full(n, np.nan)
  Tg[have] = np.bincount(idx, weights=T, minlength=n)[have] / cnt[have]
  Pg[have] = np.bincount(idx, weights=P, minlength=n)[have] / cnt[have]
  tg = (np.arange(n) + 0.5) * dt
  if not have.all():
    ar = np.arange(n)
    Tg = np.interp(tg, tg[have], Tg[have])
    last = np.maximum.accumulate(np.where(have, ar, 0))
    Pg = Pg[last]
  return tg, Tg, Pg


# ── Model ─────────────────────────────────────────────────────────────────────

def simulate(params, tg, Pg, dt, T_amb):
  """Exact discretisation of the lumped model for piecewise-constant power.

      θ[n+1] = a·θ[n] + b·P[n],   a = exp(−dt·k/mc),  b = (1−a)/k
  θ is the kettle temperature above ambient; θ0 is its value at t = 0.
  The measurement is the kettle trace delayed by L (linear interpolation).
  Returns (T_measured_model, T_kettle_model).
  """
  mc, k, L, th0 = params
  a = np.exp(-dt * k / mc)
  b = (1.0 - a) / k
  n = len(Pg)
  th = lfilter([0.0, b], [1.0, -a], Pg) + th0 * a ** np.arange(n)
  Tk = T_amb + th
  Tm = np.interp(tg - L, tg, Tk, left=Tk[0])
  return Tm, Tk


def find_phases(tg, Pg, dt):
  """Split the log into contiguous 'heat' (power on) and 'cool' (power off)
  segments.  Returns a list of dicts with kind, i0, i1 (inclusive)."""
  on = Pg > 0.05 * max(float(Pg.max()), 1.0)
  phases = []
  i0 = 0
  for i in range(1, len(on) + 1):
    if i == len(on) or on[i] != on[i0]:
      phases.append({'kind': 'heat' if on[i0] else 'cool', 'i0': i0, 'i1': i - 1})
      i0 = i
  # Drop blips shorter than 10 s by merging into neighbours.
  min_len = max(int(round(10.0 / dt)), 1)
  merged = []
  for ph in phases:
    if merged and (ph['i1'] - ph['i0'] + 1) < min_len:
      merged[-1]['i1'] = ph['i1']
    elif merged and merged[-1]['kind'] == ph['kind']:
      merged[-1]['i1'] = ph['i1']
    else:
      merged.append(dict(ph))
  return merged


def initial_guess(tg, Tg, Pg, dt, T_amb, phases):
  """Physically-motivated starting point from the energy balance."""
  E = float(np.sum(Pg) * dt)
  rise = float(Tg.max() - Tg[0])
  mc0 = E / rise if rise > 0.5 else 4186.0 * 20.0
  mc0 = float(np.clip(mc0, 1e3, 1e7))

  k0 = None
  cools = [p for p in phases if p['kind'] == 'cool' and p['i0'] > 0]
  if cools:
    ph = max(cools, key=lambda p: p['i1'] - p['i0'])
    th_a = Tg[ph['i0']] - T_amb
    th_b = Tg[ph['i1']] - T_amb
    dur = (ph['i1'] - ph['i0']) * dt
    if th_a > 1.0 and th_b > 0.2 and th_a > th_b and dur > 30.0:
      k0 = mc0 * np.log(th_a / th_b) / dur
  if k0 is None or not np.isfinite(k0) or k0 <= 0:
    k0 = mc0 / 2000.0
  k0 = float(np.clip(k0, 0.1, 1e4))
  th0 = float(Tg[0] - T_amb)
  return mc0, k0, th0


BOUNDS_LO = [1e2, 1e-3, 0.0,   -60.0]     # mc, k_loss, L, θ0
BOUNDS_HI = [1e8, 1e5,  300.0, 200.0]


def fit(tg, Tg, Pg, dt, T_amb, phases):
  mc0, k0, th0 = initial_guess(tg, Tg, Pg, dt, T_amb, phases)
  lo, hi = BOUNDS_LO, BOUNDS_HI

  def resid(x):
    return simulate(x, tg, Pg, dt, T_amb)[0] - Tg

  best = None
  for L0 in (2.0, 10.0, 30.0):
    x0 = [mc0, k0, L0, th0]
    res = least_squares(resid, x0, bounds=(lo, hi),
                        x_scale=[mc0, k0, 10.0, 5.0],
                        method='trf', max_nfev=2000)
    if best is None or res.cost < best.cost:
      best = res

  x = best.x
  r = best.fun
  n, p = len(r), len(x)
  dof = max(n - p, 1)
  s2 = float(np.sum(r ** 2) / dof)
  J = best.jac
  try:
    cov = s2 * np.linalg.pinv(J.T @ J)
    se = np.sqrt(np.clip(np.diag(cov), 0.0, None))
  except np.linalg.LinAlgError:
    se = np.full(p, np.nan)
  return x, se, np.sqrt(np.mean(r ** 2)), best.success


def fit_free_ambient(tg, Tg, Pg, dt, x_fixed, T_amb):
  """Cross-check: refit with the ambient temperature as a fifth free
  parameter, starting from the fixed-ambient solution.  Returns
  (mc, k_loss, L, θ0, T_amb_free, rmse).  Used only for diagnostics — the
  reported parameters always come from the fixed-ambient fit."""
  def resid(x):
    return simulate(x[:4], tg, Pg, dt, x[4])[0] - Tg
  x0 = list(x_fixed) + [T_amb]
  res = least_squares(resid, x0,
                      bounds=(BOUNDS_LO + [-20.0], BOUNDS_HI + [80.0]),
                      x_scale=[x_fixed[0], x_fixed[1], 10.0, 5.0, 5.0],
                      method='trf', max_nfev=2000)
  return res.x, float(np.sqrt(np.mean(res.fun ** 2)))


# ── Corner diagnostics ────────────────────────────────────────────────────────

def corner_dead_time(tg, Tg, Pg, dt, phases):
  """Dead time read off the first power-on corner: the time from power-on
  until the measured slope reaches half the steady ramp slope.  Returns
  (L_corner, steady_slope_c_per_s) or None."""
  heats = [p for p in phases if p['kind'] == 'heat' and p['i0'] > 0]
  if not heats:
    return None
  i0, i1 = heats[0]['i0'], heats[0]['i1']
  if (i1 - i0) * dt < 120.0:
    return None
  w = max(int(round(5.0 / dt)), 1)
  slope = np.full(len(Tg), np.nan)
  slope[w:-w] = (Tg[2 * w:] - Tg[:-2 * w]) / (2.0 * w * dt)
  q = (i1 - i0) // 4
  steady = float(np.nanmedian(slope[i0 + q:i1 - q]))
  if not steady > 0:
    return None
  t_on = tg[i0] - dt / 2.0
  for i in range(i0, i1 - 1):
    if slope[i] >= 0.5 * steady and slope[i + 1] >= 0.5 * steady:
      return float(tg[i] - t_on), steady
  return None


def mixing_offset(tg, Tg, Pg, dt, phases):
  """How far above the bulk the probe reads while heating.  Right after
  power-off the reading peaks (probe lag) and then falls quickly to the
  bulk before settling on the slow decay.  Extrapolate the slow decay
  (3–10 min after power-off) back to the power-off instant; the peak minus
  that line is the offset.  Returns (offset_c, P_heat_w) or None."""
  for j in range(1, len(phases)):
    ph, prev = phases[j], phases[j - 1]
    if ph['kind'] != 'cool' or prev['kind'] != 'heat':
      continue
    i_off = ph['i0']
    if (ph['i1'] - i_off + 1) * dt < 600.0:
      return None
    a = i_off + int(180.0 / dt)
    b = i_off + int(600.0 / dt)
    coef = np.polyfit(tg[a:b], Tg[a:b], 1)
    t_off = tg[i_off] - dt / 2.0
    peak = float(np.max(Tg[i_off:i_off + int(30.0 / dt)]))
    P_heat = float(np.mean(Pg[prev['i0']:prev['i1'] + 1]))
    return peak - float(np.polyval(coef, t_off)), P_heat
  return None


# ── Tuning rules ──────────────────────────────────────────────────────────────
# All rules return continuous-time (Kp [W/°C], Ki [W/°C/s], Kd [W·s/°C]).

def tune_simc(K, tau, L, tau_c=None):
  """SIMC PI tuning (Skogestad, 2003): IMC with the integral time capped.

      Kp = τ / (K·(τc + L))        Ti = min(τ, 4·(τc + L))

  τc is the desired closed-loop time constant; smaller → more aggressive.
  The cap on Ti is what makes the rule usable on a lag-dominant plant
  (τ ≫ L, e.g. an insulated kettle where τ is hours): plain IMC-PI sets
  Ti = τ and the integrator becomes uselessly slow.
  Default τc = max(3·L, 120 s): smooth rather than tight, because the corner
  transients of a real kettle (mixing offset) are not in the model.
  """
  if tau_c is None:
    tau_c = max(3.0 * L, 120.0)
  Kp = tau / (K * (tau_c + L))
  Ti = min(tau, 4.0 * (tau_c + L))
  return Kp, Kp / Ti, 0.0, {'tau_c_s': tau_c, 'Ti_s': Ti}


def tune_zn(K, tau, L):
  """Ziegler–Nichols open-loop (reaction curve) PID.

  Aggressive; often produces significant overshoot on thermal processes.
  Use as an upper bound on aggressiveness, not a target.
  """
  L = max(L, 1e-3)
  Kp = 1.2 * tau / (K * L)
  Ti = 2.0 * L
  Td = 0.5 * L
  return Kp, Kp / Ti, Kp * Td, {}


def tune_cc(K, tau, L):
  """Cohen–Coon PID.

  Best suited to 0.1 < L/τ < 1.0.  More accurate than ZN for processes
  with significant dead time relative to the time constant.
  """
  L = max(L, 1e-3)
  r = L / tau
  Kp = (1.35 / (K * r)) * (1.0 + 0.185 * r)
  Ti = L * (2.5 - 2.0 * r) / max(1.0 - 0.39 * r, 0.01)
  Td = 0.37 * L / (1.0 - 0.81 * r) if r < 0.81 else 0.0
  return Kp, Kp / Ti, Kp * Td, {}


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
  ap = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter,
  )
  ap.add_argument('csv', help='Path to bibby CSV log file')
  ap.add_argument('--ambient', type=float, required=True, metavar='C',
                  help='Room / ambient temperature during the test [°C] (required)')
  ap.add_argument('--volume-l', type=float, default=None, metavar='L',
                  help='Water volume [L]; cross-checks the delivered watts')
  ap.add_argument('--dt', type=float, default=1.0, metavar='SEC',
                  help='Resample period for the fit (default 1.0 s)')
  ap.add_argument('--rule', choices=['simc', 'zn', 'cc', 'all'], default='simc',
                  help='Tuning rule(s) to print (default: simc)')
  ap.add_argument('--lambda', dest='lam', type=float, default=None, metavar='SEC',
                  help='SIMC closed-loop time constant τc [s] (default: max(3L, 120))')
  ap.add_argument('-o', dest='out', default=None, metavar='FILE',
                  help='Save plot to FILE instead of showing interactively')
  args = ap.parse_args()
  T_amb = args.ambient

  # ── Load ─────────────────────────────────────────────────────────────────
  df = load_csv(args.csv)
  t_raw = df['t_monotonic_s'].values
  dt_all = np.diff(t_raw)
  ts = float(np.median(dt_all[dt_all > 0]))     # controller sample period
  fs = 1.0 / ts
  print(f"Log:         {args.csv}")
  print(f"Samples:     {len(df)}  over {t_raw[-1] - t_raw[0]:.0f} s")
  print(f"Sample rate: {fs:.2f} Hz  (T_s = {ts * 1000:.1f} ms)")
  print(f"Ambient:     {T_amb:.2f} °C  (given)")

  dt = args.dt
  tg, Tg, Pg = resample(df, dt)
  print(f"Fit grid:    {len(tg)} points at {dt:g} s")

  # ── Phases ───────────────────────────────────────────────────────────────
  phases = find_phases(tg, Pg, dt)
  print()
  print("── Power phases (from p_delivered_w) ──────────────────────────")
  for ph in phases:
    i0, i1 = ph['i0'], ph['i1']
    dur = (i1 - i0 + 1) * dt
    print(f"  {ph['kind']:4s}  {tg[i0]:7.0f} – {tg[i1]:7.0f} s  ({dur / 60:5.1f} min)"
          f"  mean {np.mean(Pg[i0:i1 + 1]):6.0f} W"
          f"  T {Tg[i0]:5.1f} → {Tg[i1]:5.1f} °C")

  heats = [p for p in phases if p['kind'] == 'heat']
  cools = [p for p in phases if p['kind'] == 'cool' and p['i0'] > 0]
  cool_drop = max((Tg[p['i0']] - Tg[p['i1']] for p in cools), default=0.0)
  if not heats:
    sys.exit("No heating phase found in the log.")

  # ── Fit ──────────────────────────────────────────────────────────────────
  x, se, rmse, ok = fit(tg, Tg, Pg, dt, T_amb, phases)
  mc, k_loss, L, th0 = x
  Tm, Tk = simulate(x, tg, Pg, dt, T_amb)
  K = 1.0 / k_loss
  tau = mc / k_loss
  rel = np.where(x != 0, se / np.abs(x), np.nan)

  def pm(v, s, fmt):
    return f"{v:{fmt}} ± {s:{fmt}}" if np.isfinite(s) else f"{v:{fmt}}"

  print()
  print("── Lumped kettle model  m·c·dT/dt = P − k_loss·(T − T_amb) ──")
  print(f"  m·c     = {pm(mc / 1000, se[0] / 1000, '.2f')}  kJ/°C"
        f"   (≈ {mc / 4186:.1f} L of water)")
  print(f"  k_loss  = {pm(k_loss, se[1], '.2f')}  W/°C   (heat loss per °C above ambient)")
  print(f"  L       = {pm(L, se[2], '.1f')}  s     (probe lag + filter delay)")
  print(f"  T(0)    = {T_amb + th0:.2f} °C   ({th0:+.2f} °C vs ambient at log start)")
  print(f"  Fit RMSE = {rmse:.3f} °C" + ('' if ok else '   (solver did not report convergence)'))
  print("  (± are 1-σ standard errors from the fit Jacobian; residuals are")
  print("   autocorrelated so treat them as relative, not absolute.)")

  # Residual by phase
  r = Tm - Tg
  parts = []
  for ph in phases:
    seg = r[ph['i0']:ph['i1'] + 1]
    parts.append(f"{ph['kind']} {np.sqrt(np.mean(seg ** 2)):.3f}")
  print(f"  RMSE by phase: " + ', '.join(parts) + " °C")

  # Cross-check with ambient free
  xf, rmse_f = fit_free_ambient(tg, Tg, Pg, dt, x, T_amb)
  amb_gap = xf[4] - T_amb
  print(f"  Cross-check (ambient free): T_amb = {xf[4]:.1f} °C (given {T_amb:.1f}),"
        f" k_loss = {xf[1]:.2f} W/°C, RMSE {rmse_f:.3f} °C")

  warn = []
  if abs(amb_gap) > 2.0 and rmse_f < 0.8 * rmse:
    warn.append(f"The data prefer an ambient of {xf[4]:.1f} °C, {amb_gap:+.1f} °C from the\n"
                "  value given.  Re-check the room reading; if the kettle really\n"
                "  rests that far from room temperature (pump heat, sun), use the\n"
                "  free-fit value for --ambient and feedforward.ambient_c.")
  if not cools or cool_drop < 2.0:
    warn.append("No cooling phase with ≥ 2 °C of drop: k_loss is weakly constrained.\n"
                "  Let the kettle cool longer after the heat; do not enable\n"
                "  feedforward from this fit.")
  if np.isfinite(rel[1]) and rel[1] > 0.25:
    warn.append(f"k_loss standard error is {rel[1] * 100:.0f} % of its value — poorly\n"
                "  determined.  Extend the cooling phase.")
  if np.isfinite(rel[0]) and rel[0] > 0.1:
    warn.append(f"m·c standard error is {rel[0] * 100:.0f} % of its value.  Check that the\n"
                "  heating phase has a clear ramp and the pump was running.")
  if float(Tg.max()) > 90.0:
    warn.append("Temperature exceeded 90 °C: evaporative loss makes k_loss\n"
                "  temperature-dependent up there; the fit reflects an average.")
  if rmse > 0.5:
    warn.append(f"RMSE {rmse:.2f} °C is large for a well-stirred kettle.  Check\n"
                "  mixing, the ambient value, and that the lid stayed put.")
  for w in warn:
    print(f"\n  WARNING: {w}")

  # ── Corner diagnostics ───────────────────────────────────────────────────
  print()
  print("── Corners (not in the lumped model) ──────────────────────────")
  L_fit = L
  cd = corner_dead_time(tg, Tg, Pg, dt, phases)
  if cd is not None:
    L_corner, steady = cd
    print(f"  Dead time from the power-on corner: {L_corner:.1f} s  "
          f"(slope reaches half of {steady * 60:.2f} °C/min)")
    if L_corner > L_fit + 2.0:
      print(f"  The least-squares L ({L_fit:.1f} s) is smaller — corner transients pull it")
      print(f"  toward zero; the corner value is used for tuning.")
    L = max(L_fit, L_corner)
  else:
    print("  No usable power-on corner (heating phase too short) — L from the fit only.")
  L_filt = 2 * 39 / (2 * fs)
  if L < L_filt:
    print(f"  L floored at the boxcar group delay ({L_filt:.2f} s).")
    L = L_filt
  mo = mixing_offset(tg, Tg, Pg, dt, phases)
  if mo is not None:
    off, P_heat = mo
    P_hold = max((67.0 - T_amb) * k_loss, 0.0)
    print(f"  Mixing offset: probe read {off:+.2f} °C vs the bulk while heating at "
          f"{P_heat:.0f} W")
    print(f"  (the fast settle right after power-off). At the ~{P_hold:.0f} W holding "
          f"power near 67 °C")
    print(f"  that scales to {off * P_hold / max(P_heat, 1.0):+.3f} °C — ignorable for control.")

  # ── Volume cross-check ───────────────────────────────────────────────────
  if args.volume_l is not None:
    mc_water = args.volume_l * 4186.0
    ratio = mc / mc_water
    print()
    print("── Delivered-watts cross-check ────────────────────────────────")
    print(f"  m·c fit / m·c of {args.volume_l:.1f} L water = {mc / 1000:.1f} / "
          f"{mc_water / 1000:.1f} kJ/°C = {ratio:.3f}")
    print(f"  The kettle, elements, pump and hoses add a few kJ/°C on top of the water,")
    print(f"  so a ratio a little above 1 is expected.  Anything beyond that means the")
    print(f"  elements delivered about {100.0 / ratio:.0f} % of what bibby.ini calls their")
    print(f"  rated watts (mains below the rating voltage, element resistance tolerance,")
    print(f"  SSR drop).  The gains and feedforward above are still right: the controller,")
    print(f"  the log and this fit all use the same nominal watts, so the factor cancels")
    print(f"  in the loop.  Only the displayed/logged watts, the m·c readout and the flux")
    print(f"  cap are mislabeled.  If you correct element1_watts / element2_watts by a")
    print(f"  factor s (measure V and R: P = V²/R, or s = {1.0 / ratio:.2f} if the volume is")
    print(f"  trusted), scale kp and ki by s and process_gain_c by 1/s — or rerun the")
    print(f"  test.  Changing the ratings alone makes the loop 1/s more aggressive.")

  print()
  print("── FOPDT equivalents ──────────────────────────────────────────")
  print(f"  K   = 1/k_loss   = {K * 1000:.3f}  °C per kW")
  print(f"  τ   = m·c/k_loss = {tau:.0f}  s  ({tau / 60:.1f} min)")
  print(f"  L                = {L:.1f}  s  (used for tuning)")
  print(f"  L/τ              = {L / tau:.4f}"
        f"  ({'easy' if L/tau < 0.3 else 'moderate' if L/tau < 1.0 else 'difficult'}"
        f" to control)")
  if heats:
    P_heat = float(np.mean(Pg[heats[0]['i0']:heats[0]['i1'] + 1]))
    print(f"  Ramp rate at {P_heat:.0f} W ≈ {P_heat / mc * 60:.2f} °C/min")

  # ── Tuning rules ──────────────────────────────────────────────────────────
  rules: dict = {}
  if args.rule in ('simc', 'all'):
    rules['SIMC-PI'] = tune_simc(K, tau, L, tau_c=args.lam)
  if args.rule in ('zn', 'all'):
    rules['ZN-PID'] = tune_zn(K, tau, L)
  if args.rule in ('cc', 'all'):
    rules['CC-PID'] = tune_cc(K, tau, L)

  def rule_note(info):
    if 'tau_c_s' in info:
      return f"  (τc={info['tau_c_s']:.0f} s, Ti={info['Ti_s']:.0f} s)"
    return ''

  print()
  print("── Continuous-time gains (watts-based) ────────────────────────")
  for name, (Kp, Ki, Kd, info) in rules.items():
    print(f"  {name:10s}  Kp={Kp:.2f} W/°C  Ki={Ki:.5f} W/°C/s  Kd={Kd:.2f} W·s/°C{rule_note(info)}")
  if L / tau < 0.02 and any(n in rules for n in ('ZN-PID', 'CC-PID')):
    print("  NOTE: L/τ is tiny — the ZN/CC reaction-curve rules do not apply to a")
    print("  lag-dominant plant; their gains above are not usable.")

  print()
  print(f"── bibby.ini gains  (T_s = {ts * 1000:.1f} ms; per-sample integrator) ──")
  print(f"  kp_ini = Kp_c            [W/°C, unchanged]")
  print(f"  ki_ini = Ki_c × T_s      [integral accumulates error once per sample]")
  print(f"  kd_ini = Kd_c / T_s      [derivative is a raw per-sample delta]")
  print()
  for name, (Kp, Ki, Kd, info) in rules.items():
    print(f"  ── {name}{rule_note(info)}")
    print(f"  [pid]")
    print(f"  kp = {Kp:.2f}")
    print(f"  ki = {Ki * ts:.6f}")
    print(f"  kd = {Kd / ts:.2f}")
    print()
  print("  Anti-windup needs no extra setup: conditional integration plus the")
  print("  out_max/ki backstop in control.c bound the integrator automatically.")
  print()

  # ── Feedforward block ─────────────────────────────────────────────────────
  print("── bibby.ini feedforward  (holding-power feedforward, README §9.6) ──")
  print(f"  [feedforward]")
  print(f"  process_gain_c = {K:.6f}   # 1/k_loss, °C per W")
  print(f"  ambient_c      = {T_amb:.2f}      # the --ambient you measured")
  print()
  print(f"  Holding power at 67 °C: (67 − {T_amb:.1f}) × k_loss = {(67.0 - T_amb) * k_loss:.0f} W")
  if warn and any('k_loss' in w for w in warn):
    print("  (k_loss is not trustworthy from this log — leave process_gain_c = 0)")
  print()

  # ── Plot ──────────────────────────────────────────────────────────────────
  fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
  fig.suptitle(f"bibby plant identification — {args.csv}", fontsize=10)

  ax1 = axes[0]
  for ph in phases:
    if ph['kind'] == 'heat':
      ax1.axvspan(tg[ph['i0']], tg[ph['i1']], color='orange', alpha=0.08, lw=0)
  ax1.plot(tg, Tg, color='steelblue', lw=1.5, label='temp_filt_c (measured)')
  ax1.plot(tg, Tm, '--', color='tomato', lw=1.5,
           label=(f'model  m·c={mc / 1000:.1f} kJ/°C, k_loss={k_loss:.1f} W/°C, '
                  f'L={L:.0f} s  (RMSE {rmse:.3f} °C)'))
  ax1.plot(tg, Tk, ':', color='gray', lw=0.8, label='kettle (undelayed)')
  ax1.axhline(T_amb, color='gray', ls='-.', lw=0.8, label=f'ambient {T_amb:.1f} °C')
  ax1.set_ylabel('Temperature (°C)')
  ax1.legend(fontsize=7)
  ax1.grid(True, alpha=0.3)

  ax2 = axes[1]
  ax2.plot(tg, Pg, color='darkorange', lw=1.2, label='p_delivered_w')
  ax2.set_ylabel('Power (W)')
  ax2.legend(fontsize=7)
  ax2.grid(True, alpha=0.3)

  ax3 = axes[2]
  ax3.plot(tg, Tg - Tm, color='purple', lw=1.0,
           label=f'measured − model (RMSE={rmse:.3f} °C)')
  ax3.axhline(0, color='gray', ls='--', lw=0.8)
  ax3.axhline(+rmse, color='purple', ls=':', lw=0.8)
  ax3.axhline(-rmse, color='purple', ls=':', lw=0.8)
  ax3.set_ylabel('Residual (°C)')
  ax3.set_xlabel('Time from log start (s)')
  ax3.legend(fontsize=7)
  ax3.grid(True, alpha=0.3)

  plt.tight_layout()

  if args.out:
    plt.savefig(args.out, dpi=150)
    print(f"Plot saved to {args.out}")
  else:
    plt.show()


if __name__ == '__main__':
  main()
