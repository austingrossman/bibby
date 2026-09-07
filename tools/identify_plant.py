#!/usr/bin/env python3
"""
identify_plant.py — BIAB kettle plant identification and PID tuning
--------------------------------------------------------------------
Loads a bibby CSV log from an open-loop step test, fits a First-Order Plus
Dead Time (FOPDT) model to the filtered temperature response, and computes
PID gains using three classical tuning rules.  Prints a ready-to-paste
bibby.ini [pid] + [feedforward] snippet and generates a summary plot.

The bibby control law works in WATTS: the PID output is a power demand
that the equal-flux splitter turns into per-element duties.  All gains
here are therefore watts-based:

    kp  [W / °C]
    ki  [W / °C per sample]   (the integrator accumulates raw error/sample)
    kd  [W·sample / °C]       (the derivative is a raw per-sample delta)

Requirements:
    pip install numpy scipy matplotlib pandas

Usage:
    python3 identify_plant.py <log.csv> [OPTIONS]

Options:
    --t-start SEC   Seconds from log start to begin the analysis window.
    --t-end   SEC   Seconds from log start to end the analysis window.
    --watts   W     Step power in watts.  If omitted: median of
                    p_delivered_w (new logs) or duty-derived power
                    (old logs, see --e1-watts/--e2-watts).
    --e1-watts W    Element 1 rating for duty→watts conversion of
    --e2-watts W    pre-watts logs (defaults: 5000 / 5500 per bibby.ini).
    --rule    RULE  Tuning rules to print: imc | zn | cc | all  (default: all)
    --lambda  SEC   IMC closed-loop time constant λ, seconds.  Default: auto
                    (max(0.25·L, 0.2·τ), producing a moderately-fast loop).
    -o FILE         Save plot to FILE instead of displaying interactively.

Workflow
--------
1.  Collect a step-response log (README §9.4): manual mode, fixed watts,
    starting from a settled temperature.
2.  Run this script with the log file, trimming the window to the step.
3.  Copy the recommended [pid] and [feedforward] blocks into bibby.ini.
4.  Validate with a setpoint step in auto mode (README §9.5).

Physical cross-check
--------------------
The FOPDT parameters map onto the lumped kettle model
    m·c dT/dt = P − k_loss (T − T_amb):
    k_loss = 1 / K      [W/°C]
    m·c    = τ / K      [J/°C]   (≈ 4186 J/°C per litre of water)
The script prints both — sanity-check m·c against the actual water volume.
"""

import argparse
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


# ── CSV loading ───────────────────────────────────────────────────────────────

def load_csv(path: str, e1_watts: float, e2_watts: float) -> pd.DataFrame:
  """Load a bibby log; synthesize a power_w column for either CSV format."""
  df = pd.read_csv(path, comment='#')
  df.columns = df.columns.str.strip()
  required = ['t_monotonic_s', 'temp_filt_c']
  missing = [c for c in required if c not in df.columns]
  if missing:
    sys.exit(f"CSV is missing required columns: {missing}")

  if 'p_delivered_w' in df.columns:
    # New (watts) format: prefer delivered power; fall back to demand where
    # delivery is zero because ZC sim / mains was off.
    p = df['p_delivered_w'].astype(float)
    if 'p_demand_w' in df.columns and float(p.abs().max()) < 1.0:
      p = df['p_demand_w'].astype(float)
      print("NOTE: p_delivered_w is all zero; using p_demand_w instead.")
    df['power_w'] = p
  elif 'duty1' in df.columns and 'duty2' in df.columns:
    df['power_w'] = (df['duty1'].astype(float) * e1_watts +
                     df['duty2'].astype(float) * e2_watts)
    print(f"Old-format log: power = duty1×{e1_watts:.0f} + duty2×{e2_watts:.0f} W")
  else:
    sys.exit("CSV has neither p_delivered_w nor duty1/duty2 — cannot derive power.")

  df = df.dropna(subset=['t_monotonic_s', 'temp_filt_c', 'power_w'])
  df = df.sort_values('t_monotonic_s').reset_index(drop=True)
  return df


# ── FOPDT model ───────────────────────────────────────────────────────────────
#
#   T(t) = T0  +  K * P_step * (1 − exp(−(t − L) / τ))    for t > L
#   T(t) = T0                                              for t ≤ L
#
# t is time relative to the start of the analysis window.
# K   — process gain [°C / W]
# tau — dominant thermal time constant [s]
# L   — effective dead time [s]  (probe lag + filter group delay)

def _fopdt(t_rel, K, tau, L, T0, p_step):
  out = np.full_like(t_rel, T0, dtype=float)
  mask = t_rel > L
  out[mask] = T0 + K * p_step * (1.0 - np.exp(-(t_rel[mask] - L) / np.maximum(tau, 1e-3)))
  return out

# Globals set before each curve_fit call so the closure captures them.
_ref_T0    = 0.0
_ref_pstep = 1.0

def _fopdt_fit(t_rel, K, tau, L):
  return _fopdt(t_rel, K, tau, L, _ref_T0, _ref_pstep)


# ── Tuning rules ──────────────────────────────────────────────────────────────
# All rules return continuous-time (Kp [W/°C], Ki [W/°C/s], Kd [W·s/°C]).

def tune_imc(K, tau, L, lam=None):
  """IMC-PI tuning (Rivera, Morari & Skogestad, 1986).

  λ is the closed-loop time constant.  Smaller λ → more aggressive.
  Rule of thumb: λ = max(0.25·L, 0.2·τ) gives a fast but stable loop.
  λ = L…3·L is a conservative starting point for a sluggish plant.
  """
  if lam is None:
    lam = max(0.25 * L, 0.2 * tau)
  Kp = tau / (K * (lam + L))
  Ki = Kp / tau               # = 1 / (K*(lam+L))
  Kd = 0.0
  return Kp, Ki, Kd, {'lambda_s': lam}


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
  global _ref_T0, _ref_pstep

  ap = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter,
  )
  ap.add_argument('csv', help='Path to bibby CSV log file')
  ap.add_argument('--t-start', type=float, default=None, metavar='SEC',
                  help='Analysis window start (seconds from log beginning)')
  ap.add_argument('--t-end', type=float, default=None, metavar='SEC',
                  help='Analysis window end (seconds from log beginning)')
  ap.add_argument('--watts', type=float, default=None, metavar='W',
                  help='Step power [W]; omit to infer from data median')
  ap.add_argument('--e1-watts', type=float, default=5000.0, metavar='W',
                  help='Element 1 rating for old duty-format logs')
  ap.add_argument('--e2-watts', type=float, default=5500.0, metavar='W',
                  help='Element 2 rating for old duty-format logs')
  ap.add_argument('--rule', choices=['imc', 'zn', 'cc', 'all'], default='all',
                  help='Tuning rule(s) to print (default: all)')
  ap.add_argument('--lambda', dest='lam', type=float, default=None, metavar='SEC',
                  help='IMC closed-loop time constant λ [s] (default: auto)')
  ap.add_argument('-o', dest='out', default=None, metavar='FILE',
                  help='Save plot to FILE instead of showing interactively')
  args = ap.parse_args()

  # ── Load and prepare ─────────────────────────────────────────────────────
  df = load_csv(args.csv, args.e1_watts, args.e2_watts)
  t_log_start = df['t_monotonic_s'].iloc[0]
  df['t'] = df['t_monotonic_s'] - t_log_start

  dt_all = np.diff(df['t'].values)
  dt = float(np.median(dt_all[dt_all > 0]))
  fs = 1.0 / dt
  print(f"Log:         {args.csv}")
  print(f"Samples:     {len(df)}  over {df['t'].iloc[-1]:.0f} s")
  print(f"Sample rate: {fs:.2f} Hz  (dt = {dt * 1000:.1f} ms)")

  # ── Window selection ──────────────────────────────────────────────────────
  t_start = args.t_start if args.t_start is not None else df['t'].iloc[0]
  t_end   = args.t_end   if args.t_end   is not None else df['t'].iloc[-1]
  mask = (df['t'] >= t_start) & (df['t'] <= t_end)
  dfw = df[mask].copy().reset_index(drop=True)
  if len(dfw) < 30:
    sys.exit(f"Window contains only {len(dfw)} samples — widen with --t-start/--t-end.")

  t_rel  = dfw['t'].values - dfw['t'].iloc[0]
  T_meas = dfw['temp_filt_c'].values
  T0     = float(T_meas[0])

  # ── Step power ────────────────────────────────────────────────────────────
  if args.watts is not None:
    p_step = float(args.watts)
    print(f"Step power:  {p_step:.0f} W  (specified)")
  else:
    p_on = dfw['power_w'].values
    p_step = float(np.median(p_on[p_on > 0.05 * max(p_on.max(), 1.0)]))
    print(f"Step power:  {p_step:.0f} W  (inferred from log median of nonzero power)")

  if p_step < 50.0:
    sys.exit("Inferred step power is near zero.  Command watts in manual mode "
             "before the test, or specify --watts.")

  # ── FOPDT fit ─────────────────────────────────────────────────────────────
  _ref_T0    = T0
  _ref_pstep = p_step

  dT_obs    = T_meas[-1] - T0
  K_guess   = dT_obs / p_step if dT_obs > 0.5 else 0.01
  tau_guess = t_rel[-1] * 0.5
  L_guess   = t_rel[-1] * 0.05

  bounds_lo = [1e-6, dt,              0.0]
  bounds_hi = [1.0,  t_rel[-1] * 5.0, t_rel[-1] * 0.5]

  fit_ok = False
  try:
    popt, pcov = curve_fit(
      _fopdt_fit, t_rel, T_meas,
      p0=[K_guess, tau_guess, L_guess],
      bounds=(bounds_lo, bounds_hi),
      maxfev=20000,
    )
    K, tau, L = popt
    T_fit  = _fopdt_fit(t_rel, K, tau, L)
    rmse   = float(np.sqrt(np.mean((T_meas - T_fit) ** 2)))
    fit_ok = True
  except RuntimeError as exc:
    print(f"\nWARNING: FOPDT curve fit failed: {exc}")
    print("Try narrowing the window to just the step, or check --watts.")
    K, tau, L = K_guess, tau_guess, L_guess
    T_fit = _fopdt_fit(t_rel, K, tau, L)
    rmse  = float(np.sqrt(np.mean((T_meas - T_fit) ** 2)))

  k_loss = 1.0 / K if K > 0 else float('inf')
  mc     = tau / K if K > 0 else float('inf')

  print()
  print("── FOPDT model ────────────────────────────────────────────────")
  print(f"  K   (process gain)  = {K * 1000:.3f}  °C per kW")
  print(f"  τ   (time constant) = {tau:.1f}  s  ({tau / 60:.1f} min)")
  print(f"  L   (dead time)     = {L:.1f}  s")
  print(f"  L/τ (relative DT)   = {L / tau:.4f}"
        f"  ({'easy' if L/tau < 0.3 else 'moderate' if L/tau < 1.0 else 'difficult'}"
        f" to control)")
  print(f"  Fit RMSE            = {rmse:.3f}  °C"
        + ('' if fit_ok else '  (fit did not converge)'))
  print()
  print("── Physical cross-check (lumped kettle model) ─────────────────")
  print(f"  k_loss = 1/K  = {k_loss:.1f}  W/°C   (heat loss per °C above ambient)")
  print(f"  m·c    = τ/K  = {mc / 1000:.1f}  kJ/°C  (≈ {mc / 4186:.1f} L of water)")
  print(f"  Initial slope = {p_step * K / max(tau, 1e-9) * 60:.2f} °C/min at {p_step:.0f} W")
  if tau > 0.8 * t_rel[-1]:
    print()
    print("  NOTE: τ approaches the window length — the K and τ estimates are")
    print("  weakly constrained (only their ratio, the initial slope, is solid).")
    print("  m·c is trustworthy; k_loss and steady-state K are not. Prefer a")
    print("  longer test, or gains from a log that approaches steady state.")

  # ── Tuning rules ──────────────────────────────────────────────────────────
  rules: dict = {}
  if args.rule in ('imc', 'all'):
    rules['IMC-PI'] = tune_imc(K, tau, L, lam=args.lam)
  if args.rule in ('zn', 'all'):
    rules['ZN-PID'] = tune_zn(K, tau, L)
  if args.rule in ('cc', 'all'):
    rules['CC-PID'] = tune_cc(K, tau, L)

  print()
  print("── Continuous-time gains (watts-based) ────────────────────────")
  for name, (Kp, Ki, Kd, info) in rules.items():
    note = ('  (λ=' + f"{info['lambda_s']:.1f}" + ' s)') if 'lambda_s' in info else ''
    print(f"  {name:10s}  Kp={Kp:.2f} W/°C  Ki={Ki:.5f} W/°C/s  Kd={Kd:.2f} W·s/°C{note}")

  print()
  print(f"── bibby.ini gains  (dt = {dt * 1000:.1f} ms; per-sample integrator) ──")
  print(f"  kp_ini = Kp_c            [W/°C, unchanged]")
  print(f"  ki_ini = Ki_c × dt       [integral accumulates error once per sample]")
  print(f"  kd_ini = Kd_c / dt       [derivative is a raw per-sample delta]")
  print()

  for name, (Kp, Ki, Kd, info) in rules.items():
    note = ('  (λ=' + f"{info['lambda_s']:.1f}" + ' s)') if 'lambda_s' in info else ''
    print(f"  ── {name}{note}")
    print(f"  [pid]")
    print(f"  kp = {Kp:.2f}")
    print(f"  ki = {Ki * dt:.6f}")
    print(f"  kd = {Kd / dt:.2f}")
    print()
  print("  Anti-windup needs no extra setup: conditional integration plus the")
  print("  out_max/ki backstop in control.c bound the integrator automatically.")
  print()

  # ── Feedforward block (rule-independent) ──────────────────────────────────
  print("── bibby.ini feedforward  (holding-power feedforward, README §9.6) ──")
  print(f"  [feedforward]")
  print(f"  process_gain_c = {K:.6f}   # identified K, °C per W")
  print(f"  ambient_c      = {T0:.2f}      # window start temp; use cold-soak/room temp")
  print()
  hold_67 = (67.0 - T0) / K if K > 0 else 0.0
  print(f"  Holding power at e.g. 67 °C: (67 − {T0:.1f}) / K = {hold_67:.0f} W")
  print()

  # ── Plot ──────────────────────────────────────────────────────────────────
  fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
  fig.suptitle(f"bibby plant identification — {args.csv}", fontsize=10)

  ax1 = axes[0]
  ax1.plot(t_rel, T_meas, color='steelblue', lw=1.5, label='temp_filt_c (measured)')
  if fit_ok:
    label_fit = (f'FOPDT fit  K={K * 1000:.2f} °C/kW, '
                 f'τ={tau:.0f} s ({tau/60:.1f} min), L={L:.0f} s')
    ax1.plot(t_rel, T_fit, '--', color='tomato', lw=1.5, label=label_fit)
    ax1.axvline(L, color='gray', ls=':', lw=1.0)
    ax1.annotate(f'L={L:.0f} s', xy=(L, T0), xytext=(L + 2, T0 + 0.5),
                 fontsize=7, color='gray')
    T_63 = T0 + K * p_step * 0.632
    if T_63 < T_meas.max():
      ax1.axhline(T_63, color='sandybrown', ls=':', lw=0.8)
      ax1.annotate(f'63.2 %  T={T_63:.1f}°C', xy=(t_rel[-1], T_63),
                   xytext=(t_rel[-1] * 0.6, T_63 + 0.3), fontsize=7,
                   color='sandybrown')
  ax1.set_ylabel('Temperature (°C)')
  ax1.legend(fontsize=7)
  ax1.grid(True, alpha=0.3)

  ax2 = axes[1]
  ax2.plot(t_rel, dfw['power_w'].values, color='darkorange', lw=1.2, label='power_w')
  ax2.axhline(p_step, color='gray', ls=':', lw=0.8, label=f'step = {p_step:.0f} W')
  ax2.set_ylabel('Power (W)')
  ax2.legend(fontsize=7)
  ax2.grid(True, alpha=0.3)

  ax3 = axes[2]
  residuals = T_meas - T_fit
  ax3.plot(t_rel, residuals, color='purple', lw=1.0,
           label=f'residuals (RMSE={rmse:.3f} °C)')
  ax3.axhline(0, color='gray', ls='--', lw=0.8)
  ax3.axhline(+rmse, color='purple', ls=':', lw=0.8)
  ax3.axhline(-rmse, color='purple', ls=':', lw=0.8)
  ax3.set_ylabel('Residual (°C)')
  ax3.set_xlabel('Time from window start (s)')
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
