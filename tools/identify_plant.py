#!/usr/bin/env python3
"""
identify_plant.py — BIAB kettle plant identification and PID tuning
--------------------------------------------------------------------
Loads a bibby CSV log from an open-loop step test, fits a First-Order Plus
Dead Time (FOPDT) model to the filtered temperature response, and computes
PID gains using three classical tuning rules.  Prints a ready-to-paste
bibby.ini [pid] snippet and generates a summary plot.

Requirements:
    pip install numpy scipy matplotlib pandas

Usage:
    python3 identify_plant.py <log.csv> [OPTIONS]

Options:
    --t-start SEC   Seconds from log start to begin the analysis window.
                    Defaults to the start of the log.
    --t-end   SEC   Seconds from log start to end the analysis window.
                    Defaults to the end of the log.
    --duty    D     Nominal step duty applied [0–1].  If omitted the median
                    of duty1 in the window is used.
    --rule    RULE  Tuning rules to print: imc | zn | cc | all  (default: all)
    --lambda  SEC   IMC closed-loop time constant λ, seconds.  Default: auto
                    (max(0.25·L, 0.2·τ), producing a moderately-fast loop).
    -o FILE         Save plot to FILE instead of displaying interactively.

Workflow
--------
1.  Collect a step-response log (see README §9.4).
2.  Run this script with the log file, trimming the window to the step.
3.  Copy the recommended [pid] block into bibby.ini.
4.  Validate with a setpoint step in auto mode (README §9.5).

Gain conventions
----------------
The bibby PID accumulates raw error counts (integral_ += error, not += error*dt)
and computes raw delta derivatives (deriv = error - prev_error, not /dt).
The gains in bibby.ini are therefore scaled by the sample period T_s:

    kp_ini  = Kp_c                  (same as continuous-time proportional)
    ki_ini  = Ki_c * T_s            (integral scaled down by T_s)
    kd_ini  = Kd_c / T_s            (derivative scaled up by 1/T_s)

T_s is estimated from the median inter-sample interval in the log.
"""

import argparse
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


# ── CSV loading ───────────────────────────────────────────────────────────────

def load_csv(path: str) -> pd.DataFrame:
  df = pd.read_csv(path, comment='#')
  df.columns = df.columns.str.strip()
  required = ['t_monotonic_s', 'temp_filt_c', 'duty1', 'duty2']
  missing = [c for c in required if c not in df.columns]
  if missing:
    sys.exit(f"CSV is missing required columns: {missing}")
  df = df.dropna(subset=['t_monotonic_s', 'temp_filt_c'])
  df = df.sort_values('t_monotonic_s').reset_index(drop=True)
  return df


# ── FOPDT model ───────────────────────────────────────────────────────────────
#
#   T(t) = T0  +  K * u_step * (1 − exp(−(t − L) / τ))    for t > L
#   T(t) = T0                                               for t ≤ L
#
# t is time relative to the start of the analysis window.
# K   — process gain [°C / unit duty]
# tau — dominant thermal time constant [s]
# L   — effective dead time [s]  (probe lag + filter group delay)

def _fopdt(t_rel, K, tau, L, T0, u_step):
  """Full FOPDT step response (all parameters explicit)."""
  out = np.full_like(t_rel, T0, dtype=float)
  mask = t_rel > L
  out[mask] = T0 + K * u_step * (1.0 - np.exp(-(t_rel[mask] - L) / np.maximum(tau, 1e-3)))
  return out

# Globals set before each curve_fit call so the closure captures them.
_ref_T0    = 0.0
_ref_ustep = 1.0

def _fopdt_fit(t_rel, K, tau, L):
  """Wrapper for curve_fit: K, τ, L are free; T0 and u_step are fixed."""
  return _fopdt(t_rel, K, tau, L, _ref_T0, _ref_ustep)


# ── Tuning rules ──────────────────────────────────────────────────────────────

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
  if L < 1e-3:
    L = 1e-3
  Kp = 1.2 * tau / (K * L)
  Ti = 2.0 * L
  Td = 0.5 * L
  Ki = Kp / Ti
  Kd = Kp * Td
  return Kp, Ki, Kd, {}


def tune_cc(K, tau, L):
  """Cohen–Coon PID.

  Best suited to 0.1 < L/τ < 1.0.  More accurate than ZN for processes
  with significant dead time relative to the time constant.
  """
  if L < 1e-3:
    L = 1e-3
  r = L / tau
  Kp = (1.35 / (K * r)) * (1.0 + 0.185 * r)
  Ti = L * (2.5 - 2.0 * r) / max(1.0 - 0.39 * r, 0.01)
  if r < 0.81:
    Td = 0.37 * L / (1.0 - 0.81 * r)
  else:
    Td = 0.0
  Ki = Kp / Ti
  Kd = Kp * Td
  return Kp, Ki, Kd, {}


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
  global _ref_T0, _ref_ustep

  ap = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter,
  )
  ap.add_argument('csv', help='Path to bibby CSV log file')
  ap.add_argument('--t-start', type=float, default=None,
                  metavar='SEC',
                  help='Analysis window start (seconds from log beginning)')
  ap.add_argument('--t-end', type=float, default=None,
                  metavar='SEC',
                  help='Analysis window end (seconds from log beginning)')
  ap.add_argument('--duty', type=float, default=None,
                  metavar='D',
                  help='Step duty [0–1]; omit to infer from data median')
  ap.add_argument('--rule',
                  choices=['imc', 'zn', 'cc', 'all'], default='all',
                  help='Tuning rule(s) to print (default: all)')
  ap.add_argument('--lambda', dest='lam', type=float, default=None,
                  metavar='SEC',
                  help='IMC closed-loop time constant λ [s] (default: auto)')
  ap.add_argument('-o', dest='out', default=None,
                  metavar='FILE',
                  help='Save plot to FILE instead of showing interactively')
  args = ap.parse_args()

  # ── Load and prepare ─────────────────────────────────────────────────────
  df = load_csv(args.csv)
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

  # ── Step duty ─────────────────────────────────────────────────────────────
  if args.duty is not None:
    u_step = float(args.duty)
    print(f"Step duty:   {u_step:.3f}  (specified)")
  else:
    # Use duty1 if both elements are commanded equally; average otherwise.
    d1 = dfw['duty1'].values
    d2 = dfw['duty2'].values
    if np.abs(float(np.median(d1)) - float(np.median(d2))) < 0.05:
      u_step = float(np.median(d1))
    else:
      u_step = float(np.median((d1 + d2) / 2.0))
    print(f"Step duty:   {u_step:.3f}  (inferred from log median)")

  if u_step < 0.01:
    sys.exit("Inferred step duty is near zero.  Set a duty in manual mode "
             "before the test, or specify --duty.")

  # ── FOPDT fit ─────────────────────────────────────────────────────────────
  _ref_T0    = T0
  _ref_ustep = u_step

  # Initial guesses: process gain from final rise, time constant from half
  # the window, dead time from 5 % of the window.
  dT_obs    = T_meas[-1] - T0
  K_guess   = dT_obs / u_step if dT_obs > 0.5 else 100.0
  tau_guess = t_rel[-1] * 0.5
  L_guess   = t_rel[-1] * 0.05

  bounds_lo = [0.1,           dt,              0.0]
  bounds_hi = [1e6,           t_rel[-1] * 5.0, t_rel[-1] * 0.5]

  fit_ok = False
  try:
    popt, pcov = curve_fit(
      _fopdt_fit, t_rel, T_meas,
      p0=[K_guess, tau_guess, L_guess],
      bounds=(bounds_lo, bounds_hi),
      maxfev=20000,
    )
    K, tau, L = popt
    perr = np.sqrt(np.diag(pcov))
    T_fit   = _fopdt_fit(t_rel, K, tau, L)
    rmse    = float(np.sqrt(np.mean((T_meas - T_fit) ** 2)))
    fit_ok  = True
  except RuntimeError as exc:
    print(f"\nWARNING: FOPDT curve fit failed: {exc}")
    print("Try narrowing the window to just the step, or check --duty.")
    K, tau, L = K_guess, tau_guess, L_guess
    T_fit = _fopdt_fit(t_rel, K, tau, L)
    rmse  = float(np.sqrt(np.mean((T_meas - T_fit) ** 2)))

  print()
  print("── FOPDT model ────────────────────────────────────────────────")
  print(f"  K   (process gain)  = {K:.2f}  °C per unit duty")
  print(f"  τ   (time constant) = {tau:.1f}  s  ({tau / 60:.1f} min)")
  print(f"  L   (dead time)     = {L:.1f}  s")
  print(f"  L/τ (relative DT)   = {L / tau:.4f}"
        f"  ({'easy' if L/tau < 0.3 else 'moderate' if L/tau < 1.0 else 'difficult'}"
        f" to control)")
  print(f"  Fit RMSE            = {rmse:.3f}  °C"
        + ('' if fit_ok else '  (fit did not converge)'))
  if tau > 0.8 * t_rel[-1]:
    print()
    print("  NOTE: τ approaches the window length — consider a longer test")
    print("  to get a better time-constant estimate.")

  # ── Tuning rules ──────────────────────────────────────────────────────────
  rules: dict = {}
  if args.rule in ('imc', 'all'):
    rules['IMC-PI'] = tune_imc(K, tau, L, lam=args.lam)
  if args.rule in ('zn', 'all'):
    rules['ZN-PID'] = tune_zn(K, tau, L)
  if args.rule in ('cc', 'all'):
    rules['CC-PID'] = tune_cc(K, tau, L)

  print()
  print("── Continuous-time gains ──────────────────────────────────────")
  for name, (Kp, Ki, Kd, info) in rules.items():
    note = ('  (λ=' + f"{info['lambda_s']:.1f}" + ' s)') if 'lambda_s' in info else ''
    print(f"  {name:10s}  Kp={Kp:.5f}  Ki={Ki:.7f}  Kd={Kd:.4f}{note}")

  print()
  print(f"── bibby.ini gains  (dt = {dt * 1000:.1f} ms, discrete, gains folded) ──")
  print(f"  kp_ini = Kp_c            (same)")
  print(f"  ki_ini = Ki_c × dt       (scales down by sample period)")
  print(f"  kd_ini = Kd_c / dt       (scales up by sample period)")
  print()

  INTEGRAL_MAX = 1000.0  # from pid.cpp — verify this matches your build

  for name, (Kp, Ki, Kd, info) in rules.items():
    ini_kp = Kp
    ini_ki = Ki * dt
    ini_kd = Kd / dt

    # Effective integrator authority at the anti-windup cap.
    i_authority = ini_ki * INTEGRAL_MAX
    note = ('  (λ=' + f"{info['lambda_s']:.1f}" + ' s)') if 'lambda_s' in info else ''

    print(f"  ── {name}{note}")
    print(f"  [pid]")
    print(f"  kp = {ini_kp:.6f}")
    print(f"  ki = {ini_ki:.8f}")
    print(f"  kd = {ini_kd:.5f}")
    print()
    if i_authority < 0.10:
      print(f"  NOTE: integral authority = ki × INTEGRAL_MAX = {i_authority:.4f}")
      print(f"  The integrator alone can contribute at most {i_authority * 100:.1f}% of full duty.")
      print(f"  With the [feedforward] block below this is fine — the integrator")
      print(f"  only trims feedforward error. WITHOUT feedforward it must supply all")
      needed_max = 0.5 / ini_ki if ini_ki > 0 else float('inf')
      print(f"  holding power; for 50 % authority set INTEGRAL_MAX = {needed_max:.0f} in pid.cpp.")
      print()

  # ── Feedforward block (rule-independent) ───────────────────────────────────
  # Holding feedforward needs the process gain K (printed above) and an ambient
  # reference. T0 (the window's starting temperature) is a good ambient estimate
  # when the test began from a cold soak; otherwise set ambient_c to room temp.
  print("── bibby.ini feedforward  (holding-duty feedforward, README §9.6) ──")
  print(f"  [feedforward]")
  print(f"  process_gain_c = {K:.2f}      # identified K, °C per unit duty")
  print(f"  ambient_c      = {T0:.2f}      # window start temp; use cold-soak/room temp")
  print()
  print(f"  Holding duty at e.g. 67 °C: (67 − {T0:.1f}) / {K:.1f} = "
        f"{max(0.0, min(1.0, (67.0 - T0) / K)):.3f}")
  print()

  # ── Plot ──────────────────────────────────────────────────────────────────
  fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
  fig.suptitle(f"bibby plant identification — {args.csv}", fontsize=10)

  # Temperature trace + FOPDT fit
  ax1 = axes[0]
  ax1.plot(t_rel, T_meas, color='steelblue', lw=1.5, label='temp_filt_c (measured)')
  if fit_ok:
    label_fit = (f'FOPDT fit  K={K:.0f} °C/duty, '
                 f'τ={tau:.0f} s ({tau/60:.1f} min), '
                 f'L={L:.0f} s')
    ax1.plot(t_rel, T_fit, '--', color='tomato', lw=1.5, label=label_fit)
    # Mark dead time
    ax1.axvline(L, color='gray', ls=':', lw=1.0)
    ax1.annotate(f'L={L:.0f} s', xy=(L, T0), xytext=(L + 2, T0 + 0.5),
                 fontsize=7, color='gray')
    # Mark 63.2 % of step
    T_63 = T0 + K * u_step * 0.632
    if T_63 < T_meas.max():
      ax1.axhline(T_63, color='sandybrown', ls=':', lw=0.8)
      ax1.annotate(f'63.2 %  T={T_63:.1f}°C', xy=(t_rel[-1], T_63),
                   xytext=(t_rel[-1] * 0.6, T_63 + 0.3), fontsize=7, color='sandybrown')
  ax1.set_ylabel('Temperature (°C)')
  ax1.legend(fontsize=7)
  ax1.grid(True, alpha=0.3)

  # Duty
  ax2 = axes[1]
  ax2.plot(t_rel, dfw['duty1'].values, color='darkorange', lw=1.5, label='duty1')
  ax2.plot(t_rel, dfw['duty2'].values, '--', color='peru', lw=1.2, label='duty2')
  ax2.set_ylabel('Duty [0–1]')
  ax2.set_ylim(-0.05, 1.05)
  ax2.legend(fontsize=7)
  ax2.grid(True, alpha=0.3)
  ax2.axhline(u_step, color='gray', ls=':', lw=0.8)

  # Residuals
  ax3 = axes[2]
  residuals = T_meas - T_fit
  ax3.plot(t_rel, residuals, color='purple', lw=1.0, label=f'residuals (RMSE={rmse:.3f} °C)')
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
