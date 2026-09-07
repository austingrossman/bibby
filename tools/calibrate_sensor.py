#!/usr/bin/env python3
"""
calibrate_sensor.py — derive bibby.ini [sensor] calibration values
------------------------------------------------------------------
Turns raw calibration measurements into the three bibby.ini [sensor] keys.
No dependencies beyond the standard library.  The full procedure, including
how to take the measurements, is README §9.1–9.2; the same formulas live as
comments in bibby.ini.

Two steps, run in order:

1. Ice-point Rref trim  (README §9.1)
   Stir the probe in a 0 °C ice/water slush, note what bibby reads with the
   CURRENT ref_resistor_ohms, then:

       python3 calibrate_sensor.py rref --ice-reading 1.4 --rref-current 400

   → prints the trimmed ref_resistor_ohms.  Put it in bibby.ini, restart,
     and confirm the ice bath now reads ≈ 0.0 °C.

2. Two-point span trim  (README §9.2)
   With the Rref trim already applied, take two reference points against a
   trusted thermometer (e.g. the ice bath again plus a fan-mixed warm bath):

       python3 calibrate_sensor.py span --p1 0.0 0.0 --p2 25.0 23.78

   (--p1/--p2 take REFERENCE then MEASURED °C.)
   → prints temp_cal_gain / temp_cal_offset for bibby.ini.

Boiling-point check: pure water boils below 100 °C at altitude —
    T_boil ≈ 100 − 0.00335 × altitude_m  (°C, first-order)
Use `span --p2 <corrected boil> <measured>` for a brew-range second point.
"""

import argparse
import sys

# Callendar–Van Dusen, T >= 0 °C:  R(T) = R0 (1 + A·T + B·T²)
CVD_A = 3.9083e-3
CVD_B = -5.775e-7
R0    = 100.0  # PT100


def pt100_resistance(temp_c: float) -> float:
  return R0 * (1.0 + CVD_A * temp_c + CVD_B * temp_c * temp_c)


def cmd_rref(args):
  r_ice = pt100_resistance(args.ice_reading)
  rref = args.rref_current * R0 / r_ice
  print(f"Ice bath read {args.ice_reading:+.3f} °C with Rref = {args.rref_current:.2f} Ω")
  print(f"R_pt100({args.ice_reading:.3f} °C) = {r_ice:.3f} Ω  (should be 100.000 Ω at 0 °C)")
  print()
  print("bibby.ini:")
  print("  [sensor]")
  print(f"  ref_resistor_ohms = {rref:.2f}")
  print()
  print("Restart bibby and verify the ice bath now reads ≈ 0.0 °C, then do the")
  print("span step (calibrate_sensor.py span ...).")


def cmd_span(args):
  (t1_ref, t1_meas), (t2_ref, t2_meas) = args.p1, args.p2
  if abs(t2_meas - t1_meas) < 1e-6:
    sys.exit("The two measured temperatures are identical — need distinct points.")
  gain = (t2_ref - t1_ref) / (t2_meas - t1_meas)
  offset = t1_ref - gain * t1_meas
  print(f"Point 1: reference {t1_ref:.3f} °C, measured {t1_meas:.3f} °C")
  print(f"Point 2: reference {t2_ref:.3f} °C, measured {t2_meas:.3f} °C")
  print()
  print("bibby.ini:")
  print("  [sensor]")
  print(f"  temp_cal_gain   = {gain:.4f}")
  print(f"  temp_cal_offset = {offset:.3f}")
  print()
  span = abs(t2_ref - t1_ref)
  if span < 40.0:
    print(f"NOTE: the calibration span is only {span:.0f} °C. Errors grow when")
    print("extrapolating; confirm against a boiling-point reference (altitude-")
    print("corrected) before trusting brew-range temperatures.")


def main():
  ap = argparse.ArgumentParser(
      description=__doc__,
      formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = ap.add_subparsers(dest='cmd', required=True)

  ap_r = sub.add_parser('rref', help='ice-point Rref trim')
  ap_r.add_argument('--ice-reading', type=float, required=True, metavar='C',
                    help='what bibby reads in the 0 °C ice bath')
  ap_r.add_argument('--rref-current', type=float, default=400.0, metavar='OHM',
                    help='ref_resistor_ohms currently in bibby.ini (default 400)')
  ap_r.set_defaults(fn=cmd_rref)

  ap_s = sub.add_parser('span', help='two-point gain/offset trim')
  ap_s.add_argument('--p1', type=float, nargs=2, required=True,
                    metavar=('REF_C', 'MEAS_C'),
                    help='first point: reference °C then measured °C')
  ap_s.add_argument('--p2', type=float, nargs=2, required=True,
                    metavar=('REF_C', 'MEAS_C'),
                    help='second point: reference °C then measured °C')
  ap_s.set_defaults(fn=cmd_span)

  args = ap.parse_args()
  args.fn(args)


if __name__ == '__main__':
  main()
