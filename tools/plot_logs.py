#!/usr/bin/env python3
"""
plot_logs.py — bibby CSV log viewer
-----------------------------------
A small Tk GUI for browsing the bibby log tree and plotting any subset of the
recorded variables.  Pick a log from the folder tree on the left, choose an
X axis (wall_time or t_monotonic_s) and tick the variables to plot.  The
default selection is temp_raw_c and setpoint_c against wall_time.

Columns are read straight from each file's header, so new log fields show up
automatically without touching this script.

Requirements:
    pip install pandas matplotlib   (tkinter ships with CPython)

Usage:
    python3 plot_logs.py [LOGS_ROOT]

    LOGS_ROOT defaults to ../logs next to this script (the bibby logs tree).
"""

import argparse
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

import pandas as pd
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

# Columns that make sense as a time / X axis.
X_CANDIDATES = ("wall_time", "t_monotonic_s")
# Variables ticked on by default.
DEFAULT_Y = ("temp_raw_c", "setpoint_c")


class LogViewer(tk.Tk):
  def __init__(self, logs_root):
    super().__init__()
    self.title("bibby log viewer")
    self.geometry("1100x700")
    self.logs_root = Path(logs_root).expanduser().resolve()

    self.df = None          # currently loaded DataFrame
    self.current_path = None
    self.item_paths = {}    # tree item id -> filesystem Path
    self.y_vars = {}        # column name -> tk.BooleanVar

    self._build_ui()
    self._populate_tree()
    self._select_newest()

  # ----- UI construction -------------------------------------------------
  def _build_ui(self):
    left = ttk.Frame(self, padding=4)
    left.pack(side=tk.LEFT, fill=tk.Y)

    ttk.Label(left, text="Log files").pack(anchor="w")
    tree_frame = ttk.Frame(left)
    tree_frame.pack(fill=tk.Y)
    self.tree = ttk.Treeview(tree_frame, show="tree", height=16)
    tsb = ttk.Scrollbar(tree_frame, orient="vertical", command=self.tree.yview)
    self.tree.configure(yscrollcommand=tsb.set)
    self.tree.pack(side=tk.LEFT, fill=tk.Y)
    tsb.pack(side=tk.LEFT, fill=tk.Y)
    self.tree.bind("<<TreeviewSelect>>", self._on_tree_select)

    xrow = ttk.Frame(left)
    xrow.pack(fill=tk.X, pady=(8, 2))
    ttk.Label(xrow, text="X axis:").pack(side=tk.LEFT)
    self.x_var = tk.StringVar(value=X_CANDIDATES[0])
    x_combo = ttk.Combobox(xrow, textvariable=self.x_var, values=list(X_CANDIDATES),
                           state="readonly", width=13)
    x_combo.pack(side=tk.LEFT, padx=4)
    x_combo.bind("<<ComboboxSelected>>", lambda e: self._plot())

    ttk.Label(left, text="Variables").pack(anchor="w", pady=(8, 0))
    var_container = ttk.Frame(left)
    var_container.pack(fill=tk.BOTH, expand=True)
    self.var_inner = self._make_scrollable(var_container)

    right = ttk.Frame(self)
    right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    self.fig = Figure(figsize=(7, 5), dpi=100)
    self.ax = self.fig.add_subplot(111)
    self.canvas = FigureCanvasTkAgg(self.fig, master=right)
    toolbar = NavigationToolbar2Tk(self.canvas, right, pack_toolbar=False)
    toolbar.update()
    toolbar.pack(side=tk.BOTTOM, fill=tk.X)
    self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

  def _make_scrollable(self, parent):
    # A vertically scrollable frame for the variable checkboxes.
    canvas = tk.Canvas(parent, borderwidth=0, highlightthickness=0, width=170)
    sb = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
    inner = ttk.Frame(canvas)
    inner.bind("<Configure>",
               lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
    canvas.create_window((0, 0), window=inner, anchor="nw")
    canvas.configure(yscrollcommand=sb.set)
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    sb.pack(side=tk.LEFT, fill=tk.Y)
    return inner

  # ----- file tree -------------------------------------------------------
  def _populate_tree(self):
    if not self.logs_root.exists():
      messagebox.showerror("bibby log viewer",
                           f"Logs directory not found:\n{self.logs_root}")
      return
    self._add_dir("", self.logs_root)

  def _add_dir(self, parent_iid, path):
    for entry in sorted(path.iterdir()):
      if entry.is_dir():
        iid = self.tree.insert(parent_iid, "end", text=entry.name)
        self.item_paths[iid] = entry
        self._add_dir(iid, entry)
      elif entry.suffix == ".csv":
        iid = self.tree.insert(parent_iid, "end", text=entry.name)
        self.item_paths[iid] = entry

  def _select_newest(self):
    files = [(iid, p) for iid, p in self.item_paths.items() if p.is_file()]
    if not files:
      return
    iid, _ = max(files, key=lambda ip: ip[1].stat().st_mtime)
    self.tree.see(iid)              # expands ancestors
    self.tree.selection_set(iid)    # fires <<TreeviewSelect>> -> load

  def _on_tree_select(self, _event):
    sel = self.tree.selection()
    if not sel:
      return
    path = self.item_paths.get(sel[0])
    if path is not None and path.is_file():
      self._load_file(path)

  # ----- data + plotting -------------------------------------------------
  def _load_file(self, path):
    try:
      df = pd.read_csv(path)
    except Exception as exc:
      messagebox.showerror("bibby log viewer", f"Failed to read {path.name}:\n{exc}")
      return
    if "wall_time" in df.columns:
      df["wall_time"] = pd.to_datetime(df["wall_time"], format="ISO8601",
                                       errors="coerce")
    self.df = df
    self.current_path = path
    self._build_var_checkboxes(list(df.columns))
    self._plot()

  def _build_var_checkboxes(self, columns):
    # Time columns belong on the X axis only; everything else is a Y series.
    y_cols = [c for c in columns if c not in X_CANDIDATES]
    if set(y_cols) == set(self.y_vars):
      return  # same schema: keep current widgets and selection
    for child in self.var_inner.winfo_children():
      child.destroy()
    self.y_vars = {}
    for c in y_cols:
      var = tk.BooleanVar(value=(c in DEFAULT_Y))
      ttk.Checkbutton(self.var_inner, text=c, variable=var,
                      command=self._plot).pack(anchor="w")
      self.y_vars[c] = var

  def _plot(self):
    if self.df is None:
      return
    self.ax.clear()
    xcol = self.x_var.get()
    if xcol in self.df.columns:
      x = self.df[xcol]
      selected = [c for c, v in self.y_vars.items()
                  if v.get() and c in self.df.columns]
      for c in selected:
        self.ax.plot(x, self.df[c], label=c, linewidth=1)
      if selected:
        self.ax.legend(loc="best", fontsize=8)
      self.ax.set_xlabel(xcol)
      self.ax.grid(True, alpha=0.3)
      if self.current_path:
        self.ax.set_title(str(self.current_path.relative_to(self.logs_root)))
      self.fig.autofmt_xdate()  # rotate date ticks; harmless for numeric X
    self.fig.tight_layout()
    self.canvas.draw_idle()


def main():
  default_root = Path(__file__).resolve().parent.parent / "logs"
  ap = argparse.ArgumentParser(description="bibby CSV log viewer")
  ap.add_argument("logs_root", nargs="?", default=str(default_root),
                  help="Root of the logs tree (default: ../logs next to this script)")
  args = ap.parse_args()
  LogViewer(args.logs_root).mainloop()


if __name__ == "__main__":
  main()
