#!/usr/bin/env python3
"""Draws Fig. 2 (fault rate over the course of a run) from results/fault_rate_curve.csv.
Needs matplotlib:  pip install matplotlib
"""
import csv, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

src = sys.argv[1] if len(sys.argv) > 1 else "results/fault_rate_curve.csv"
out = sys.argv[2] if len(sys.argv) > 2 else "results/fig2_fault_rate.png"
rows = list(csv.DictReader(open(src)))
series = {}
for r in rows:
    mid = (int(r["bin_start_event"]) + int(r["bin_end_event"])) / 2
    series.setdefault(r["policy"], []).append((mid, float(r["fault_pct_mean"]), float(r["ci95_halfwidth"])))

plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Liberation Serif", "DejaVu Serif"],
                     "font.size": 8, "axes.linewidth": 0.6})
fig, ax = plt.subplots(figsize=(3.45, 2.15), dpi=600)
styles = {"heal_no_learn": dict(color="#666666", ls="--", marker="s", label="Heal, no learning"),
          "proposed": dict(color="#000000", ls="-", marker="o", label="Proposed (rules installed)")}
for name, st in styles.items():
    x, y, hw = zip(*series[name])
    ax.fill_between(x, [a - b for a, b in zip(y, hw)], [a + b for a, b in zip(y, hw)], color=st["color"], alpha=0.15, lw=0)
    ax.plot(x, y, color=st["color"], ls=st["ls"], marker=st["marker"], ms=2.4, lw=1.0)
ax.axhline(4, color="#888888", lw=0.6, ls=":")
ax.text(497, 5.2, "4% execution-anomaly floor", ha="right", va="bottom", fontsize=7, color="#444444")
ax.text(497, 41.2, "Heal, no learning", ha="right", va="bottom", fontsize=7.5, color="#444444")
ax.text(110, 12.5, "Proposed (rules installed)", ha="left", va="bottom", fontsize=7.5, color="#000000")
ax.set_xlim(0, 500); ax.set_ylim(0, 48)
ax.set_xlabel("Position in run (event number)"); ax.set_ylabel("Fault rate (%)")
ax.set_xticks(range(0, 501, 100)); ax.set_yticks(range(0, 49, 10))
ax.grid(axis="y", color="#dddddd", lw=0.4); ax.set_axisbelow(True)
for s in ("top", "right"):
    ax.spines[s].set_visible(False)
fig.tight_layout(pad=0.4)
fig.savefig(out)
print("wrote", out)
