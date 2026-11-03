"""Shared figure style: recessive grid and axes, neutral text, a fixed colour order.

Colours, in this order: blue, orange, aqua (checked for colour-blind separation). The references
(setpoints, design curves) are drawn in grey and dashed, never with a series colour.
"""

import matplotlib as mpl

BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e1", "#fcfcfb"
REF = dict(color=MUTED, ls="--", lw=1.2)


def apply() -> None:
    mpl.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "axes.titlecolor": INK,
        "axes.titlesize": 10, "axes.labelsize": 9, "axes.titleweight": "regular",
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
        "xtick.color": INK2, "ytick.color": INK2, "xtick.labelsize": 8, "ytick.labelsize": 8,
        "legend.frameon": False, "legend.fontsize": 8, "lines.linewidth": 1.5,
        "font.size": 9, "text.color": INK,
    })
