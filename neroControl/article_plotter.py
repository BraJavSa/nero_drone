#!/usr/bin/env python3
# UAV flight telemetry plotter for publication-quality figures
import os
import sys
import glob
import argparse
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path

def apply_publication_style():
    mpl.rcParams.update({
        "font.family":        "serif",
        "font.serif":         ["Times New Roman", "Times", "DejaVu Serif"],
        "mathtext.fontset":   "stix",
        "font.size":          9,
        "axes.titlesize":     9,
        "axes.labelsize":     8,
        "xtick.labelsize":    7.5,
        "ytick.labelsize":    7.5,
        "legend.fontsize":    5.5,
        "figure.titlesize":   10,
        "lines.linewidth":    1.2,
        "lines.markersize":   3.5,
        "axes.spines.top":    False,
        "axes.spines.right":  False,
        "axes.linewidth":     0.7,
        "axes.grid":          True,
        "grid.linestyle":     "--",
        "grid.linewidth":     0.4,
        "grid.alpha":         0.55,
        "grid.color":         "#b0b0b0",
        "xtick.direction":    "out",
        "ytick.direction":    "out",
        "xtick.major.width":  0.7,
        "ytick.major.width":  0.7,
        "xtick.major.size":   3.0,
        "ytick.major.size":   3.0,
        "xtick.minor.visible": True,
        "ytick.minor.visible": True,
        "xtick.minor.size":   1.5,
        "ytick.minor.size":   1.5,
        "legend.frameon":        True,
        "legend.framealpha":     0.85,
        "legend.edgecolor":      "#cccccc",
        "legend.borderpad":      0.4,
        "legend.handlelength":   1.8,
        "legend.columnspacing":  1.0,
        "figure.dpi":         150,
        "savefig.dpi":        300,
        "savefig.bbox":       "tight",
        "savefig.pad_inches": 0.02,
        "pdf.fonttype":       42,
        "ps.fonttype":        42,
    })

PALETTE = {
    "measured":   "#0072B2",
    "reference":  "#D55E00",
    "command":    "#009E73",
}

LS = {
    "measured":  "-",
    "reference": "--",
    "command":   ":",
}

LW = {
    "measured":  1.3,
    "reference": 1.1,
    "command":   1.0,
}

PRESET_WIDTHS = {
    "ieee_double":  7.16,
    "ieee_single":  3.5,
    "elsevier":     6.93,
    "screen":       14.0,
}
DEFAULT_HEIGHT = 9.0
DATA_DIR = os.path.expanduser("~/ros2_ws/src/neroControl/data")

def load_latest_csv(directory=DATA_DIR):
    files = glob.glob(os.path.join(directory, "*.csv"))
    if not files: return None
    return max(files, key=os.path.getmtime)

def pick_file_gui():
    try:
        from tkinter import Tk
        from tkinter.filedialog import askopenfilename
        root = Tk(); root.withdraw()
        path = askopenfilename(title="Select Bebop CSV", initialdir=DATA_DIR, filetypes=[("CSV files", "*.csv")])
        root.destroy()
        return path or None
    except Exception as e:
        print(f"GUI picker unavailable: {e}")
        return None

def preprocess(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df["time"]  = pd.to_numeric(df["time"], errors="coerce")
    df["t_rel"] = df["time"] - df["time"].iloc[0]
    if "yaw" in df.columns and "yawd" in df.columns:
        diff = df["yaw"] - df["yawd"]
        df["yaw_corr"] = df["yawd"] + np.arctan2(np.sin(diff), np.cos(diff))
    elif "yaw" in df.columns:
        df["yaw_corr"] = df["yaw"]
    else:
        df["yaw_corr"] = 0.0
    if "yaw_rate" in df.columns:
        try:
            from scipy.signal import savgol_filter
            df["yaw_rate"] = savgol_filter(df["yaw_rate"].fillna(method="ffill").values, window_length=15, polyorder=2)
        except ImportError:
            df["yaw_rate"] = (df["yaw_rate"].rolling(window=11, center=True, min_periods=1).mean())
    return df

def safe_plot(ax, t, df, col, role, **kwargs):
    if col not in df.columns: return None
    line, = ax.plot(t, df[col], color=PALETTE[role], ls=LS[role], lw=LW[role], **kwargs)
    return line

def _add_legend(ax, handles: list, labels: list):
    if not handles: return
    ax.legend(handles, labels, loc="best", fontsize=5.5, frameon=True, framealpha=0.85, edgecolor="#cccccc", borderpad=0.35, handlelength=1.6, labelspacing=0.25)

def _vel_ylim(df: pd.DataFrame, cols: list, margin: float = 0.05):
    vals = [df[c].dropna().values for c in cols if c in df.columns]
    if not vals: return (-1.0, 1.0)
    all_v  = np.concatenate(vals)
    v_min, v_max = float(all_v.min()), float(all_v.max())
    pad = margin * max(abs(v_min), abs(v_max), 1e-9)
    return (v_min - pad, v_max + pad)

def build_figure(df: pd.DataFrame, preset: str = "ieee_double", fig_height: float = DEFAULT_HEIGHT):
    apply_publication_style()
    fig_width = PRESET_WIDTHS.get(preset, PRESET_WIDTHS["ieee_double"])
    fig = plt.figure(figsize=(fig_width, fig_height), constrained_layout=False)
    gs = gridspec.GridSpec(4, 2, figure=fig, hspace=0.30, wspace=0.48)
    axes = np.array([[fig.add_subplot(gs[r, c]) for c in range(2)] for r in range(4)])
    t, t_s, t_e = df["t_rel"], df["t_rel"].min(), df["t_rel"].max()
    fmt1d = mpl.ticker.FormatStrFormatter('%.1f')
    pos_rows = [
        ("x", "xd", r"Position $\mathregular{x}$ [m]"),
        ("y", "yd", r"Position $\mathregular{y}$ [m]"),
        ("z", "zd", r"Position $\mathregular{z}$ [m]"),
        ("yaw_corr", "yawd", r"Yaw $\psi$ [rad]"),
    ]
    for i, (meas, ref, ylabel) in enumerate(pos_rows):
        ax = axes[i, 0]
        h1 = safe_plot(ax, t, df, meas, "measured")
        h2 = safe_plot(ax, t, df, ref,  "reference")
        ax.set_ylabel(ylabel, fontsize=8.5)
        ax.set_xlim(t_s, t_e)
        ax.yaxis.set_major_formatter(fmt1d)
        handles = [(h, lbl) for h, lbl in [(h1,"Measured"),(h2,"Reference")] if h]
        _add_legend(ax, [h for h,_ in handles], [l for _,l in handles])
        if i == 3: ax.set_xlabel(r"$t$ [s]")
    vel_rows = [
        ("cmd_linx", "linx_b", "vxd_b", r"Velocity $v_x$ [m/s]"),
        ("cmd_liny", "liny_b", "vyd_b", r"Velocity $v_y$ [m/s]"),
        ("cmd_linz", "linz_b", "vzd_b", r"Velocity $v_z$ [m/s]"),
        ("cmd_angz", "yaw_rate","wyawd", r"Yaw rate $\dot{\psi}$ [rad/s]"),
    ]
    for i, (cmd, meas, ref, ylabel) in enumerate(vel_rows):
        ax = axes[i, 1]
        hc = safe_plot(ax, t, df, cmd,  "command")
        hm = safe_plot(ax, t, df, meas, "measured")
        hr = safe_plot(ax, t, df, ref,  "reference")
        ax.set_ylabel(ylabel, fontsize=8.5)
        ax.set_xlim(t_s, t_e)
        ax.yaxis.set_major_formatter(fmt1d)
        ax.set_ylim(_vel_ylim(df, [cmd, meas, ref]))
        handles = [(h, lbl) for h, lbl in [(hc,"Command"),(hm,"Measured"),(hr,"Reference")] if h]
        _add_legend(ax, [h for h,_ in handles], [l for _,l in handles])
        if i == 3: ax.set_xlabel(r"$t$ [s]")
    fig.align_ylabels(axes[:, 0]); fig.align_ylabels(axes[:, 1])
    plt.tight_layout(rect=[0, 0.01, 1, 1])
    return fig

def export(fig, stem: str, out_dir: str = "."):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / f"{stem}.pdf", format="pdf")
    fig.savefig(out / f"{stem}.png", format="png", dpi=300)
    print(f"Saved to {out}")

def main():
    parser = argparse.ArgumentParser(description="Publication-quality UAV telemetry plotter.")
    parser.add_argument("csv", nargs="?", help="Path to CSV file")
    parser.add_argument("--pick", action="store_true", help="GUI picker")
    parser.add_argument("--preset", default="ieee_double", choices=list(PRESET_WIDTHS.keys()), help="Width preset")
    parser.add_argument("--height", type=float, default=DEFAULT_HEIGHT, help="Height")
    parser.add_argument("--export", metavar="DIR", default=None, help="Export directory")
    parser.add_argument("--no-show", action="store_true", help="Skip display")
    args = parser.parse_args()
    csv_path = args.csv if args.csv else pick_file_gui() if args.pick else load_latest_csv()
    if not csv_path or not os.path.isfile(csv_path):
        print("No CSV found."); sys.exit(1)
    df = preprocess(pd.read_csv(csv_path))
    fig = build_figure(df, preset=args.preset, fig_height=args.height)
    if args.export: export(fig, stem=Path(csv_path).stem, out_dir=args.export)
    if not args.no_show: plt.show()

if __name__ == "__main__":
    main()