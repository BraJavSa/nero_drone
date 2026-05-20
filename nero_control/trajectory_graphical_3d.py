#!/usr/bin/env python3

# Renders a 3D visualization of the drone flight path compared to the reference.
import os
import sys
import glob
import argparse
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from pathlib import Path

def apply_publication_style():
    mpl.rcParams.update({
        "font.family":          "serif",
        "font.serif":           ["Times New Roman", "Times", "DejaVu Serif"],
        "mathtext.fontset":     "stix",
        "font.size":            8,
        "axes.labelsize":       8.5,
        "xtick.labelsize":      7.5,
        "ytick.labelsize":      7.5,
        "legend.fontsize":      7,
        "axes.linewidth":       0.7,
        "lines.linewidth":      1.2,
        "figure.dpi":           150,
        "savefig.dpi":          300,
        "savefig.bbox":         "tight",
        "savefig.pad_inches":   0.04,
        "pdf.fonttype":         42,
        "ps.fonttype":          42,
    })

DATA_DIR = os.path.expanduser("~/ros2_ws/src/neroControl/data")

def load_latest_csv(directory=DATA_DIR):
    files = glob.glob(os.path.join(directory, "*.csv"))
    return max(files, key=os.path.getmtime) if files else None

def pick_file_gui():
    try:
        from tkinter import Tk
        from tkinter.filedialog import askopenfilename
        root = Tk(); root.withdraw()
        path = askopenfilename(title="Select Bebop CSV", initialdir=DATA_DIR, filetypes=[("CSV files", "*.csv")])
        root.destroy()
        return path or None
    except Exception as e:
        print(f"[GUI unavailable] {e}")
        return None

def preprocess(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df["time"]  = pd.to_numeric(df["time"], errors="coerce")
    df["t_rel"] = df["time"] - df["time"].iloc[0]
    if "yaw" in df.columns and "yawd" in df.columns:
        d = df["yaw"] - df["yawd"]
        df["yaw_corr"] = df["yawd"] + np.arctan2(np.sin(d), np.cos(d))
    elif "yaw" in df.columns:
        df["yaw_corr"] = df["yaw"]
    else:
        df["yaw_corr"] = 0.0
    return df

def _get_xyz(df, cols):
    arrays = []
    for c in cols:
        arrays.append(df[c].values if c in df.columns else np.zeros(len(df)))
    return np.column_stack(arrays)

def _position_error(meas: np.ndarray, ref: np.ndarray) -> np.ndarray:
    return np.linalg.norm(meas - ref, axis=1)

def _colored_line_3d(ax, x, y, z, scalar, cmap, norm, lw=1.5, zorder=5):
    pts  = np.array([x, y, z]).T.reshape(-1, 1, 3)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc   = Line3DCollection(segs, cmap=cmap, norm=norm, linewidth=lw, zorder=zorder, capstyle="round", joinstyle="round")
    lc.set_array(scalar)
    ax.add_collection3d(lc)
    return lc

def _floor_shadow(ax, x, y, z_floor, alpha=0.18, color="#555555", lw=0.8):
    ax.plot(x, y, z_floor, color=color, lw=lw, alpha=alpha, zorder=1, linestyle="-")

def build_trajectory_figure(df: pd.DataFrame, cmap_name: str = "plasma", elev: float = 22, azim: float = -50, fig_width: float = 3.5, fig_height: float = 4.0) -> plt.Figure:
    apply_publication_style()
    meas = _get_xyz(df, ["x", "y", "z"])
    ref = _get_xyz(df, ["xd", "yd", "zd"])
    x_m, y_m, z_m = meas[:, 0], meas[:, 1], meas[:, 2]
    x_r, y_r, z_r = ref[:, 0], ref[:, 1], ref[:, 2]
    err = _position_error(meas, ref)
    rmse = float(np.sqrt(np.mean(err ** 2)))
    cmap = cm.get_cmap(cmap_name)
    norm = mcolors.Normalize(vmin=err.min(), vmax=err.max())
    fig = plt.figure(figsize=(fig_width, fig_height))
    ax = fig.add_subplot(111, projection="3d")
    ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor("#cccccc")
    ax.yaxis.pane.set_edgecolor("#cccccc")
    ax.zaxis.pane.set_edgecolor("#cccccc")
    ax.xaxis._axinfo["grid"].update({"linewidth": 0.4, "color": "#d8d8d8", "linestyle": "--"})
    ax.yaxis._axinfo["grid"].update({"linewidth": 0.4, "color": "#d8d8d8", "linestyle": "--"})
    ax.zaxis._axinfo["grid"].update({"linewidth": 0.4, "color": "#d8d8d8", "linestyle": "--"})
    z_floor = z_m.min() - 0.02 * (z_m.max() - z_m.min() + 1e-6)
    _floor_shadow(ax, x_m, y_m, z_floor)
    ax.plot(x_r, y_r, z_r, color="#888888", lw=0.9, ls="--", alpha=0.75, label="Reference", zorder=3)
    lc = _colored_line_3d(ax, x_m, y_m, z_m, scalar=err, cmap=cmap, norm=norm, lw=1.6, zorder=6)
    marker_kw = dict(s=28, zorder=10, edgecolors="white", linewidths=0.5)
    ax.scatter(*meas[0], c=[[cmap(norm(err[0]))]], marker="o", **marker_kw)
    ax.scatter(*meas[-1], c=[[cmap(norm(err[-1]))]], marker="s", **marker_kw)
    txt_kw = dict(fontsize=6.5, color="#333333", fontfamily="serif", zorder=11)
    ax.text(x_m[0], y_m[0], z_m[0] + 0.03, "start", **txt_kw)
    ax.text(x_m[-1], y_m[-1], z_m[-1] + 0.03, "end", **txt_kw)
    ax.set_xlabel(r"$x$ [m]", fontsize=8, labelpad=2)
    ax.set_ylabel(r"$y$ [m]", fontsize=8, labelpad=2)
    ax.set_zlabel(r"$z$ [m]", fontsize=8, labelpad=2)
    ax.tick_params(labelsize=7, pad=0.5)
    ax.view_init(elev=elev, azim=azim)
    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, shrink=0.55, aspect=18, pad=0.08, orientation="vertical")
    cbar.set_label(r"Position error $\|e\|_2$ [m]", fontsize=7.5, labelpad=4)
    cbar.ax.tick_params(labelsize=7)
    cbar.outline.set_linewidth(0.5)
    cbar.ax.yaxis.set_major_formatter(mpl.ticker.FormatStrFormatter("%.2f"))
    ref_proxy = plt.Line2D([0], [0], color="#888888", lw=0.9, ls="--", label="Reference")
    meas_proxy = plt.Line2D([0], [0], color=cmap(0.7), lw=1.6, label="Measured")
    ax.legend(handles=[meas_proxy, ref_proxy], loc="upper left", fontsize=7, frameon=True, framealpha=0.85, edgecolor="#cccccc", borderpad=0.4, handlelength=1.5)
    ax.text2D(0.98, 0.02, f"RMSE $= {rmse:.3f}$ m", transform=ax.transAxes, fontsize=7, ha="right", va="bottom", bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="#cccccc", alpha=0.85, linewidth=0.5))
    plt.tight_layout()
    return fig

def export(fig, stem: str, out_dir: str = "."):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / f"{stem}_3d.pdf", format="pdf")
    fig.savefig(out / f"{stem}_3d.png", format="png", dpi=300)
    print(f"Exported to {out}")

def main():
    parser = argparse.ArgumentParser(description="3D UAV trajectory colored by error.")
    parser.add_argument("csv", nargs="?", help="Path to CSV file")
    parser.add_argument("--pick", action="store_true", help="GUI file picker")
    parser.add_argument("--cmap", default="plasma", help="Colormap")
    parser.add_argument("--elev", type=float, default=22, help="Elevation")
    parser.add_argument("--azim", type=float, default=-50, help="Azimuth")
    parser.add_argument("--width", type=float, default=3.5, help="Width")
    parser.add_argument("--height", type=float, default=3.8, help="Height")
    parser.add_argument("--export", metavar="DIR", default=None, help="Export directory")
    parser.add_argument("--no-show", action="store_true", help="Skip display")
    args = parser.parse_args()
    csv_path = args.csv if args.csv else pick_file_gui() if args.pick else load_latest_csv()
    if not csv_path or not os.path.isfile(csv_path):
        print("No CSV found.")
        sys.exit(1)
    df = preprocess(pd.read_csv(csv_path))
    fig = build_trajectory_figure(df, cmap_name=args.cmap, elev=args.elev, azim=args.azim, fig_width=args.width, fig_height=args.height)
    if args.export: export(fig, stem=Path(csv_path).stem, out_dir=args.export)
    if not args.no_show: plt.show()

if __name__ == "__main__":
    main()
