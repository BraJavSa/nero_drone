#!/usr/bin/env python3

import os
import sys
import glob
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from tkinter import Tk
from tkinter.filedialog import askopenfilename
from pathlib import Path

DATA_DIR = os.path.expanduser("~/ros2_ws/src/neroControl/data")

PALETTE = {
    "measured":  "#0072B2",
    "reference": "#D55E00",
    "command":   "#009E73",
}
LS = {"measured": "-", "reference": "--", "command": ":"}
LW = {"measured": 1.3, "reference": 1.1, "command": 1.0}


def apply_publication_style():
    mpl.rcParams.update({
        "font.family":           "serif",
        "font.serif":            ["Times New Roman", "Times", "DejaVu Serif"],
        "mathtext.fontset":      "stix",
        "font.size":             9,
        "axes.titlesize":        9,
        "axes.labelsize":        10,
        "xtick.labelsize":       7.5,
        "ytick.labelsize":       7.5,
        "legend.fontsize":       5.5,
        "figure.titlesize":      10,
        "lines.linewidth":       1.2,
        "lines.markersize":      3.5,
        "axes.spines.top":       False,
        "axes.spines.right":     False,
        "axes.linewidth":        0.7,
        "axes.grid":             True,
        "grid.linestyle":        "--",
        "grid.linewidth":        0.4,
        "grid.alpha":            0.55,
        "grid.color":            "#b0b0b0",
        "xtick.direction":       "out",
        "ytick.direction":       "out",
        "xtick.major.width":     0.7,
        "ytick.major.width":     0.7,
        "xtick.major.size":      3.0,
        "ytick.major.size":      3.0,
        "xtick.minor.visible":   True,
        "ytick.minor.visible":   True,
        "xtick.minor.size":      1.5,
        "ytick.minor.size":      1.5,
        "legend.frameon":        True,
        "legend.framealpha":     0.80,
        "legend.edgecolor":      "#cccccc",
        "legend.borderpad":      0.3,
        "legend.handlelength":   1.4,
        "legend.columnspacing":  0.8,
        "legend.labelspacing":   0.18,
        "figure.dpi":            150,
        "savefig.dpi":           300,
        "savefig.bbox":          "tight",
        "savefig.pad_inches":    0.02,
        "pdf.fonttype":          42,
        "ps.fonttype":           42,
    })


def safe_plot(ax, t, df, col, role, **kwargs):
    if col not in df.columns:
        return None
    line, = ax.plot(t, df[col], color=PALETTE[role], ls=LS[role], lw=LW[role], **kwargs)
    return line


def add_legend(ax, handles, labels):
    if not handles:
        return
    ax.legend(handles, labels, loc="best", fontsize=5.5, frameon=True,
              framealpha=0.80, edgecolor="#cccccc", borderpad=0.3,
              handlelength=1.4, labelspacing=0.18)


def preprocess(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df["time"]  = pd.to_numeric(df["time"], errors="coerce")
    df["t_rel"] = df["time"] - df["time"].iloc[0]
    if "yaw" in df.columns:
        df["yaw_plot"] = np.unwrap(df["yaw"].values)
    if "yawd" in df.columns:
        df["yawd_plot"] = np.unwrap(df["yawd"].values)
    if "yaw_rate" in df.columns:
        try:
            from scipy.signal import savgol_filter
            df["yaw_rate"] = savgol_filter(
                df["yaw_rate"].ffill().values,
                window_length=15, polyorder=2)
        except ImportError:
            df["yaw_rate"] = df["yaw_rate"].rolling(
                window=11, center=True, min_periods=1).mean()
    return df


class BebopVisualizer:
    def __init__(self):
        os.makedirs(DATA_DIR, exist_ok=True)
        self.current_file = self.get_latest_file()
        if self.current_file:
            self.run_visualization()
        else:
            self.manual_select()

    def get_latest_file(self):
        files = glob.glob(os.path.join(DATA_DIR, "*.csv"))
        return max(files, key=os.path.getmtime) if files else None

    def refresh_latest(self, event=None):
        newest = self.get_latest_file()
        if newest:
            plt.close("all")
            self.current_file = newest
            self.run_visualization()

    def manual_select(self, event=None):
        root = Tk()
        root.withdraw()
        selected = askopenfilename(
            title="Select Bebop CSV File",
            initialdir=DATA_DIR,
            filetypes=[("CSV files", "*.csv")]
        )
        root.destroy()
        if selected:
            plt.close("all")
            self.current_file = selected
            self.run_visualization()

    def close_application(self, event=None):
        plt.close("all")
        sys.exit()

    def run_visualization(self):
        try:
            df = preprocess(pd.read_csv(self.current_file))
            self.render(df)
        except Exception as e:
            print(f"Error: {e}")

    def render(self, df):
        apply_publication_style()

        t, t_s, t_e = df["t_rel"], df["t_rel"].min(), df["t_rel"].max()
        fmt1d = mpl.ticker.FormatStrFormatter("%.1f")

        fig = plt.figure(figsize=(15, 9))

        # Tight outer margins: left/right for ylabel space, top for title+buttons
        fig.subplots_adjust(
            left=0.07, right=0.99,
            top=0.93,  bottom=0.07,
            hspace=0.18, wspace=0.28
        )

        axes = [[fig.add_subplot(4, 2, r * 2 + c + 1) for c in range(2)] for r in range(4)]

        try:
            plt.get_current_fig_manager().window.showMaximized()
        except Exception:
            pass

        fig.suptitle(
            f"UAV Flight Telemetry — {Path(self.current_file).name}",
            fontsize=10, fontweight="bold", y=0.975)

        ax_refresh = fig.add_axes([0.74, 0.942, 0.07, 0.025])
        self.btn_refresh = plt.Button(
            ax_refresh, "Update", color="#e1f5fe", hovercolor="#b3e5fc")
        self.btn_refresh.label.set_fontsize(8)
        self.btn_refresh.on_clicked(self.refresh_latest)

        ax_open = fig.add_axes([0.82, 0.942, 0.07, 0.025])
        self.btn_open = plt.Button(
            ax_open, "Open", color="#f0f0f0", hovercolor="#d0d0d0")
        self.btn_open.label.set_fontsize(8)
        self.btn_open.on_clicked(self.manual_select)

        ax_exit = fig.add_axes([0.90, 0.942, 0.07, 0.025])
        self.btn_exit = plt.Button(
            ax_exit, "Exit", color="#ffcccc", hovercolor="#ffaaaa")
        self.btn_exit.label.set_fontsize(8)
        self.btn_exit.on_clicked(self.close_application)

        pos_rows = [
            ("x",        "xd",        r"Position $x$ [m]"),
            ("y",        "yd",        r"Position $y$ [m]"),
            ("z",        "zd",        r"Position $z$ [m]"),
            ("yaw_plot", "yawd_plot", r"Yaw $\psi$ [rad]"),
        ]
        for i, (meas, ref, ylabel) in enumerate(pos_rows):
            ax = axes[i][0]
            h1 = safe_plot(ax, t, df, meas, "measured")
            h2 = safe_plot(ax, t, df, ref,  "reference")
            ax.set_ylabel(ylabel)
            ax.set_xlim(t_s, t_e)
            ax.yaxis.set_major_formatter(fmt1d)
            # 10% margin above max and below min across measured + reference
            cols = [c for c in [meas, ref] if c in df.columns]
            if cols:
                vals = np.concatenate([df[c].dropna().values for c in cols])
                vmin, vmax = vals.min(), vals.max()
                pad = 0.10 * max(abs(vmax - vmin), 1e-6)
                ax.set_ylim(vmin - pad, vmax + pad)
            handles = [(h, l) for h, l in [(h1, "Measured"), (h2, "Reference")] if h]
            add_legend(ax, [h for h, _ in handles], [l for _, l in handles])
            if i < 3:
                ax.tick_params(labelbottom=False)
            else:
                ax.set_xlabel(r"$t$ [s]")

        vel_rows = [
            ("cmd_linx", "linx_b",   "vxd_b", r"Velocity $v_x$ [m/s]",          (-1.1,  1.1)),
            ("cmd_liny", "liny_b",   "vyd_b", r"Velocity $v_y$ [m/s]",          (-1.1,  1.1)),
            ("cmd_linz", "linz_b",   "vzd_b", r"Velocity $v_z$ [m/s]",          (-1.1,  1.1)),
            ("cmd_angz", "yaw_rate", "wyawd", r"Yaw rate $\dot{\psi}$ [rad/s]", (-1.76, 1.76)),
        ]
        for i, (cmd, meas, ref, ylabel, ylim) in enumerate(vel_rows):
            ax = axes[i][1]
            hc = safe_plot(ax, t, df, cmd,  "command")
            hm = safe_plot(ax, t, df, meas, "measured")
            hr = safe_plot(ax, t, df, ref,  "reference")
            ax.set_ylabel(ylabel)
            ax.set_xlim(t_s, t_e)
            ax.set_ylim(ylim)
            ax.yaxis.set_major_formatter(fmt1d)
            handles = [(h, l) for h, l in
                       [(hc, "Command"), (hm, "Measured"), (hr, "Reference")] if h]
            add_legend(ax, [h for h, _ in handles], [l for _, l in handles])
            if i < 3:
                ax.tick_params(labelbottom=False)
            else:
                ax.set_xlabel(r"$t$ [s]")

        axes_np = np.array(axes)
        fig.align_ylabels(axes_np[:, 0])
        fig.align_ylabels(axes_np[:, 1])

        plt.show()


if __name__ == "__main__":
    BebopVisualizer()