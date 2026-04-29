#!/usr/bin/env python3
# Visualizer for UAV flight telemetry data from CSV files
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import os
import glob
import sys

class BebopVisualizer:
    def __init__(self):
        self.default_dir = os.path.expanduser("~/ros2_ws/src/neroControl/data")
        os.makedirs(self.default_dir, exist_ok=True)
        self.current_file = self.get_latest_file()
        if self.current_file:
            self.run_visualization()
        else:
            print("No experimental data found in the designated directory.")
            self.manual_select()

    def get_latest_file(self):
        files = glob.glob(os.path.join(self.default_dir, "*.csv"))
        return max(files, key=os.path.getmtime) if files else None

    def refresh_latest(self, event=None):
        newest_file = self.get_latest_file()
        if newest_file:
            plt.close('all')
            self.current_file = newest_file
            self.run_visualization()

    def manual_select(self, event=None):
        root = Tk()
        root.withdraw()
        selected = askopenfilename(
            title="Select Bebop CSV File",
            initialdir=self.default_dir,
            filetypes=[("CSV files", "*.csv")]
        )
        root.destroy()
        if selected:
            plt.close('all')
            self.current_file = selected
            self.run_visualization()

    def close_application(self, event=None):
        plt.close('all')
        sys.exit()

    def run_visualization(self):
        try:
            df = pd.read_csv(self.current_file)
            df["time"] = pd.to_numeric(df["time"], errors="coerce")
            df["t_rel"] = df["time"] - df["time"].iloc[0]
            if "yaw" in df.columns and "yawd" in df.columns:
                diff = df["yaw"] - df["yawd"]
                df["yaw_corr"] = df["yawd"] + np.arctan2(np.sin(diff), np.cos(diff))
            else:
                df["yaw_corr"] = df["yaw"] if "yaw" in df.columns else 0
            self.render(df)
        except Exception as e:
            print(f"Operational error: {e}")

    def render(self, df):
        t = df["t_rel"]
        t_start, t_end = t.min(), t.max()
        fig, axes = plt.subplots(4, 2, figsize=(16, 10), sharex=True)
        try:
            from screeninfo import get_monitors
            monitors = get_monitors()
            target_monitor = next((m for m in monitors if "ViewSonic" in m.name), monitors[-1])
            manager = plt.get_current_fig_manager()
            if hasattr(manager.window, 'wm_geometry'):
                manager.window.wm_geometry(f"+{target_monitor.x}+{target_monitor.y}")
            manager.window.showMaximized()
        except:
            plt.get_current_fig_manager().full_screen_toggle()
        fig.suptitle(f"UAV Flight Telemetry Analysis: {os.path.basename(self.current_file)}", fontsize=14, fontweight="bold", y=0.975)
        ax_refresh = fig.add_axes([0.74, 0.945, 0.07, 0.03])
        self.btn_refresh = plt.Button(ax_refresh, 'Update', color='#e1f5fe', hovercolor='#b3e5fc')
        self.btn_refresh.on_clicked(self.refresh_latest)
        ax_open = fig.add_axes([0.82, 0.945, 0.07, 0.03])
        self.btn_open = plt.Button(ax_open, 'Open', color='#f0f0f0', hovercolor='#d0d0d0')
        self.btn_open.on_clicked(self.manual_select)
        ax_exit = fig.add_axes([0.90, 0.945, 0.07, 0.03])
        self.btn_exit = plt.Button(ax_exit, 'Exit', color='#ffcccc', hovercolor='#ffaaaa')
        self.btn_exit.on_clicked(self.close_application)
        for i, (m, r, lbl) in enumerate([('x', 'xd', 'X [m]'), ('y', 'yd', 'Y [m]'), ('z', 'zd', 'Z [m]'), ('yaw_corr', 'yawd', 'Yaw [rad]')]):
            axes[i, 0].plot(t, df[m], label='Measured')
            if r in df.columns: axes[i, 0].plot(t, df[r], '--', label='Reference')
            axes[i, 0].set_ylabel(lbl); axes[i, 0].grid(True, alpha=0.7); axes[i, 0].set_xlim(t_start, t_end)
            axes[i, 0].legend(loc='upper right', fontsize='x-small')
        for i, (c, m, r, lbl) in enumerate([('cmd_linx', 'linx_b', 'vxd_b', 'Vx [m/s]'), ('cmd_liny', 'liny_b', 'vyd_b', 'Vy [m/s]'), ('cmd_linz', 'linz_b', 'vzd_b', 'Vz [m/s]'), ('cmd_angz', 'yaw_rate', 'wyawd', 'Yaw rate [rad/s]')]):
            axes[i, 1].plot(t, df[c], ':', color='#e53935', alpha=0.7, label='Command')
            axes[i, 1].plot(t, df[m], color='#1e88e5', label='Measured (Body)')
            if r in df.columns: axes[i, 1].plot(t, df[r], '--', color='#43a047', label='Ref (Body)')
            axes[i, 1].set_ylabel(lbl); axes[i, 1].grid(True, alpha=0.7); axes[i, 1].set_xlim(t_start, t_end)
            if i < 3:
                axes[i, 1].set_ylim(-1.8, 1.8)
            else:
                axes[i, 1].set_ylim(-1.76, 1.76)
            axes[i, 1].legend(loc='upper right', fontsize='x-small')
        plt.tight_layout(rect=[0, 0.02, 1, 0.94])
        plt.show()

if __name__ == "__main__":
    BebopVisualizer()