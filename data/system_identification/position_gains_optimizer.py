#!/usr/bin/env python3

# Automated optimization of position control gains using performance metrics.
"""
Bebop Controller Optimizer
- Trayectoria: waypoints con velocidad cero (4 puntos, 15 s cada uno)
- Modelo identificado Ad/Bd, controlador 15 Hz
- Algoritmos: Differential Evolution (global) · Nelder-Mead (local)
- Función de costo: ISE o ITAE, ponderada por canal
- Optimización en hilo separado, GUI siempre responsive
- Resultado: mejores 16 ganancias + simulación completa
"""

import tkinter as tk
from tkinter import ttk
import numpy as np
import threading
import queue
import time
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.optimize import differential_evolution, minimize

Ad = np.array(

[[ 9.63895373e-01, -6.18407625e-02,  0.00000000e+00,  1.60376855e-02],
 [-8.14310627e-05,  9.99449376e-01,  0.00000000e+00,  5.21436812e-03],
 [ 0.00000000e+00,  0.00000000e+00,  8.57642038e-01,  0.00000000e+00],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  8.22489970e-01],
])

Bd = np.array([

    [ 4.68509248e-02,  9.89534630e-03,  0.00000000e+00, -1.41951368e-02],
    [ 2.04387648e-05,  3.40358601e-02,  0.00000000e+00, -8.51778748e-03],
    [ 0.00000000e+00, 0.00000000e+00,  1.80292391e-01,  0.00000000e+00],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.84677304e-01],
])

DT          = 1.0 / 15.0
DWELL_TIME  = 15.0
N_WAYPOINTS = 4
TOTAL_TIME  = DWELL_TIME * N_WAYPOINTS
OPT_STEPS   = int(TOTAL_TIME / DT)
FULL_STEPS  = OPT_STEPS
T_OPT       = np.arange(OPT_STEPS)  * DT
T_FULL      = np.arange(FULL_STEPS) * DT
DWELL_STEPS = int(DWELL_TIME / DT)

GAIN_NAMES = [
    'ksp_x','ksp_y','ksp_z','ksp_psi',
    'ksd_x','ksd_y','ksd_z','ksd_psi',
    'kp_x', 'kp_y', 'kp_z', 'kp_psi',
    'kd_x', 'kd_y', 'kd_z', 'kd_psi',
]
GAIN_BOUNDS_FULL = [
    (0.0,5.0),(0.0,5.0),(0.0,5.0),(0.0,8.0),
    (0.0,8.0),(0.0,8.0),(0.0,8.0),(0.0,8.0),
    (0.0,5.0),(0.0,5.0),(0.0,5.0),(0.0,8.0),
    (0.0,3.0),(0.0,3.0),(0.0,3.0),(0.0,3.0),
]
GAIN_DEFAULTS = np.array([
    1.0, 1.0, 1.0, 1.0,
    0.5333, 0.4, 0.6, 0.4667,
    0.833, 1.95, 1.0, 2.2667,
    0.00, 0.00, 0.00,   0.00,
])

SYM_PAIRS  = [(0,1),(4,5),(8,9),(12,13)]
SYM_SINGLE = [0,2,3,4,6,7,8,10,11,12,14,15]

def compute_waypoints(t_arr, L):
    """
    Genera xd (ref posición) y dxd (ref velocidad = 0) para los 4 waypoints.
    Cada punto dura DWELL_TIME segundos publicado a 15 Hz.

    Waypoints  (x, y, z):
        WP1  [ L/2,  L/2, 1.2]   yaw =  45°
        WP2  [-L/2,  L/2, 1.2]   yaw = 135°
        WP3  [-L/2, -L/2, 1.2]   yaw =-135°
        WP4  [ 0.0,  0.0, 1.2]   yaw =   0°
    """
    points = np.array([
        [ L/2,  L/2, 1.2],
        [-L/2,  L/2, 1.2],
        [-L/2, -L/2, 1.2],
        [ 0.0,  0.0, 1.2],
    ])
    yaws = np.deg2rad([45.0, 135.0, -135.0, 0.0])

    n   = len(t_arr)
    xd  = np.zeros((n, 4))
    dxd = np.zeros((n, 4))

    for k in range(n):
        wp_idx    = min(int(k // DWELL_STEPS), N_WAYPOINTS - 1)
        xd[k, :3] = points[wp_idx]
        xd[k,  3] = yaws[wp_idx]

    return xd, dxd


def wFb(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])

def run_simulation(gains_vec, xd_arr, dxd_arr, n_steps):
    g = gains_vec
    ksp = np.diag(g[0:4])
    ksd = np.diag(g[4:8])
    kp  = np.diag(g[8:12])
    kd  = np.diag(g[12:16])

    state     = np.zeros(8)
    state[:4] = xd_arr[0]
    w_Ur_prev = np.zeros(4)
    b_Ud_prev = np.zeros(4)
    AdmI      = Ad - np.eye(4)

    pos_h  = np.zeros((n_steps, 4))
    epos_h = np.zeros((n_steps, 4))
    vel_h  = np.zeros((n_steps, 4))
    evel_h = np.zeros((n_steps, 4))

    for k in range(n_steps):
        pos = state[:4];  vb = state[4:];  psi = pos[3]
        w_Xd  = xd_arr[k];  w_dXd = dxd_arr[k]
        Fw = wFb(psi);  Fb = Fw.T
        w_dX  = Fw @ vb
        xtil  = w_Xd - pos
        xtil[3] = np.arctan2(np.sin(xtil[3]), np.cos(xtil[3]))
        w_Ur  = kd @ w_dXd + ksp @ np.tanh(kp @ xtil)
        w_dUr = (w_Ur - w_Ur_prev) / DT
        w_Ur_prev = w_Ur.copy()
        Ad_damp = Fw @ AdmI @ Fb @ w_dX
        rhs  = w_dUr + ksd @ (w_Ur - w_dX) + Ad_damp
        W_Bd = Fw @ Bd
        try:    b_Ud_raw = np.linalg.solve(W_Bd, rhs)
        except: b_Ud_raw = np.zeros(4)
        b_Ud      = 0.6 * b_Ud_raw + 0.4 * b_Ud_prev
        b_Ud_prev = b_Ud.copy()
        u = np.clip(b_Ud, -1.0, 1.0)
        pos_h[k]  = pos
        epos_h[k] = xtil
        vel_h[k]  = vb
        evel_h[k] = vb - Fb @ w_dXd
        vb_next   = Ad @ vb + Bd @ u
        psi_m = psi + 0.5 * DT * vb[3]
        dpos  = wFb(psi_m) @ (0.5*(vb + vb_next))
        state = np.concatenate([pos + DT*dpos, vb_next])

    return dict(pos=pos_h, epos=epos_h, vel=vel_h, evel=evel_h)

def cost_fn(gains_vec, xd, dxd, n_steps, t_arr,
            w_xy, w_z, w_psi, w_vel, cost_type):
    h = run_simulation(gains_vec, xd, dxd, n_steps)
    ep = h['epos'];  ev = h['evel']
    if cost_type == 'ITAE':
        t  = t_arr[:, None]
        c  = (w_xy  * np.sum(t * np.abs(ep[:, :2]))
            + w_z   * np.sum(t * np.abs(ep[:, 2]))
            + w_psi * np.sum(t * np.abs(ep[:, 3]))
            + w_vel * np.sum(t * np.abs(ev))) * DT
    else:
        c  = (w_xy  * np.sum(ep[:, :2]**2)
            + w_z   * np.sum(ep[:, 2]**2)
            + w_psi * np.sum(ep[:, 3]**2)
            + w_vel * np.sum(ev**2)) * DT
    return float(c)

class OptThread(threading.Thread):
    def __init__(self, cfg, result_q, stop_ev):
        super().__init__(daemon=True)
        self.cfg      = cfg
        self.q        = result_q
        self.stop_ev  = stop_ev
        self._n_eval  = 0
        self._best    = np.inf
        self._t0      = None

    def _eval(self, x_reduced):
        if self.stop_ev.is_set():
            raise StopIteration
        x = self._expand(x_reduced)
        c = cost_fn(x,
                    self.cfg['xd'], self.cfg['dxd'],
                    self.cfg['n_steps'], self.cfg['t_arr'],
                    self.cfg['w_xy'],   self.cfg['w_z'],
                    self.cfg['w_psi'],  self.cfg['w_vel'],
                    self.cfg['cost_type'])
        self._n_eval += 1
        if c < self._best:
            self._best = c
            elapsed = time.time() - self._t0
            self.q.put(('progress', self._n_eval, c, x.copy(), elapsed))
        return c

    def _expand(self, xr):
        if not self.cfg['sym_xy']:
            return xr.copy()
        x = np.zeros(16)
        for dst, src_idx in enumerate(SYM_SINGLE):
            x[src_idx] = xr[dst]
        for a, b in SYM_PAIRS:
            x[b] = x[a]
        return x

    def _build_bounds(self):
        if not self.cfg['sym_xy']:
            return GAIN_BOUNDS_FULL
        return [GAIN_BOUNDS_FULL[i] for i in SYM_SINGLE]

    def _x0_reduced(self):
        x0 = GAIN_DEFAULTS.copy()
        if not self.cfg['sym_xy']:
            return x0
        return x0[SYM_SINGLE]

    def run(self):
        self._t0 = time.time()
        bounds   = self._build_bounds()
        x0       = self._x0_reduced()
        algo     = self.cfg['algorithm']

        try:
            if algo == 'DE':
                result = differential_evolution(
                    self._eval,
                    bounds=bounds,
                    maxiter=self.cfg['maxiter'],
                    popsize=self.cfg['popsize'],
                    tol=1e-6,
                    seed=42,
                    init='latinhypercube',
                    mutation=(0.5, 1.2),
                    recombination=0.9,
                    workers=1,
                )
                best_x = self._expand(result.x)
                best_c = result.fun

            elif algo == 'Nelder-Mead':
                result = minimize(
                    self._eval,
                    x0,
                    method='Nelder-Mead',
                    options=dict(maxiter=self.cfg['maxiter']*200,
                                 xatol=1e-4, fatol=1e-4,
                                 adaptive=True),
                )
                best_x = self._expand(result.x)
                best_c = result.fun

            elif algo == 'L-BFGS-B':
                result = minimize(
                    self._eval,
                    x0,
                    method='L-BFGS-B',
                    bounds=bounds,
                    options=dict(maxiter=self.cfg['maxiter']*50,
                                 ftol=1e-9, gtol=1e-6),
                )
                best_x = self._expand(result.x)
                best_c = result.fun

        except StopIteration:
            self.q.put(('stopped',))
            return
        except Exception as e:
            self.q.put(('error', str(e)))
            return

        self.q.put(('done', best_x, best_c, self._n_eval,
                    time.time() - self._t0))


BG       = '#0d0f14'
PANEL_BG = '#11141c'
CARD_BG  = '#13161f'
TEXT_COL = '#c8d0e8'
ACCENT   = '#00ccee'
GOOD     = '#88ee00'
WARN     = '#ffcc00'
BAD      = '#ff3355'
VIOLET   = '#cc88ff'
COLORS   = [ACCENT, BAD, GOOD, WARN]
LABELS   = ['X','Y','Z','ψ']

def apply_dark_matplotlib():
    import matplotlib
    matplotlib.rcParams.update({
        'figure.facecolor': BG,
        'axes.facecolor':   '#0a0c12',
        'axes.edgecolor':   '#1e2530',
        'axes.labelcolor':  TEXT_COL,
        'xtick.color':      '#445566',
        'ytick.color':      '#445566',
        'grid.color':       '#1a2030',
        'grid.linewidth':   0.8,
        'text.color':       TEXT_COL,
        'legend.facecolor': '#0d0f18',
        'legend.edgecolor': '#1e2530',
        'legend.fontsize':  7,
    })


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Bebop Controller Optimizer — Waypoints · 15 Hz")
        self.configure(bg=BG)
        try:    self.state('zoomed')
        except: self.attributes('-zoomed', True)

        apply_dark_matplotlib()

        self._opt_thread  = None
        self._result_q    = queue.Queue()
        self._stop_ev     = threading.Event()
        self._best_gains  = GAIN_DEFAULTS.copy()
        self._eval_log    = []
        self._opt_running = False

        self._traj_vars   = {}
        self._weight_vars = {}
        self._algo_var    = tk.StringVar(value='DE')
        self._cost_var    = tk.StringVar(value='ISE')
        self._sym_var     = tk.BooleanVar(value=True)
        self._maxiter_var = tk.IntVar(value=80)
        self._popsize_var = tk.IntVar(value=8)

        self._build_ui()
        self._poll_queue()

    def _build_ui(self):
        hdr = tk.Frame(self, bg=PANEL_BG, height=46)
        hdr.pack(side='top', fill='x')
        hdr.pack_propagate(False)
        tk.Label(hdr, text="BEBOP CONTROLLER OPTIMIZER",
                 font=('Courier',13,'bold'), bg=PANEL_BG, fg=ACCENT
                 ).pack(side='left', padx=18, pady=10)
        tk.Label(hdr, text="waypoints · Ad/Bd · 15 Hz",
                 font=('Courier',9), bg=PANEL_BG, fg='#334455'
                 ).pack(side='left', pady=10)
        self._lbl_status = tk.Label(hdr, text="LISTO",
                                    font=('Courier',10,'bold'),
                                    bg=PANEL_BG, fg=GOOD)
        self._lbl_status.pack(side='right', padx=16)

        btn_bar = tk.Frame(self, bg=PANEL_BG, height=40)
        btn_bar.pack(side='top', fill='x')
        btn_bar.pack_propagate(False)

        bkw = dict(font=('Courier',9,'bold'), relief='flat', cursor='hand2',
                   padx=14, pady=5)
        self._btn_start = tk.Button(btn_bar, text="▶  INICIAR",
                                    bg=GOOD, fg='#0d0f14',
                                    command=self._start_opt, **bkw)
        self._btn_start.pack(side='left', padx=8, pady=4)
        self._btn_stop = tk.Button(btn_bar, text="■  DETENER",
                                   bg=BAD,  fg='white',
                                   command=self._stop_opt,  **bkw,
                                   state='disabled')
        self._btn_stop.pack(side='left', padx=4, pady=4)
        self._btn_sim = tk.Button(btn_bar, text="📊  VER SIMULACIÓN",
                                  bg='#1e2a3a', fg=ACCENT,
                                  command=self._show_full_sim, **bkw)
        self._btn_sim.pack(side='left', padx=8, pady=4)
        self._btn_copy = tk.Button(btn_bar, text="📋  COPIAR PARAMS",
                                   bg='#1e2a3a', fg=WARN,
                                   command=self._copy_params, **bkw)
        self._btn_copy.pack(side='left', padx=4, pady=4)

        self._lbl_eval = tk.Label(btn_bar, text="",
                                  font=('Courier',8), bg=PANEL_BG, fg=TEXT_COL)
        self._lbl_eval.pack(side='right', padx=16)

        body = tk.Frame(self, bg=BG)
        body.pack(fill='both', expand=True)

        self._left = tk.Frame(body, bg=PANEL_BG, width=290)
        self._left.pack(side='left', fill='y')
        self._left.pack_propagate(False)
        self._build_left_panel()

        right = tk.Frame(body, bg=BG)
        right.pack(side='left', fill='both', expand=True)
        self._build_right_panel(right)

    def _build_left_panel(self):
        canvas = tk.Canvas(self._left, bg=PANEL_BG, highlightthickness=0)
        sb     = ttk.Scrollbar(self._left, orient='vertical',
                               command=canvas.yview)
        inner  = tk.Frame(canvas, bg=PANEL_BG)
        inner.bind('<Configure>',
                   lambda e: canvas.configure(
                       scrollregion=canvas.bbox('all')))
        canvas.create_window((0,0), window=inner, anchor='nw')
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side='left', fill='both', expand=True)
        sb.pack(side='right', fill='y')
        canvas.bind_all('<MouseWheel>',
                        lambda e: canvas.yview_scroll(
                            int(-1*(e.delta/120)), 'units'))

        def section(title, color=ACCENT):
            tk.Frame(inner, bg='#1a2030', height=1).pack(
                fill='x', padx=6, pady=(10,2))
            tk.Label(inner, text=f"── {title} ──",
                     font=('Courier',8,'bold'),
                     bg=PANEL_BG, fg=color).pack(anchor='w', padx=10, pady=(2,0))

        section("TRAYECTORIA", VIOLET)

        v_L = tk.DoubleVar(value=1.0)
        self._traj_vars['L'] = v_L
        row = tk.Frame(inner, bg=PANEL_BG)
        row.pack(fill='x', padx=10, pady=2)
        tk.Label(row, text='L [m]', font=('Courier',8),
                 bg=PANEL_BG, fg=TEXT_COL, width=11, anchor='w').pack(side='left')
        lbl_L = tk.Label(row, text=f"{v_L.get():.2f}",
                         font=('Courier',8,'bold'),
                         bg=PANEL_BG, fg=VIOLET, width=7, anchor='e')
        lbl_L.pack(side='right')
        ttk.Scale(row, from_=0.2, to=3.0, orient='horizontal',
                  variable=v_L).pack(fill='x')
        v_L.trace_add('write', lambda *_: lbl_L.config(
            text=f"{v_L.get():.2f}"))

        info = tk.Frame(inner, bg=CARD_BG)
        info.pack(fill='x', padx=10, pady=(4,8))
        for line in [
            "  WP   X      Y      Z    Yaw",
            "   1   L/2    L/2   1.2   45°",
            "   2  -L/2    L/2   1.2  135°",
            "   3  -L/2   -L/2   1.2 -135°",
            "   4   0.0    0.0   1.2    0°",
            f"  hold {DWELL_TIME:.0f}s · {DWELL_STEPS} pasos  dxd=0",
        ]:
            tk.Label(info, text=line, font=('Courier',7),
                     bg=CARD_BG, fg='#556677', anchor='w'
                     ).pack(fill='x', padx=4, pady=1)

        section("FUNCIÓN DE COSTO", WARN)
        weight_defs = [('w_xy', 0.0, 5.0, 1.0, ACCENT),
                       ('w_z',  0.0, 5.0, 1.0, GOOD),
                       ('w_psi',0.0, 5.0, 0.5, WARN),
                       ('w_vel',0.0, 2.0, 0.2, BAD)]
        for n, lo, hi, d, col in weight_defs:
            v = tk.DoubleVar(value=d)
            self._weight_vars[n] = v
            row = tk.Frame(inner, bg=PANEL_BG)
            row.pack(fill='x', padx=10, pady=2)
            tk.Label(row, text=n, font=('Courier',8),
                     bg=PANEL_BG, fg=TEXT_COL, width=11, anchor='w'
                     ).pack(side='left')
            lbl_v = tk.Label(row, text=f"{d:.3f}",
                             font=('Courier',8,'bold'),
                             bg=PANEL_BG, fg=col, width=7, anchor='e')
            lbl_v.pack(side='right')
            ttk.Scale(row, from_=lo, to=hi, orient='horizontal',
                      variable=v).pack(fill='x')
            def _upd(*_, vv=v, l=lbl_v):
                l.config(text=f"{vv.get():.3f}")
            v.trace_add('write', _upd)

        row_ct = tk.Frame(inner, bg=PANEL_BG)
        row_ct.pack(fill='x', padx=10, pady=4)
        tk.Label(row_ct, text="Tipo:", font=('Courier',8),
                 bg=PANEL_BG, fg=TEXT_COL).pack(side='left')
        for ct in ('ISE','ITAE'):
            tk.Radiobutton(row_ct, text=ct, variable=self._cost_var,
                           value=ct, bg=PANEL_BG, fg=WARN,
                           selectcolor=BG, activebackground=PANEL_BG,
                           font=('Courier',8)).pack(side='left', padx=6)

        section("OPTIMIZADOR", GOOD)

        row_al = tk.Frame(inner, bg=PANEL_BG)
        row_al.pack(fill='x', padx=10, pady=4)
        tk.Label(row_al, text="Algoritmo:", font=('Courier',8),
                 bg=PANEL_BG, fg=TEXT_COL).pack(side='left')
        for al in ('DE','Nelder-Mead','L-BFGS-B'):
            tk.Radiobutton(row_al, text=al, variable=self._algo_var,
                           value=al, bg=PANEL_BG, fg=GOOD,
                           selectcolor=BG, activebackground=PANEL_BG,
                           font=('Courier',8)).pack(side='left', padx=4)

        row_sy = tk.Frame(inner, bg=PANEL_BG)
        row_sy.pack(fill='x', padx=10, pady=2)
        tk.Checkbutton(row_sy, text="Simétrico XY (ksp_x=ksp_y, etc.)",
                       variable=self._sym_var,
                       bg=PANEL_BG, fg=TEXT_COL, selectcolor=BG,
                       activebackground=PANEL_BG,
                       font=('Courier',8)).pack(side='left')

        for label, var, lo, hi in [
                ("Max iter", self._maxiter_var, 10, 300),
                ("Popsize",  self._popsize_var,  4,  20)]:
            row = tk.Frame(inner, bg=PANEL_BG)
            row.pack(fill='x', padx=10, pady=2)
            tk.Label(row, text=label, font=('Courier',8),
                     bg=PANEL_BG, fg=TEXT_COL, width=11, anchor='w'
                     ).pack(side='left')
            lbl_v = tk.Label(row, text=str(var.get()),
                             font=('Courier',8,'bold'),
                             bg=PANEL_BG, fg=GOOD, width=7, anchor='e')
            lbl_v.pack(side='right')
            ttk.Scale(row, from_=lo, to=hi, orient='horizontal',
                      variable=var).pack(fill='x')
            def _upd(*_, vv=var, l=lbl_v):
                l.config(text=str(int(vv.get())))
            var.trace_add('write', _upd)

        section("MEJORES PARÁMETROS", ACCENT)
        self._params_text = tk.Text(inner, height=20, width=28,
                                    bg=CARD_BG, fg=ACCENT,
                                    font=('Courier',8),
                                    relief='flat', padx=6, pady=4,
                                    state='disabled')
        self._params_text.pack(fill='x', padx=8, pady=4)
        self._update_params_text(GAIN_DEFAULTS)

    def _build_right_panel(self, parent):
        top = tk.Frame(parent, bg=BG, height=220)
        top.pack(side='top', fill='x')
        top.pack_propagate(False)

        self._fig_prog = Figure(figsize=(1,1), dpi=95)
        self._fig_prog.patch.set_facecolor(BG)
        self._ax_prog  = self._fig_prog.add_subplot(111)
        self._ax_prog.set_facecolor('#0a0c12')
        self._ax_prog.set_title("Progreso de optimización",
                                fontsize=9, color=TEXT_COL)
        self._ax_prog.set_xlabel("Evaluaciones", fontsize=8)
        self._ax_prog.set_ylabel("Costo (mejor)", fontsize=8)
        self._ax_prog.grid(True)
        self._fig_prog.tight_layout(pad=1.8)
        c1 = FigureCanvasTkAgg(self._fig_prog, master=top)
        c1.get_tk_widget().pack(fill='both', expand=True)
        self._canvas_prog = c1

        bot = tk.Frame(parent, bg=BG)
        bot.pack(side='top', fill='both', expand=True)

        self._fig_sim = Figure(figsize=(1,1), dpi=95)
        self._fig_sim.patch.set_facecolor(BG)
        self._axes_sim = []
        titles = ["Posición [m/rad]",
                  "Error posición [m/rad]",
                  "Vel. body [m/s·rad/s]",
                  "Error vel. [m/s·rad/s]"]
        for i in range(4):
            ax = self._fig_sim.add_subplot(2,2,i+1)
            ax.set_title(titles[i], fontsize=8, color=TEXT_COL, pad=3)
            ax.set_facecolor('#0a0c12')
            ax.grid(True)
            ax.set_xlabel('t [s]', fontsize=7)
            self._axes_sim.append(ax)
        self._fig_sim.tight_layout(pad=1.8)
        c2 = FigureCanvasTkAgg(self._fig_sim, master=bot)
        c2.get_tk_widget().pack(fill='both', expand=True)
        self._canvas_sim = c2

    def _get_traj(self):
        return {k: v.get() for k, v in self._traj_vars.items()}

    def _get_weights(self):
        return {k: v.get() for k, v in self._weight_vars.items()}

    def _update_params_text(self, gains):
        self._params_text.config(state='normal')
        self._params_text.delete('1.0', 'end')
        lines = []
        groups = [('Ksp', 0,4), ('Ksd', 4,8), ('Kp', 8,12), ('Kd', 12,16)]
        for grp, a, b in groups:
            lines.append(f"  ── {grp} ──")
            for i in range(a, b):
                lines.append(f"  {GAIN_NAMES[i]:10s} {gains[i]:8.5f}")
        self._params_text.insert('end', '\n'.join(lines))
        self._params_text.config(state='disabled')

    def _copy_params(self):
        lines = [f"{n} = {self._best_gains[i]:.6f}"
                 for i, n in enumerate(GAIN_NAMES)]
        self.clipboard_clear()
        self.clipboard_append('\n'.join(lines))
        self._lbl_status.config(text="COPIADO AL PORTAPAPELES", fg=WARN)
        self.after(2000, lambda: self._lbl_status.config(
            text="LISTO", fg=GOOD))

    def _start_opt(self):
        if self._opt_running:
            return
        self._eval_log.clear()
        self._stop_ev.clear()

        tp  = self._get_traj()
        w   = self._get_weights()
        xd, dxd = compute_waypoints(T_OPT, tp['L'])

        cfg = dict(
            xd=xd, dxd=dxd, n_steps=OPT_STEPS, t_arr=T_OPT,
            w_xy=w['w_xy'], w_z=w['w_z'],
            w_psi=w['w_psi'], w_vel=w['w_vel'],
            cost_type=self._cost_var.get(),
            algorithm=self._algo_var.get(),
            maxiter=int(self._maxiter_var.get()),
            popsize=int(self._popsize_var.get()),
            sym_xy=self._sym_var.get(),
        )
        self._opt_thread = OptThread(cfg, self._result_q, self._stop_ev)
        self._opt_thread.start()
        self._opt_running = True
        self._btn_start.config(state='disabled')
        self._btn_stop.config(state='normal')
        self._lbl_status.config(text="OPTIMIZANDO...", fg=WARN)

    def _stop_opt(self):
        self._stop_ev.set()
        self._lbl_status.config(text="DETENIENDO...", fg=BAD)

    def _poll_queue(self):
        try:
            while True:
                msg = self._result_q.get_nowait()
                self._handle_msg(msg)
        except queue.Empty:
            pass
        self.after(120, self._poll_queue)

    def _handle_msg(self, msg):
        kind = msg[0]

        if kind == 'progress':
            _, n_eval, cost, gains, elapsed = msg
            self._eval_log.append((n_eval, cost))
            self._best_gains = gains.copy()
            self._update_params_text(gains)
            self._update_progress_plot()
            self._lbl_eval.config(
                text=f"eval {n_eval:5d}  costo {cost:.5f}  {elapsed:.0f}s")

        elif kind == 'done':
            _, best_x, best_c, n_eval, elapsed = msg
            self._best_gains = best_x.copy()
            self._update_params_text(best_x)
            self._opt_running = False
            self._btn_start.config(state='normal')
            self._btn_stop.config(state='disabled')
            self._lbl_status.config(
                text=f"TERMINADO · costo={best_c:.5f} · {elapsed:.1f}s", fg=GOOD)
            self._show_full_sim()

        elif kind == 'stopped':
            self._opt_running = False
            self._btn_start.config(state='normal')
            self._btn_stop.config(state='disabled')
            self._lbl_status.config(text="DETENIDO", fg=WARN)

        elif kind == 'error':
            self._opt_running = False
            self._btn_start.config(state='normal')
            self._btn_stop.config(state='disabled')
            self._lbl_status.config(text=f"ERROR: {msg[1]}", fg=BAD)

    def _update_progress_plot(self):
        if not self._eval_log:
            return
        ns, cs = zip(*self._eval_log)
        ax = self._ax_prog
        ax.cla()
        ax.set_facecolor('#0a0c12')
        ax.set_title("Progreso de optimización  (mejor costo encontrado)",
                     fontsize=9, color=TEXT_COL)
        ax.set_xlabel("Evaluaciones", fontsize=8)
        ax.set_ylabel("Costo", fontsize=8)
        ax.plot(ns, cs, color=ACCENT, lw=1.5)
        ax.fill_between(ns, cs, alpha=0.12, color=ACCENT)
        ax.scatter([ns[-1]], [cs[-1]], color=WARN, s=30, zorder=5)
        ax.grid(True)
        for sp in ax.spines.values():
            sp.set_edgecolor('#1e2530')
        self._fig_prog.tight_layout(pad=1.8)
        self._canvas_prog.draw_idle()

    def _show_full_sim(self):
        tp = self._get_traj()
        xd, dxd = compute_waypoints(T_FULL, tp['L'])
        h  = run_simulation(self._best_gains, xd, dxd, FULL_STEPS)
        t  = T_FULL

        wp_times = [DWELL_TIME * i for i in range(1, N_WAYPOINTS)]

        ax0, ax1, ax2, ax3 = self._axes_sim
        vl = ['vx','vy','vz','r']

        def clr(ax, title):
            ax.cla(); ax.set_facecolor('#0a0c12')
            ax.set_title(title, fontsize=8, color=TEXT_COL, pad=3)
            ax.grid(True); ax.set_xlabel('t [s]', fontsize=7)
            for sp in ax.spines.values():
                sp.set_edgecolor('#1e2530')
            for wt in wp_times:
                ax.axvline(wt, color='#334455', lw=0.8, ls='--')

        clr(ax0, "Posición [m/rad]")
        for i, l in enumerate(LABELS):
            ax0.plot(t, h['pos'][:,i], color=COLORS[i], lw=1.2, label=l)
            ax0.plot(t, xd[:,i],       color=COLORS[i], lw=0.7,
                     ls='--', alpha=0.5, label=f'{l} ref')
        ax0.legend(ncol=4, loc='upper right', fontsize=6)

        clr(ax1, "Error posición [m/rad]")
        ax1.axhline(0, color='#334', lw=0.8)
        for i, l in enumerate(LABELS):
            ax1.plot(t, h['epos'][:,i], color=COLORS[i], lw=1.2, label=f'e{l}')
        ax1.legend(ncol=4, loc='upper right', fontsize=6)

        clr(ax2, "Vel. body [m/s·rad/s]")
        for i, l in enumerate(vl):
            ax2.plot(t, h['vel'][:,i], color=COLORS[i], lw=1.2, label=l)
        ax2.legend(ncol=4, loc='upper right', fontsize=6)

        clr(ax3, "Error vel. [m/s·rad/s]")
        ax3.axhline(0, color='#334', lw=0.8)
        for i, l in enumerate(vl):
            ax3.plot(t, h['evel'][:,i], color=COLORS[i], lw=1.2, label=f'e{l}')
        ax3.legend(ncol=4, loc='upper right', fontsize=6)

        self._fig_sim.tight_layout(pad=1.8)
        self._canvas_sim.draw()


if __name__ == "__main__":
    app = App()
    app.mainloop()