%% =========================================================
%  bebop_sim_tuning.m
%  Simulación cerrada: Dinámica Webots + Controlador PID
%  Basado en BebopWebotsFullSim.py + BebopVelocityController.py
%  ---------------------------------------------------------
%  Frecuencias:
%    Dinámica  : SIM_HZ  (equivalente a BasicTimeStep, e.g. 32 ms -> ~31.25 Hz)
%    Controlador: CTRL_HZ = 30 Hz
%  =========================================================
clear; clc; close all;

%% ── Parámetros de tiempo ─────────────────────────────────
SIM_TIMESTEP_MS = 32;               % ms  (BasicTimeStep de Webots)
SIM_HZ   = 1000 / SIM_TIMESTEP_MS; % Hz de la dinámica
CTRL_HZ  = 30.0;                    % Hz del controlador (BebopVelocityController)
T_SIM    = 60.0;                    % Duración total [s]

dt_sim  = 1 / SIM_HZ;
dt_ctrl = 1 / CTRL_HZ;

%% ── Modelo de la dinámica (BebopWebotsFullSim) ───────────
% xddot = F(yaw) * Ku * u  -  Kv * xdot
% estado x = [px, py, pz, yaw]   xdot = [vx, vy, vz, dyaw]  (inerciales)
Ku = diag([0.8417, 0.8354, 3.966, 9.8524]);
Kv = diag([0.18227, 0.17095, 4.001, 4.7295]);
z_ground = 0.05;
max_tilt = deg2rad(5.0);   % no afecta la dinámica traslacional aquí

%% ── Parámetros PID (BebopVelocityController) ────────────
% Formato: [kp, ki, kd, e_int_max]
PID = struct();
PID.x   = [3.5,  0.01, 0.55,  0.2];
PID.y   = [3.5,  0.01, 0.55,  0.2];
PID.z   = [1.0,  0.05, 0.3,  0.3];
PID.yaw = [0.8,  0.02, 0.2,  0.3];

tau_d    = 0.005;   % filtro derivativo
ff_w     = 0.85;    % peso feedforward

% Saturaciones de referencia (inercial)
VX_MAX   = 4.5;
VY_MAX   = 4.5;
VZ_MAX   = 1.0;
DYAW_MAX = deg2rad(100.0);

% Velocidades de estado estacionario (Ku/Kv diagonal)
vss = Ku(1,1)/Kv(1,1);   % vx_ss
wss = Ku(2,2)/Kv(2,2);   % vy_ss
zss = Ku(3,3)/Kv(3,3);   % vz_ss
yss = Ku(4,4)/Kv(4,4);   % dyaw_ss

%% ── Trayectoria de prueba (velocity_trajectory.py) ───────
V_XY  = 1.0;
V_Z   = 0.5;
V_YAW = deg2rad(30.0);
T_RAMP   = 5.0;
T_CRUISE = 3.0;
T_HOVER  = 2.0;

seg_dur = 2*T_RAMP + T_CRUISE;  % duración de un segmento

% [nombre, v_peak, eje (1=vx,2=vy,3=vz,4=dyaw)]
segments = {
    '+X forward',   V_XY,   1;
    '-X backward', -V_XY,   1;
    '+Y right',     V_XY,   2;
    '-Y left',     -V_XY,   2;
    '+Z up',        V_Z,    3;
    '-Z down',     -V_Z,    3;
    '+Yaw CW',      V_YAW,  4;
    '-Yaw CCW',    -V_YAW,  4;
};

% Pre-calcula referencia en vector de tiempo de CONTROL
t_ctrl_vec = 0 : dt_ctrl : T_SIM;
ref_i = zeros(4, length(t_ctrl_vec));   % [vx;vy;vz;dyaw] inercial

t_cursor = 2.0;   % hover inicial
for s = 1:size(segments,1)
    v_peak = segments{s,2};
    ax     = segments{s,3};
    for k = 1:length(t_ctrl_vec)
        tc = t_ctrl_vec(k) - t_cursor;
        ref_i(ax,k) = ref_i(ax,k) + cosine_trapezoid(tc, v_peak, T_RAMP, T_CRUISE);
    end
    t_cursor = t_cursor + seg_dur + T_HOVER;
end

%% ── Inicialización ───────────────────────────────────────
N_sim = round(T_SIM / dt_sim);
t_sim_vec = (0:N_sim-1) * dt_sim;

% Estado de la dinámica
x    = [0; 0; z_ground; 0];   % [px,py,pz,yaw]
xdot = zeros(4,1);             % [vx,vy,vz,dyaw]
u    = zeros(4,1);             % señal de control [-1,1]

% Estado del PID (e_int y e_prev para cada canal)
% Orden: x, y, z, yaw
e_int  = zeros(4,1);
e_prev = zeros(4,1);
d_filt = zeros(4,1);

% Logging
log_t    = zeros(1, N_sim);
log_x    = zeros(4, N_sim);
log_xdot = zeros(4, N_sim);
log_u    = zeros(4, N_sim);
log_ref  = zeros(4, N_sim);   % referencia body-frame

mode = "FLYING";   % para esta simulación arrancamos directo en vuelo
ctrl_idx = 1;      % índice en t_ctrl_vec para el controlador

%% ── Loop de simulación ───────────────────────────────────
ctrl_accum = 0.0;

for k = 1:N_sim
    t = (k-1) * dt_sim;

    %% — CONTROLADOR (se ejecuta a CTRL_HZ) ——————————————
    ctrl_accum = ctrl_accum + dt_sim;
    if ctrl_accum >= dt_ctrl - 1e-9
        ctrl_accum = 0.0;

        % Referencia inercial en el tick actual
        ci = min(ctrl_idx, length(t_ctrl_vec));
        ref = ref_i(:, ci);
        ctrl_idx = ctrl_idx + 1;

        % Saturar referencia
        ref(1) = clamp(ref(1), -VX_MAX,   VX_MAX);
        ref(2) = clamp(ref(2), -VY_MAX,   VY_MAX);
        ref(3) = clamp(ref(3), -VZ_MAX,   VZ_MAX);
        ref(4) = clamp(ref(4), -DYAW_MAX, DYAW_MAX);

        % Rotación inercial → body
        yaw = x(4);
        cp = cos(yaw); sp = sin(yaw);
        ref_vx_b =  cp*ref(1) + sp*ref(2);
        ref_vy_b = -sp*ref(1) + cp*ref(2);

        % Velocidades medidas body-frame (simulador las publica directo)
        vx_b = xdot(1);  % ya están en body frame por la integración inercial+rot
        vy_b = xdot(2);
        vz_b = xdot(3);
        dyaw_b = xdot(4);

        % Errores
        ex   = ref_vx_b  - vx_b;
        ey   = ref_vy_b  - vy_b;
        ez   = ref(3)    - vz_b;
        eyaw = ref(4)    - dyaw_b;
        errs = [ex; ey; ez; eyaw];
        refs_b = [ref_vx_b; ref_vy_b; ref(3); ref(4)];

        % Reset integral si setpoint ~= 0
        if abs(ref(1)) < 0.01, e_int(1) = 0; end
        if abs(ref(2)) < 0.01, e_int(2) = 0; end

        % PID con filtro derivativo
        kparams = [PID.x; PID.y; PID.z; PID.yaw];  % [kp,ki,kd,emax]
        vss_vec = [vss; wss; zss; yss];
        u_new   = zeros(4,1);
        for ch = 1:4
            kp = kparams(ch,1); ki = kparams(ch,2);
            kd = kparams(ch,3); emax = kparams(ch,4);
            e  = errs(ch);

            % Integral con anti-windup
            e_int(ch) = clamp(e_int(ch) + e*dt_ctrl, -emax, emax);

            % Derivada filtrada
            e_dot_raw = (e - e_prev(ch)) / dt_ctrl;
            alpha = dt_ctrl / (tau_d + dt_ctrl);
            d_filt(ch) = (1-alpha)*d_filt(ch) + alpha*e_dot_raw;
            e_prev(ch) = e;

            pid_out = kp*e + ki*e_int(ch) + kd*d_filt(ch);

            % Feedforward + PID
            u_new(ch) = (ff_w * refs_b(ch) / vss_vec(ch)) + pid_out;
        end

        % Saturar salida [-1, 1]
        u = clamp(u_new, -1, 1);
    end

    %% — DINÁMICA (se ejecuta a SIM_HZ) ——————————————————
    yaw = x(4);
    c   = cos(yaw); s = sin(yaw);
    F   = [c, -s, 0, 0;
           s,  c, 0, 0;
           0,  0, 1, 0;
           0,  0, 0, 1];

    xddot = F*(Ku*u) - Kv*xdot;
    xdot  = xdot + xddot*dt_sim;
    x     = x    + xdot *dt_sim;

    if x(3) < z_ground
        x(3)    = z_ground;
        xdot(3) = 0;
    end

    %% — Log ——————————————————————————————————————————————
    log_t(k)    = t;
    log_x(:,k)  = x;
    log_xdot(:,k) = xdot;
    log_u(:,k)  = u;

    % Referencia body-frame (para graficar) — índice protegido contra 0
    ci2  = max(1, min(ctrl_idx - 1, length(t_ctrl_vec)));
    ref2 = ref_i(:, ci2);
    yaw2 = x(4); cp2 = cos(yaw2); sp2 = sin(yaw2);
    log_ref(1,k) =  cp2*ref2(1) + sp2*ref2(2);
    log_ref(2,k) = -sp2*ref2(1) + cp2*ref2(2);
    log_ref(3,k) = ref2(3);
    log_ref(4,k) = ref2(4);
end

%% ── Figura 1: Velocidades ────────────────────────────────
cRef  = [0.17, 0.63, 0.17];   % verde
cMeas = [0.12, 0.47, 0.71];   % azul
cCmd  = [0.84, 0.15, 0.16];   % rojo

labels_v   = {'Vx body [m/s]', 'Vy body [m/s]', 'Vz [m/s]', 'Yaw rate [rad/s]'};
ylims_v    = [-1.8, 1.8; -1.8, 1.8; -0.9, 0.9; -0.65, 0.65];

fig1 = figure('Name','Velocity Controller Tuning','Position',[60 60 1200 750]);
for i = 1:4
    ax = subplot(4,1,i);
    hold on; grid on; box on;
    hR = plot(log_t, log_ref(i,:),   '--', 'Color', cRef,  'LineWidth', 1.6);
    hM = plot(log_t, log_xdot(i,:),  '-',  'Color', cMeas, 'LineWidth', 1.2);
    hC = plot(log_t, log_u(i,:),     ':',  'Color', cCmd,  'LineWidth', 1.1);
    yline(0, 'Color', [0.6 0.6 0.6], 'LineWidth', 0.5);
    ylabel(labels_v{i}, 'FontSize', 9);
    ylim(ylims_v(i,:));
    xlim([0 T_SIM]);
    set(ax, 'XTickLabel', []);
    if i == 1
        legend([hR hM hC], {'Reference (body)', 'Measured (body)', 'Command [-1,1]'}, ...
               'Location', 'northeast', 'FontSize', 8, 'Box', 'off');
    end

    % Líneas verticales en cada transición de segmento
    t_cursor_plot = 2.0;
    for s = 1:size(segments,1)
        xline(t_cursor_plot,              ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8, 'HandleVisibility','off');
        xline(t_cursor_plot + seg_dur,    ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8, 'HandleVisibility','off');
        if i == 1
            text(t_cursor_plot + seg_dur/2, ylims_v(i,2)*0.82, segments{s,1}, ...
                 'FontSize', 6.5, 'HorizontalAlignment', 'center', 'Color', [0.35 0.35 0.35]);
        end
        t_cursor_plot = t_cursor_plot + seg_dur + T_HOVER;
    end
end
set(subplot(4,1,4), 'XTickLabelMode', 'auto');
xlabel('Time [s]', 'FontSize', 10);
sgtitle(sprintf('Velocity Controller  |  Sim: %g Hz   Controller: %.0f Hz', SIM_HZ, CTRL_HZ), ...
        'FontSize', 12, 'FontWeight', 'bold');

%% ── Figura 2: Posición + Yaw ─────────────────────────────
labels_p = {'px [m]', 'py [m]', 'pz [m]', 'Yaw [deg]'};
log_x_plot = log_x;
log_x_plot(4,:) = rad2deg(log_x(4,:));

fig2 = figure('Name','Drone Position & Yaw','Position',[80 80 1200 600]);
for i = 1:4
    ax2 = subplot(4,1,i);
    hold on; grid on; box on;
    plot(log_t, log_x_plot(i,:), 'Color', cMeas, 'LineWidth', 1.3);
    yline(0, 'Color', [0.6 0.6 0.6], 'LineWidth', 0.5);
    ylabel(labels_p{i}, 'FontSize', 9);
    xlim([0 T_SIM]);
    if i < 4, set(ax2, 'XTickLabelMode', 'auto'); end
end
xlabel('Time [s]', 'FontSize', 10);
sgtitle('Drone Position & Heading', 'FontSize', 12, 'FontWeight', 'bold');

%% ── Métricas por segmento ────────────────────────────────
fprintf('\n%-18s  %8s  %8s  %8s  %8s\n', 'Segment', 'Overshoot', 'Rise(s)', 'RMSE', 'Settle(s)');
fprintf('%s\n', repmat('-',1,62));
t_cursor2 = 2.0;
for s = 1:size(segments,1)
    ax    = segments{s,3};
    vpk   = segments{s,2};
    t0    = t_cursor2;
    t1    = t_cursor2 + seg_dur;

    idx = log_t >= t0 & log_t <= t1;
    t_seg  = log_t(idx);
    v_meas = log_xdot(ax, idx);
    v_ref  = log_ref(ax, idx);

    if isempty(t_seg)
        t_cursor2 = t_cursor2 + seg_dur + T_HOVER;
        continue;
    end

    rmse = sqrt(mean((v_meas - v_ref).^2));

    % Overshoot respecto al peak
    if vpk > 0
        os = (max(v_meas) - vpk) / abs(vpk) * 100;
    else
        os = (min(v_meas) - vpk) / abs(vpk) * 100;
    end

    % Tiempo de subida (10%→90% del pico)
    thresh_lo = 0.1*abs(vpk);
    thresh_hi = 0.9*abs(vpk);
    idx10 = find(abs(v_meas) >= thresh_lo, 1, 'first');
    idx90 = find(abs(v_meas) >= thresh_hi, 1, 'first');
    if ~isempty(idx10) && ~isempty(idx90)
        t_rise = t_seg(idx90) - t_seg(idx10);
    else
        t_rise = NaN;
    end

    % Tiempo de establecimiento (±5% banda)
    band = 0.05*abs(vpk);
    in_band = abs(v_meas - vpk) <= band;
    settled_idx = find(~in_band, 1, 'last');
    if ~isempty(settled_idx) && settled_idx < length(t_seg)
        t_settle = t_seg(settled_idx) - t0;
    else
        t_settle = NaN;
    end

    fprintf('%-18s  %7.1f%%  %7.2fs  %7.3f  %7.2fs\n', ...
        segments{s,1}, os, t_rise, rmse, t_settle);

    t_cursor2 = t_cursor2 + seg_dur + T_HOVER;
end

%% ── Funciones auxiliares ─────────────────────────────────
function v = cosine_trapezoid(t, v_peak, t_ramp, t_cruise)
    t_brake = t_ramp + t_cruise;
    t_end   = t_brake + t_ramp;
    if t <= 0
        v = 0;
    elseif t <= t_ramp
        x = t / t_ramp;
        v = v_peak * 0.5*(1 - cos(pi*x));
    elseif t <= t_brake
        v = v_peak;
    elseif t <= t_end
        x = (t - t_brake) / t_ramp;
        v = v_peak * (1 - 0.5*(1 - cos(pi*x)));
    else
        v = 0;
    end
end

function y = clamp(x, lo, hi)
    if numel(lo) == 1
        y = max(lo, min(hi, x));
    else
        y = max(lo, min(hi, x));
    end
end