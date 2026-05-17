clear; close all; clc;

Ku = diag([0.8417,  0.8354,  3.9660,  9.8524]);
Kv = diag([0.18227, 0.17095, 4.00100, 4.72950]);

PHASE_DUR    = 4.0;
N_PHASES     = 5;
PHASE_COLORS = [
    0.933 0.933 0.996;
    0.882 0.961 0.933;
    0.980 0.925 0.906;
    0.980 0.933 0.855;
    0.945 0.937 0.910;
];


C_MEAS_V = [0.000 0.447 0.698];
C_MOD_V  = [0.700 0.090 0.090];

C_MEAS_P = [0.549 0.114 0.255];   
C_MOD_P  = [0.929 0.569 0.388];   

C_CTRL = [0.333 0.333 0.333];

FIG_W = 9;
FIG_H = 18;

S     = load('/home/brayan/ros2_ws/src/neroControl/data/sysid_data.mat');
t     = S.t(:);
u     = S.u;
pos   = S.pos;
vel_w = S.vel;

DISCARD = 10;
t     = t    (DISCARD+1 : end);
u     = u    (DISCARD+1 : end, :);
pos   = pos  (DISCARD+1 : end, :);
vel_w = vel_w(DISCARD+1 : end, :);



[v_p_b, p_p_w, v_m_b] = simulate_stepbystep(t, u, pos, vel_w, Ku, Kv);

vel_labels = {'vx','vy','vz','vpsi'};
pos_labels = {'etax','etay','etaz','etapsi'};

valid_v = ~isnan(v_p_b(:,1));
valid_p = ~isnan(p_p_w(:,1));

fprintf('\n===== RMSE & R² — Velocidad (cuerpo) =====\n');
for i = 1:4
    meas = v_m_b(valid_v, i);
    pred = v_p_b(valid_v, i);
    rmse = sqrt(mean((meas - pred).^2));
    ss_res = sum((meas - pred).^2);
    ss_tot = sum((meas - mean(meas)).^2);
    r2   = 1 - ss_res / ss_tot;
    fprintf('  %5s  ->  RMSE = %.6f   R² = %.6f\n', vel_labels{i}, rmse, r2);
end

fprintf('\n===== RMSE & R² — Posición (mundo)   =====\n');
for i = 1:4
    meas = pos(valid_p, i);
    pred = p_p_w(valid_p, i);
    rmse = sqrt(mean((meas - pred).^2));
    ss_res = sum((meas - pred).^2);
    ss_tot = sum((meas - mean(meas)).^2);
    r2   = 1 - ss_res / ss_tot;
    fprintf('  %5s  ->  RMSE = %.6f   R² = %.6f\n', pos_labels{i}, rmse, r2);
end
fprintf('\n');
% ──────────────────────────────────────────────────────────────────────────

ylabels_all = {
    '$\nu_{x}$ [m/s]',      '$\nu_{y}$ [m/s]', ...
    '$\nu_{z}$ [m/s]',      '$\nu_{\psi}$ [rad/s]', ...
    '$\eta_{x}$ [m]',       '$\eta_{y}$ [m]', ...
    '$\eta_{z}$ [m]',       '$\eta_{\psi}$ [rad]'
};

measured_all  = [v_m_b, pos];
predicted_all = [v_p_b, p_p_w];

make_combined_figure(t, u, measured_all, predicted_all, ...
    C_MEAS_V, C_MOD_V, C_MEAS_P, C_MOD_P, C_CTRL, ...
    PHASE_DUR, N_PHASES, PHASE_COLORS, ...
    ylabels_all, FIG_W, FIG_H);


function make_combined_figure(t, u, measured, predicted, ...
        C_MEAS_V, C_MOD_V, C_MEAS_P, C_MOD_P, C_CTRL, ...
        PHASE_DUR, N_PHASES, PHASE_COLORS, ...
        ylabels, FIG_W, FIG_H)

    n     = 8;
    valid = ~isnan(predicted(:,1));

    fig = figure('Units','inches','Position',[1 1 FIG_W FIG_H],'Color','w');

    left_m  = 0.13;
    width_m = 0.84;
    bot_m   = 0.07;
    top_m   = 0.985;
    gap     = 0.010;
    h_each  = (top_m - bot_m - (n-1)*gap) / n;

    ax = gobjects(n,1);

    for i = 1:n
        
        if i <= 4
            C_MEAS = C_MEAS_V;
            C_MOD  = C_MOD_V;
        else
            C_MEAS = C_MEAS_P;
            C_MOD  = C_MOD_P;
        end

        bot_i = bot_m + (n-i)*(h_each + gap);

        ax(i) = subplot('Position',[left_m, bot_i, width_m, h_each]);
        hold on; box off;
        set(ax(i), 'FontName','Times New Roman','FontSize',8, ...
            'TickDir','out','XGrid','on','YGrid','on', ...
            'GridLineStyle','--','GridAlpha',0.5, ...
            'GridColor',[0.69 0.69 0.69]);

        
        for p = 1:N_PHASES
            x0 = (p-1)*PHASE_DUR;
            x1 =  p   *PHASE_DUR;
            fill([x0 x1 x1 x0], [-1e6 -1e6 1e6 1e6], ...
                 PHASE_COLORS(p,:), 'EdgeColor','none', ...
                 'FaceAlpha',0.30, 'HandleVisibility','off');
        end

        
        plot(t, measured(:,i), '-', 'Color',C_MEAS, 'LineWidth',1.4);

        
        plot(t(valid), predicted(valid,i), '--', 'Color',C_MOD, 'LineWidth',1.4);

        
        alldata = [measured(:,i); predicted(valid,i)];
        ymin_d  = min(alldata, [], 'omitnan');
        ymax_d  = max(alldata, [], 'omitnan');
        yspan   = ymax_d - ymin_d;
        if yspan < 1e-6; yspan = 1; end
        ypad    = 0.12 * yspan;
        ylim([ymin_d - ypad, ymax_d + ypad]);

        u_col  = mod(i-1, 4) + 1;
        u_sig  = u(:, u_col);
        u_min  = min(u_sig);
        u_max  = max(u_sig);
        u_span = u_max - u_min;
        if u_span < 1e-9; u_span = 1; end
        y_lo   = ymin_d - ypad;
        y_hi   = ymax_d + ypad;
        u_plot = y_lo + (u_sig - u_min) ./ u_span .* (y_hi - y_lo);
        stairs(t, u_plot, '-.', 'Color',[C_CTRL 0.55], 'LineWidth',0.7, ...
               'HandleVisibility','off');

        xlim([t(1) t(end)]);

        ylabel(ylabels{i}, 'Interpreter','latex', ...
               'FontSize',16, 'FontName','Times New Roman');

        if i < n
            set(ax(i),'XTickLabel',[]);
        else
            xlabel('Time [s]','FontName','Times New Roman','FontSize',10);
        end
    end


    annotation(fig,'line', ...
        [left_m, left_m+width_m], ...
        [bot_m + 4*(h_each+gap) - gap/2, bot_m + 4*(h_each+gap) - gap/2], ...
        'Color',[0.5 0.5 0.5],'LineStyle','--','LineWidth',0.8);

    
    h1 = plot(ax(1), NaN, NaN, '-',  'Color',C_MEAS_V, 'LineWidth',1.4);
    h2 = plot(ax(1), NaN, NaN, '--', 'Color',C_MOD_V,  'LineWidth',1.4);
    h3 = plot(ax(1), NaN, NaN, '-',  'Color',C_MEAS_P, 'LineWidth',1.4);
    h4 = plot(ax(1), NaN, NaN, '--', 'Color',C_MOD_P,  'LineWidth',1.4);
    h5 = plot(ax(1), NaN, NaN, '-.', 'Color',[C_CTRL 0.6], 'LineWidth',1.0);

    leg = legend([h1 h2 h3 h4 h5], ...
        {'$\mathbf{\nu}_{\scriptscriptstyle\mathrm{Meas}}$', ...
         '$\mathbf{\nu}_{\scriptscriptstyle\mathrm{Model}}$', ...
         '$\mathbf{\eta}_{\scriptscriptstyle\mathrm{Meas}}$', ...
         '$\mathbf{\eta}_{\scriptscriptstyle\mathrm{Model}}$', ...
         '$U_{\scriptscriptstyle\mathrm{Input}}$'}, ...
        'Interpreter','latex','FontSize',18, ...
        'Orientation','horizontal','Box','off','Units','normalized');
    leg.Position(1) = 0.5 - leg.Position(3)/2;
    leg.Position(2) = 0.001;

    % Export PDF
    fig.PaperUnits    = 'inches';
    fig.PaperSize     = [FIG_W FIG_H];
    fig.PaperPosition = [0 0 FIG_W FIG_H];

    exportgraphics(fig, 'sysid_combined.pdf', ...
        'ContentType','vector','BackgroundColor','white');
end


function [v_p_b, p_p_w, v_m_b] = simulate_stepbystep(t, u, pos, vel_world, Ku, Kv)
    N     = length(t);
    v_p_b = nan(N,4);
    p_p_w = nan(N,4);
    v_m_b = nan(N,4);
    a_lim = 0.2;

    p_p_w(1,:) = pos(1,:);
    R0         = rot_mat(pos(1,4));
    v_p_b(1,:) = (R0' * vel_world(1,:)')';

    for k = 1:N-1
        dt         = t(k+1) - t(k);
        R_bw       = rot_mat(pos(k,4));
        xdot_b     = (R_bw' * vel_world(k,:)')';
        v_m_b(k,:) = xdot_b;
        xddot      = (Ku * u(k,:)')' - (Kv * xdot_b')';
        xddot(1:3) = max(min(xddot(1:3), a_lim), -a_lim);
        v_p_b(k+1,:) = xdot_b    + xddot         * dt;
        p_p_w(k+1,:) = pos(k,:)  + vel_world(k,:) * dt;
    end
    v_m_b(N,:) = (rot_mat(pos(N,4))' * vel_world(N,:)')';
end


function R = rot_mat(psi)
    c = cos(psi); s = sin(psi);
    R = [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
end