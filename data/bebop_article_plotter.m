%% =========================================================
%  bebop_article_plotter.m
%  UAV Flight Telemetry – Publication-Quality Plotter
%  ---------------------------------------------------------
%  Generates IEEE/Elsevier-ready figures from a Bebop CSV.
%
%  Usage:
%    bebop_article_plotter          % opens GUI file picker
%    bebop_article_plotter('file.csv')
%
%  Exports (optional – see EXPORT section at the bottom):
%    PDF   (vector, print-ready)
%    PNG   (300 dpi)
%    EPS   (for LaTeX \includegraphics)
%% =========================================================

function bebop_article_plotter(csv_path)

    % ── 0. RESOLVE CSV PATH ──────────────────────────────────────────
    if nargin < 1 || isempty(csv_path)
        [fname, fpath] = uigetfile('*.csv', 'Select Bebop CSV file');
        if isequal(fname, 0)
            disp('[!] No file selected. Aborting.');
            return
        end
        csv_path = fullfile(fpath, fname);
    end
    fprintf('[→] Loading: %s\n', csv_path);

    % ── 1. LOAD & PREPROCESS ─────────────────────────────────────────
    df = readtable(csv_path);
    df.t_rel = df.time - df.time(1);

    % Yaw correction: wrap difference into [−π, π]
    if ismember('yaw', df.Properties.VariableNames) && ...
       ismember('yawd', df.Properties.VariableNames)
        d = df.yaw - df.yawd;
        df.yaw_corr = df.yawd + atan2(sin(d), cos(d));
    elseif ismember('yaw', df.Properties.VariableNames)
        df.yaw_corr = df.yaw;
    else
        df.yaw_corr = zeros(height(df), 1);
    end

    t      = df.t_rel;
    t_lim  = [min(t), max(t)];
    stem_  = get_filestem(csv_path);

    % ── 2. GLOBAL STYLE SETTINGS ─────────────────────────────────────
    set_publication_style();

    % ── 3. FULL TELEMETRY FIGURE  (4 × 2) ───────────────────────────
    fig = figure('Name', 'UAV Telemetry', 'NumberTitle', 'off', ...
                 'Units', 'inches', 'Position', [1 1 7.16 9.0]);

    % Color palette (Okabe–Ito, colorblind-safe)
    C_meas = [0.000, 0.447, 0.698];   % blue
    C_ref  = [0.835, 0.369, 0.000];   % vermillion
    C_cmd  = [0.000, 0.620, 0.451];   % teal-green

    % Line styles
    LS_meas = '-';   LW_meas = 1.3;
    LS_ref  = '--';  LW_ref  = 1.1;
    LS_cmd  = ':';   LW_cmd  = 1.0;

    % ── Column 0 : Position / Attitude ───────────────────────────────
    pos_data = { ...
        'x',        'xd',   '$x$ [m]',          '$x$ position'; ...
        'y',        'yd',   '$y$ [m]',          '$y$ position'; ...
        'z',        'zd',   '$z$ [m]',          '$z$ position'; ...
        'yaw_corr', 'yawd', '$\psi$ [rad]',     'Yaw $\psi$'; ...
    };

    for i = 1:4
        ax = subplot(4, 2, 2*i - 1);
        hold(ax, 'on');

        meas_col = pos_data{i, 1};
        ref_col  = pos_data{i, 2};
        ylabel_  = pos_data{i, 3};
        title_   = pos_data{i, 4};

        if ismember(meas_col, df.Properties.VariableNames)
            plot(ax, t, df.(meas_col), LS_meas, ...
                 'Color', C_meas, 'LineWidth', LW_meas);
        end
        if ismember(ref_col, df.Properties.VariableNames)
            plot(ax, t, df.(ref_col), LS_ref, ...
                 'Color', C_ref, 'LineWidth', LW_ref);
        end

        style_axes(ax, t_lim, [], ylabel_, title_);
        if i == 4
            xlabel(ax, '$t$ [s]', 'Interpreter', 'latex');
        end
    end

    % ── Column 1 : Velocities / Rates ────────────────────────────────
    vel_data = { ...
        'cmd_linx', 'linx_b', 'vxd_b',   '$v_x$ [m/s]',            'Velocity $v_x$'; ...
        'cmd_liny', 'liny_b', 'vyd_b',   '$v_y$ [m/s]',            'Velocity $v_y$'; ...
        'cmd_linz', 'linz_b', 'vzd_b',   '$v_z$ [m/s]',            'Velocity $v_z$'; ...
        'cmd_angz', 'yaw_rate','wyawd',  '$\dot{\psi}$ [rad/s]',   'Yaw rate $\dot{\psi}$'; ...
    };
    VEL_LIM = 1.8;

    for i = 1:4
        ax = subplot(4, 2, 2*i);
        hold(ax, 'on');

        cmd_col  = vel_data{i, 1};
        meas_col = vel_data{i, 2};
        ref_col  = vel_data{i, 3};
        ylabel_  = vel_data{i, 4};
        title_   = vel_data{i, 5};

        if ismember(cmd_col, df.Properties.VariableNames)
            plot(ax, t, df.(cmd_col), LS_cmd, ...
                 'Color', C_cmd, 'LineWidth', LW_cmd);
        end
        if ismember(meas_col, df.Properties.VariableNames)
            plot(ax, t, df.(meas_col), LS_meas, ...
                 'Color', C_meas, 'LineWidth', LW_meas);
        end
        if ismember(ref_col, df.Properties.VariableNames)
            plot(ax, t, df.(ref_col), LS_ref, ...
                 'Color', C_ref, 'LineWidth', LW_ref);
        end

        style_axes(ax, t_lim, [-VEL_LIM VEL_LIM], ylabel_, title_);
        if i == 4
            xlabel(ax, '$t$ [s]', 'Interpreter', 'latex');
        end
    end

    % ── Shared legend ─────────────────────────────────────────────────
    h_meas = plot(NaN, NaN, LS_meas, 'Color', C_meas, 'LineWidth', LW_meas);
    h_ref  = plot(NaN, NaN, LS_ref,  'Color', C_ref,  'LineWidth', LW_ref);
    h_cmd  = plot(NaN, NaN, LS_cmd,  'Color', C_cmd,  'LineWidth', LW_cmd);

    leg = legend([h_meas h_ref h_cmd], ...
        {'Measured', 'Reference', 'Command'}, ...
        'Orientation', 'horizontal', ...
        'Location', 'southoutside', ...
        'Interpreter', 'latex', ...
        'FontSize', 8, ...
        'Box', 'on');
    leg.Position(2) = 0.005;

    % ── Suptitle ──────────────────────────────────────────────────────
    sgtitle(fig, sprintf('UAV Flight Telemetry — %s', stem_), ...
            'Interpreter', 'none', 'FontWeight', 'bold', 'FontSize', 10, ...
            'FontName', 'Times New Roman');

    % ── EXPORT ────────────────────────────────────────────────────────
    % Uncomment the lines you need:
    %
    % export_figure(fig, stem_, 'pdf')   % vector PDF  (recommended)
    % export_figure(fig, stem_, 'png')   % 300 dpi PNG
    % export_figure(fig, stem_, 'epsc')  % colour EPS for LaTeX
    %
    % Or export individual subplot (see build_single_axis below).

    % ── Show ──────────────────────────────────────────────────────────
    drawnow;
    fprintf('[✓] Done.\n');

end  % main function


%% =========================================================
%  build_single_axis
%  ---------------------------------------------------------
%  Creates a tight single-panel figure for a specific variable.
%  Ideal for half-column or quarter-column journal floats.
%
%  Example:
%    df = preprocess_csv('flight.csv');
%    fig = build_single_axis(df, 'z', 'zd', [], ...
%          '$z$ [m]', 'Position $z$', [3.5 2.2]);
%% =========================================================
function fig = build_single_axis(df, meas_col, ref_col, cmd_col, ...
                                  ylabel_str, title_str, figsize)

    set_publication_style();

    C_meas = [0.000, 0.447, 0.698];
    C_ref  = [0.835, 0.369, 0.000];
    C_cmd  = [0.000, 0.620, 0.451];

    if nargin < 7 || isempty(figsize), figsize = [3.5 2.2]; end

    fig = figure('Units', 'inches', 'Position', [1 1 figsize(1) figsize(2)]);
    ax  = axes(fig);
    hold(ax, 'on');

    t     = df.t_rel;
    t_lim = [min(t), max(t)];
    h     = gobjects(0);
    lbl   = {};

    if ~isempty(meas_col) && ismember(meas_col, df.Properties.VariableNames)
        h(end+1) = plot(ax, t, df.(meas_col), '-',  'Color', C_meas, 'LineWidth', 1.3);
        lbl{end+1} = 'Measured';
    end
    if ~isempty(ref_col) && ismember(ref_col, df.Properties.VariableNames)
        h(end+1) = plot(ax, t, df.(ref_col),  '--', 'Color', C_ref,  'LineWidth', 1.1);
        lbl{end+1} = 'Reference';
    end
    if ~isempty(cmd_col) && ismember(cmd_col, df.Properties.VariableNames)
        h(end+1) = plot(ax, t, df.(cmd_col),  ':',  'Color', C_cmd,  'LineWidth', 1.0);
        lbl{end+1} = 'Command';
    end

    style_axes(ax, t_lim, [], ylabel_str, title_str);
    xlabel(ax, '$t$ [s]', 'Interpreter', 'latex');
    if ~isempty(h)
        legend(ax, h, lbl, 'Interpreter', 'latex', 'FontSize', 8, 'Location', 'best');
    end

    set(fig, 'PaperUnits', 'inches', 'PaperSize', figsize, ...
             'PaperPosition', [0 0 figsize(1) figsize(2)]);
    drawnow;
end


%% =========================================================
%  HELPERS
%% =========================================================

function set_publication_style()
%SET_PUBLICATION_STYLE  Apply journal-ready global defaults.
    set(groot, ...
        'defaultAxesFontName',         'Times New Roman', ...
        'defaultAxesFontSize',         9, ...
        'defaultAxesTitleFontWeight',  'normal', ...
        'defaultAxesTickLabelInterpreter', 'latex', ...
        'defaultAxesXMinorTick',       'on', ...
        'defaultAxesYMinorTick',       'on', ...
        'defaultAxesLineWidth',        0.7, ...
        'defaultAxesBox',              'off', ...
        'defaultAxesXGrid',            'on', ...
        'defaultAxesYGrid',            'on', ...
        'defaultAxesGridAlpha',        0.4, ...
        'defaultAxesGridLineStyle',    '--', ...
        'defaultAxesGridColor',        [0.7 0.7 0.7], ...
        'defaultLineLineWidth',        1.2, ...
        'defaultTextFontName',         'Times New Roman', ...
        'defaultTextInterpreter',      'latex', ...
        'defaultLegendInterpreter',    'latex', ...
        'defaultColorbarTickLabelInterpreter', 'latex' ...
    );
end

% ─────────────────────────────────────────────────────────
function style_axes(ax, x_lim, y_lim, ylabel_str, title_str)
%STYLE_AXES  Apply consistent cosmetics to a single axes object.
    ax.XLim = x_lim;
    if ~isempty(y_lim)
        ax.YLim = y_lim;
    end
    ylabel(ax, ylabel_str, 'Interpreter', 'latex', 'FontSize', 9);
    title(ax, title_str,   'Interpreter', 'latex', 'FontSize', 9, ...
          'HorizontalAlignment', 'left', 'Units', 'normalized', ...
          'Position', [0, 1.02, 0]);
    ax.TickDir         = 'out';
    ax.FontName        = 'Times New Roman';
    ax.FontSize        = 8;
    ax.XColor          = [0.2 0.2 0.2];
    ax.YColor          = [0.2 0.2 0.2];
    ax.GridColor       = [0.7 0.7 0.7];
    ax.GridAlpha       = 0.45;
    ax.MinorGridAlpha  = 0.25;
    ax.Layer           = 'top';        % grid behind data
end

% ─────────────────────────────────────────────────────────
function export_figure(fig, stem_, format_)
%EXPORT_FIGURE  Save a figure to PDF, PNG, or EPS.
    out_dir = fullfile(pwd, 'article_figures');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    switch lower(format_)
        case 'pdf'
            fname = fullfile(out_dir, [stem_ '.pdf']);
            exportgraphics(fig, fname, ...
                'ContentType', 'vector', ...
                'BackgroundColor', 'white');
        case 'png'
            fname = fullfile(out_dir, [stem_ '.png']);
            exportgraphics(fig, fname, ...
                'Resolution', 300, ...
                'BackgroundColor', 'white');
        case {'eps', 'epsc'}
            fname = fullfile(out_dir, [stem_ '.eps']);
            print(fig, fname, '-depsc', '-r300');
        otherwise
            warning('Unknown format: %s', format_);
            return
    end
    fprintf('[✓] Saved: %s\n', fname);
end

% ─────────────────────────────────────────────────────────
function s = get_filestem(path_)
%GET_FILESTEM  Return filename without extension.
    [~, s, ~] = fileparts(path_);
end
