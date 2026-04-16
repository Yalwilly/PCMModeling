% filepath: c:\Users\yalwilly\OneDrive - Intel Corporation\Documents\yazn\work\Next Gen DC2DC\PCM Modeling\PCMModeling\PCMModeling\NGSTability_V6.m
%% ====== NGSTability_V6.m ======
% Optimization-only PID tuning for PCM buck converter
% Supports:
%   - Nelder-Mead multi-start
%   - PSO
% Evaluates quantized RTL controller with K1/K2/K3 limits

clear; clc; close all;

%% ==================== USER SETTINGS ====================

% Controller model
CONTROLLER_DOMAIN = 'laplace';   % 'laplace' or 'z'

% RTL scaling / coefficient limits
SHIFT_BITS = 6;
G_digital  = 2^(-SHIFT_BITS);

K1_MAG_BITS = 19;   % max magnitude = 2^19-1
K2_MAG_BITS = 19;   % max magnitude = 2^19-1
K3_MAG_BITS = 10;   % 10->1023, 11->2047, 12->4095

K1_MAX = 2^K1_MAG_BITS - 1;
K2_MAX = 2^K2_MAG_BITS - 1;
K3_MAX = 2^K3_MAG_BITS - 1;

% Feedback
beta = 0.5;
H = beta;

% Converter parameters
%VOUT = 1.2;
VOUT = 0.8;
VIN  = 3.3;
%Lind = 1e-6;
Lind = 0.47e-6;

fs   = 6e6;
Ts   = 1/fs;
fc   = fs/10;
Tf   = 1/(10*fc);

fdig = 256e6;
Tctrl = 1/fdig;

% Current sensing
%GI=(1/3200)*(18/32)*(1/4);
GI= (1/640)*(10/64)*(1/4);
Rsense = 4e3;
%Rsense = 6e3;
Ri     = GI * Rsense;

% Duty / slope compensation
D     = VOUT/VIN;
D_tag = 1 - D;

se   = 0.7e6;
sn   = (VIN - VOUT)/Lind;
sf   = VOUT/Lind;
mc   = 1 + se/sn;

% Output filter
C    = 22e-6;
Resr = 3e-3;

% Digital blocks
N_ADC    = 4;
VADC_MIN = 0.36;
VADC_MAX = 0.44;
V_FS_ADC = VADC_MAX - VADC_MIN;

N_DAC    = 12;
V_FS_DAC = 1.2;

USE_SMALL_SIGNAL_ADC_DAC = false;
if USE_SMALL_SIGNAL_ADC_DAC
    G_ADC = (2^N_ADC) / V_FS_ADC;
    G_DAC = V_FS_DAC / (2^N_DAC);
else
    G_ADC = 1;
    G_DAC = 1;
end

wp_dac  = 2*pi*500e3;
Td      = 8e-9;
Td_ADC  = 5e-9;
Tblank  = 15e-9;

DAC_VDD       = 1.8;        % V
DAC_NMOS_VTH  = 0.2;        % V
DAC_OUT_MIN   = 0.0;        % V
DAC_OUT_MAX   = DAC_VDD - DAC_NMOS_VTH;  % NMOS-limited max

PID_OUT_BITS       = 12;
PID_OUT_OFFSET     = 2^(PID_OUT_BITS-1);            % 2048 midscale
PID_PRECLAMP_MIN   = -2048 * 2^SHIFT_BITS;          % RTL MIN_VALUE
PID_PRECLAMP_MAX   =  2047 * 2^SHIFT_BITS;          % RTL MAX_VALUE

%% ==================== OPTIMIZATION SETTINGS ====================

RUN_SIMPLE_PID_OPT = true;
SIMPLE_OPT_VERBOSE = true;

OPT_I_LOAD_VEC = [0.001 0.10 0.30 0.50 0.70 0.90 1.10 2 3];

OPT_MIN_PM_DEG = 35;
OPT_MIN_GM_DB  = 6;

OPT_W_PM_MIN   = 2;
OPT_W_GM_MIN   = 1;
OPT_W_PM_AVG   = 0.15;
OPT_W_GM_AVG   = 0.10;
OPT_W_FC       = 1;
OPT_W_SPREAD   = 0.1;

OPT_PM_CAP     = 85;
OPT_GM_CAP_DB  = 40;
OPT_MAX_ITER   = 300;

OPT_KP_MIN = 0.015625;
OPT_KP_MAX = 10e3;
OPT_KI_MIN = 4e6;
OPT_KI_MAX = 5e8;
OPT_KD_MIN = 1 * Tctrl * G_digital;
OPT_KD_MAX = K3_MAX * Tctrl * G_digital;

%% Load-step metric in optimizer
OPT_ENABLE_LOADSTEP_METRIC = true;

OPT_LOADSTEP_DI_A           = 1;      % load step amplitude [A]
OPT_LOADSTEP_TFINAL         = 40e-6;    % simulation window [s]
OPT_LOADSTEP_NPTS           = 400;      % keep modest for optimizer speed
OPT_LOADSTEP_ERR_BAND_V     = 2e-3;     % settle band [V]

OPT_W_LOADSTEP_SLEW         = 12;     % reward fast recovery slew
OPT_W_LOADSTEP_SETTLE       = 12;      % penalize slow settling
OPT_W_LOADSTEP_UNDERSHOOT   = 12;      % penalize deep undershoot

OPT_LOADSTEP_SLEW_REF_VUS   = 0.002;    % reference recovery slew [V/us]
OPT_LOADSTEP_SETTLE_REF_US  = 5;      % reference settling time [us]
OPT_LOADSTEP_UNDERSHOOT_REF_MV = 20;  % reference undershoot [mV]

% Nonlinear predictor settings (V5-style cycle-by-cycle)
OPT_USE_NONLINEAR_PREDICTOR = true;    % true = V5 predictor, false = linear step()
OPT_LOADSTEP_VOUT_INIT   = VOUT;      % V, initial Vout for predictor
OPT_LOADSTEP_DELAY_S     = 2e-6;      % s, pre-step settle time (keep short for speed)
OPT_LOADSTEP_RISE_S      = 100e-9;    % s, rising edge time
OPT_LOADSTEP_WIDTH_S     = inf;       % s, inf = one-way step
OPT_LOADSTEP_FALL_S      = 100e-9;    % s, falling edge time
USE_QUANTIZED_ADC_DAC     = true;      % exact ADC/DAC quantization in predictor
USE_PRECLAMP_STATE        = true;      % true matches RTL pre-clamp state behavior


%% ==================== 3D STABILITY MAP SETTINGS ====================
RUN_STABILITY_MAP    = true;        % enable/disable 3D scan
STAB_MAP_N_KP        = 25;          % grid points along Kp axis
STAB_MAP_N_KI        = 25;          % grid points along Ki axis
STAB_MAP_N_KD        = 15;          % grid points along Kd axis
STAB_MAP_KP_RANGE    = [0.1, 200];      % [min max] Kp_target
STAB_MAP_KI_RANGE    = [4e6, 1e8];      % [min max] Ki_target
STAB_MAP_KD_RANGE    = [OPT_KD_MIN, OPT_KD_MAX]; % [min max] Kd_target
STAB_MAP_LOG_SCALE   = true;        % use log-spaced grid (recommended)
STAB_MAP_ILOAD_VEC   = OPT_I_LOAD_VEC; % load currents for stability check
STAB_MAP_MIN_PM_DEG  = OPT_MIN_PM_DEG;  % PM threshold for "stable"
STAB_MAP_MIN_GM_DB   = OPT_MIN_GM_DB;   % GM threshold for "stable"
STAB_MAP_MARKER_SIZE = 20;          % scatter marker size

%% ==================== OPTIMIZER SELECTION ====================

OPT_METHOD = 'pso';   % 'nelder-mead' or 'pso'
OPT_RANDOM_SEED = 1;

% Nelder-Mead multi-start
OPT_NM_NSTARTS = 24;

% PSO
OPT_PSO_SWARM_SIZE = 40;
OPT_PSO_MAX_ITER   = 80;
OPT_PSO_USE_HYBRID = true;



%% ==================== PREPARE FIXED BLOCKS ====================

s = tf('s');

[numBlank, denBlank] = pade(Tblank, 2);
Hblank = tf(numBlank, denBlank);

[numD, denD] = pade(Td, 2);
Delay = tf(numD, denD);

[numADC, denADC] = pade(Td_ADC, 2);
Hadc = tf(numADC, denADC);

Hdac = 1 / (1 + s/wp_dac);

%% ==================== BUILD OPT CFG ====================

opt_cfg = struct();

opt_cfg.Tctrl = Tctrl;
opt_cfg.Ts = Ts;
opt_cfg.Tf = Tf;
opt_cfg.fc = fc;

opt_cfg.G_digital = G_digital;
opt_cfg.G_ADC = G_ADC;
opt_cfg.G_DAC = G_DAC;

opt_cfg.H = H;
opt_cfg.Hadc = Hadc;
opt_cfg.Hdac = Hdac;
opt_cfg.Hblank = Hblank;
opt_cfg.Delay = Delay;
opt_cfg.controller_domain = CONTROLLER_DOMAIN;

opt_cfg.VOUT = VOUT;
opt_cfg.Lind = Lind;
opt_cfg.C = C;
opt_cfg.Resr = Resr;
opt_cfg.Ri = Ri;
opt_cfg.mc = mc;
opt_cfg.D_tag = D_tag;

opt_cfg.Iload_vec = OPT_I_LOAD_VEC;

opt_cfg.OPT_MIN_PM_DEG = OPT_MIN_PM_DEG;
opt_cfg.OPT_MIN_GM_DB = OPT_MIN_GM_DB;

opt_cfg.OPT_W_PM_MIN = OPT_W_PM_MIN;
opt_cfg.OPT_W_GM_MIN = OPT_W_GM_MIN;
opt_cfg.OPT_W_PM_AVG = OPT_W_PM_AVG;
opt_cfg.OPT_W_GM_AVG = OPT_W_GM_AVG;
opt_cfg.OPT_W_FC = OPT_W_FC;
opt_cfg.OPT_W_SPREAD = OPT_W_SPREAD;

opt_cfg.OPT_PM_CAP = OPT_PM_CAP;
opt_cfg.OPT_GM_CAP_DB = OPT_GM_CAP_DB;
opt_cfg.OPT_MAX_ITER = OPT_MAX_ITER;

opt_cfg.OPT_KP_MIN = OPT_KP_MIN;
opt_cfg.OPT_KP_MAX = OPT_KP_MAX;
opt_cfg.OPT_KI_MIN = OPT_KI_MIN;
opt_cfg.OPT_KI_MAX = OPT_KI_MAX;
opt_cfg.OPT_KD_MIN = OPT_KD_MIN;
opt_cfg.OPT_KD_MAX = OPT_KD_MAX;

opt_cfg.K1_MAX = K1_MAX;
opt_cfg.K2_MAX = K2_MAX;
opt_cfg.K3_MAX = K3_MAX;

opt_cfg.SIMPLE_OPT_VERBOSE = SIMPLE_OPT_VERBOSE;

opt_cfg.optimizer = struct();
opt_cfg.optimizer.method = OPT_METHOD;
opt_cfg.optimizer.random_seed = OPT_RANDOM_SEED;
opt_cfg.optimizer.nm_nstarts = OPT_NM_NSTARTS;
opt_cfg.optimizer.pso_swarm_size = OPT_PSO_SWARM_SIZE;
opt_cfg.optimizer.pso_max_iter = OPT_PSO_MAX_ITER;
opt_cfg.optimizer.pso_use_hybrid = OPT_PSO_USE_HYBRID;

opt_cfg.OPT_ENABLE_LOADSTEP_METRIC = OPT_ENABLE_LOADSTEP_METRIC;

opt_cfg.OPT_LOADSTEP_DI_A = OPT_LOADSTEP_DI_A;
opt_cfg.OPT_LOADSTEP_TFINAL = OPT_LOADSTEP_TFINAL;
opt_cfg.OPT_LOADSTEP_NPTS = OPT_LOADSTEP_NPTS;
opt_cfg.OPT_LOADSTEP_ERR_BAND_V = OPT_LOADSTEP_ERR_BAND_V;

opt_cfg.OPT_W_LOADSTEP_SLEW = OPT_W_LOADSTEP_SLEW;
opt_cfg.OPT_W_LOADSTEP_SETTLE = OPT_W_LOADSTEP_SETTLE;
opt_cfg.OPT_W_LOADSTEP_UNDERSHOOT = OPT_W_LOADSTEP_UNDERSHOOT;

opt_cfg.OPT_LOADSTEP_SLEW_REF_VUS = OPT_LOADSTEP_SLEW_REF_VUS;
opt_cfg.OPT_LOADSTEP_SETTLE_REF_US = OPT_LOADSTEP_SETTLE_REF_US;
opt_cfg.OPT_LOADSTEP_UNDERSHOOT_REF_MV = OPT_LOADSTEP_UNDERSHOOT_REF_MV;

% Nonlinear predictor params
opt_cfg.OPT_USE_NONLINEAR_PREDICTOR = OPT_USE_NONLINEAR_PREDICTOR;
opt_cfg.OPT_LOADSTEP_VOUT_INIT = OPT_LOADSTEP_VOUT_INIT;
opt_cfg.OPT_LOADSTEP_DELAY_S = OPT_LOADSTEP_DELAY_S;
opt_cfg.OPT_LOADSTEP_RISE_S = OPT_LOADSTEP_RISE_S;
opt_cfg.OPT_LOADSTEP_WIDTH_S = OPT_LOADSTEP_WIDTH_S;
opt_cfg.OPT_LOADSTEP_FALL_S = OPT_LOADSTEP_FALL_S;

opt_cfg.USE_QUANTIZED_ADC_DAC = USE_QUANTIZED_ADC_DAC;
opt_cfg.USE_PRECLAMP_STATE = USE_PRECLAMP_STATE;

opt_cfg.SHIFT_BITS = SHIFT_BITS;
opt_cfg.PID_OUT_BITS = PID_OUT_BITS;
opt_cfg.PID_OUT_OFFSET = PID_OUT_OFFSET;
opt_cfg.PID_PRECLAMP_MIN = PID_PRECLAMP_MIN;
opt_cfg.PID_PRECLAMP_MAX = PID_PRECLAMP_MAX;

opt_cfg.N_ADC = N_ADC;
opt_cfg.VADC_MIN = VADC_MIN;
opt_cfg.VADC_MAX = VADC_MAX;

opt_cfg.N_DAC = N_DAC;
opt_cfg.V_FS_DAC = V_FS_DAC;
opt_cfg.wp_dac = wp_dac;
opt_cfg.DAC_OUT_MIN = DAC_OUT_MIN;
opt_cfg.DAC_OUT_MAX = DAC_OUT_MAX;

opt_cfg.beta = beta;
opt_cfg.Vref = VOUT;  % feedback reference

% Stability map params
opt_cfg.RUN_STABILITY_MAP   = RUN_STABILITY_MAP;
opt_cfg.STAB_MAP_N_KP       = STAB_MAP_N_KP;
opt_cfg.STAB_MAP_N_KI       = STAB_MAP_N_KI;
opt_cfg.STAB_MAP_N_KD       = STAB_MAP_N_KD;
opt_cfg.STAB_MAP_KP_RANGE   = STAB_MAP_KP_RANGE;
opt_cfg.STAB_MAP_KI_RANGE   = STAB_MAP_KI_RANGE;
opt_cfg.STAB_MAP_KD_RANGE   = STAB_MAP_KD_RANGE;
opt_cfg.STAB_MAP_LOG_SCALE  = STAB_MAP_LOG_SCALE;
opt_cfg.STAB_MAP_ILOAD_VEC  = STAB_MAP_ILOAD_VEC;
opt_cfg.STAB_MAP_MIN_PM_DEG = STAB_MAP_MIN_PM_DEG;
opt_cfg.STAB_MAP_MIN_GM_DB  = STAB_MAP_MIN_GM_DB;
opt_cfg.STAB_MAP_MARKER_SIZE = STAB_MAP_MARKER_SIZE;

%% ==================== RUN OPTIMIZATION ==

if RUN_SIMPLE_PID_OPT
    opt_result = simple_optimize_pid(opt_cfg);

    fprintf('\n===== OPTIMIZATION RESULT =====\n');
    fprintf('Optimizer method:\n');
    fprintf('  %s\n', opt_result.optimizer_method);

    fprintf('\nSuggested target gains:\n');
    fprintf('  Kp_target = %.6g\n', opt_result.Kp_target);
    fprintf('  Ki_target = %.6g\n', opt_result.Ki_target);
    fprintf('  Kd_target = %.6g\n', opt_result.Kd_target);

    fprintf('\nImplemented effective gains after quantization:\n');
    fprintf('  Kp_ctrl_eff = %.6g\n', opt_result.Kp_ctrl_eff);
    fprintf('  Ki_ctrl_eff = %.6g\n', opt_result.Ki_ctrl_eff);
    fprintf('  Kd_ctrl_eff = %.6g\n', opt_result.Kd_ctrl_eff);

    fprintf('\nLab positions:\n');
    fprintf('  P = %d\n', opt_result.P_cmd);
    fprintf('  I = %d\n', opt_result.I_cmd);
    fprintf('  D = %d\n', opt_result.D_cmd);

    fprintf('\nRTL coefficients:\n');
    fprintf('  K1 = %d  (max %d)\n', opt_result.K1, K1_MAX);
    fprintf('  K2 = %d  (max %d)\n', opt_result.K2, K2_MAX);
    fprintf('  K3 = %d  (max %d)\n', opt_result.K3, K3_MAX);

    fprintf('\nWorst-case margins across load sweep:\n');
    fprintf('  GM = %.2f dB\n', opt_result.Gm_dB_min);
    fprintf('  PM = %.2f deg\n', opt_result.Pm_deg_min);

    fprintf('\nAverage margins across load sweep:\n');
    fprintf('  GM = %.2f dB\n', opt_result.Gm_dB_avg);
    fprintf('  PM = %.2f deg\n', opt_result.Pm_deg_avg);

    fprintf('\nWorst-case load-step metrics:\n');
    fprintf('  Recovery slew = %.4f V/us\n', opt_result.LoadStepRecoverySlew_Vus_worst);
    fprintf('  Settling time = %.3f us\n', opt_result.LoadStepSettle_us_worst);
    fprintf('  Undershoot    = %.3f mV\n', opt_result.LoadStepUndershoot_mV_worst);

    fprintf('\nPer-load summary:\n');
    for ii = 1:numel(opt_result.Iload_vec)
        fprintf('  Iload = %.3f A | GM = %6.2f dB | PM = %6.2f deg | Fc = %8.2f kHz | LS=%d | Slew = %7.4f V/us | Ts = %7.3f us | US = %7.3f mV\n', ...
            opt_result.Iload_vec(ii), ...
            opt_result.Gm_dB_vec(ii), ...
            opt_result.Pm_deg_vec(ii), ...
            opt_result.Fc_kHz_vec(ii), ...
            opt_result.LoadStepEvalRan_vec(ii), ...
            opt_result.LoadStepRecoverySlew_Vus_vec(ii), ...
            opt_result.LoadStepSettle_us_vec(ii), ...
            opt_result.LoadStepUndershoot_mV_vec(ii));
    end

    fprintf('\nProgramming line:\n');
    fprintf('  ChipTC.FullChip.Power.SetPIDCofficients(1, %d, %d, %d);\n', ...
        opt_result.P_cmd, opt_result.I_cmd, opt_result.D_cmd);

    %% ---- Convergence Plot ----
    if isfield(opt_result, 'convergence') && opt_result.convergence.n_evals > 0
        conv = opt_result.convergence;
        figure;

        has_mark = ~isempty(conv.mark_eval) && conv.mark_eval > 0;

        subplot(2,1,1);
        plot(conv.elapsed_s, conv.best_cost, 'b-', 'LineWidth', 1.5);
        hold on;
        plot(conv.elapsed_s, conv.cost, '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 3);
        plot(conv.elapsed_s, conv.best_cost, 'b-', 'LineWidth', 1.5);
        if has_mark
            xline(conv.mark_t, 'k--', 'PSO \rightarrow fminsearch', ...
                'LineWidth', 1.2, 'LabelOrientation', 'horizontal', ...
                'LabelHorizontalAlignment', 'left', 'FontSize', 8);
        end
        hold off;
        grid on;
        xlabel('Elapsed Time (s)');
        ylabel('Cost');
        title(sprintf('Optimizer Convergence (%s)  —  %d evals in %.1f s', ...
            opt_result.optimizer_method, conv.n_evals, conv.total_time));
        legend('Best-so-far', 'Individual evals', 'Location', 'best');

        subplot(2,1,2);
        eval_idx = 1:conv.n_evals;
        plot(eval_idx, conv.best_cost, 'r-', 'LineWidth', 1.5);
        hold on;
        plot(eval_idx, conv.cost, '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 3);
        plot(eval_idx, conv.best_cost, 'r-', 'LineWidth', 1.5);
        if has_mark
            xline(conv.mark_eval, 'k--', 'PSO \rightarrow fminsearch', ...
                'LineWidth', 1.2, 'LabelOrientation', 'horizontal', ...
                'LabelHorizontalAlignment', 'left', 'FontSize', 8);
        end
        hold off;
        grid on;
        xlabel('Function Evaluation #');
        ylabel('Cost');
        title('Cost vs Evaluation Count');
        legend('Best-so-far', 'Individual evals', 'Location', 'best');

        fprintf('\nConvergence summary:\n');
        fprintf('  Total evaluations: %d\n', conv.n_evals);
        fprintf('  Total wall time:   %.1f s\n', conv.total_time);
        fprintf('  Final best cost:   %.4f\n', conv.best_cost(end));
    end
end

%% ==================== 3D STABILITY MAP ====================
if RUN_STABILITY_MAP
    fprintf('\n===== 3D STABILITY MAP SCAN =====\n');

    if STAB_MAP_LOG_SCALE
        kp_vec = logspace(log10(STAB_MAP_KP_RANGE(1)), log10(STAB_MAP_KP_RANGE(2)), STAB_MAP_N_KP);
        ki_vec = logspace(log10(STAB_MAP_KI_RANGE(1)), log10(STAB_MAP_KI_RANGE(2)), STAB_MAP_N_KI);
        kd_vec = logspace(log10(STAB_MAP_KD_RANGE(1)), log10(STAB_MAP_KD_RANGE(2)), STAB_MAP_N_KD);
    else
        kp_vec = linspace(STAB_MAP_KP_RANGE(1), STAB_MAP_KP_RANGE(2), STAB_MAP_N_KP);
        ki_vec = linspace(STAB_MAP_KI_RANGE(1), STAB_MAP_KI_RANGE(2), STAB_MAP_N_KI);
        kd_vec = linspace(STAB_MAP_KD_RANGE(1), STAB_MAP_KD_RANGE(2), STAB_MAP_N_KD);
    end

    n_total = STAB_MAP_N_KP * STAB_MAP_N_KI * STAB_MAP_N_KD;
    fprintf('  Grid: %d x %d x %d = %d points\n', STAB_MAP_N_KP, STAB_MAP_N_KI, STAB_MAP_N_KD, n_total);
    fprintf('  Scanning...\n');

    map_kp = zeros(n_total, 1);
    map_ki = zeros(n_total, 1);
    map_kd = zeros(n_total, 1);
    map_stable = false(n_total, 1);
    map_pm_min = zeros(n_total, 1);
    map_gm_min = zeros(n_total, 1);

    idx = 0;
    t_scan_start = tic;
    for ikp = 1:STAB_MAP_N_KP
        for iki = 1:STAB_MAP_N_KI
            for ikd = 1:STAB_MAP_N_KD
                idx = idx + 1;
                map_kp(idx) = kp_vec(ikp);
                map_ki(idx) = ki_vec(iki);
                map_kd(idx) = kd_vec(ikd);

                [is_stable, pm_min, gm_min] = check_stability_fast( ...
                    kp_vec(ikp), ki_vec(iki), kd_vec(ikd), opt_cfg);

                map_stable(idx) = is_stable;
                map_pm_min(idx) = pm_min;
                map_gm_min(idx) = gm_min;
            end
        end
        if mod(ikp, max(1,round(STAB_MAP_N_KP/10))) == 0
            fprintf('    %d / %d Kp slices done (%.1f s)\n', ikp, STAB_MAP_N_KP, toc(t_scan_start));
        end
    end
    t_scan = toc(t_scan_start);

    n_stable   = sum(map_stable);
    n_unstable = sum(~map_stable);
    fprintf('  Done in %.1f s.  Stable: %d (%.1f%%)  Unstable: %d (%.1f%%)\n', ...
        t_scan, n_stable, 100*n_stable/n_total, n_unstable, 100*n_unstable/n_total);

    % ---- 3D Scatter Plot ----
    figure;
    hold on;

    if any(~map_stable)
        scatter3(map_kd(~map_stable), map_ki(~map_stable), map_kp(~map_stable), ...
            STAB_MAP_MARKER_SIZE, 'r', 'filled', 'MarkerFaceAlpha', 0.25, ...
            'DisplayName', sprintf('Unstable (%d)', n_unstable));
    end
    if any(map_stable)
        scatter3(map_kd(map_stable), map_ki(map_stable), map_kp(map_stable), ...
            STAB_MAP_MARKER_SIZE, 'g', 'filled', 'MarkerFaceAlpha', 0.5, ...
            'DisplayName', sprintf('Stable (%d)', n_stable));
    end

    % Mark optimizer result if available
    if exist('opt_result', 'var') && isfield(opt_result, 'Kp_target')
        scatter3(opt_result.Kd_target, opt_result.Ki_target, opt_result.Kp_target, ...
            120, 'b', 'p', 'filled', 'LineWidth', 1.5, ...
            'DisplayName', 'Optimizer result');
    end

    hold off;
    if STAB_MAP_LOG_SCALE
        set(gca, 'XScale', 'log', 'YScale', 'log', 'ZScale', 'log');
    end
    xlabel('Kd');
    ylabel('Ki');
    zlabel('Kp');
    title(sprintf('3D Stability Map  (PM \geq %.0f°, GM \geq %.0f dB)  —  %d loads', ...
        STAB_MAP_MIN_PM_DEG, STAB_MAP_MIN_GM_DB, numel(STAB_MAP_ILOAD_VEC)));
    legend('Location', 'best');
    grid on;
    view(35, 25);
    rotate3d on;

    % ---- 2D Slice Projections ----
    figure;

    % Kd vs Ki (collapse Kp: stable if stable for ANY Kp)
    subplot(1,3,1);
    hold on;
    for iki = 1:STAB_MAP_N_KI
        for ikd = 1:STAB_MAP_N_KD
            slice_idx = ((1:STAB_MAP_N_KP)' - 1) * STAB_MAP_N_KI * STAB_MAP_N_KD ...
                        + (iki-1) * STAB_MAP_N_KD + ikd;
            if any(map_stable(slice_idx))
                plot(kd_vec(ikd), ki_vec(iki), 'gs', 'MarkerSize', 4, 'MarkerFaceColor', 'g');
            else
                plot(kd_vec(ikd), ki_vec(iki), 'rs', 'MarkerSize', 4, 'MarkerFaceColor', 'r');
            end
        end
    end
    if STAB_MAP_LOG_SCALE, set(gca, 'XScale', 'log', 'YScale', 'log'); end
    xlabel('Kd'); ylabel('Ki'); title('Kd vs Ki (any Kp)');
    grid on; hold off;

    % Kd vs Kp (collapse Ki: stable if stable for ANY Ki)
    subplot(1,3,2);
    hold on;
    for ikp = 1:STAB_MAP_N_KP
        for ikd = 1:STAB_MAP_N_KD
            slice_idx = (ikp-1) * STAB_MAP_N_KI * STAB_MAP_N_KD ...
                        + ((1:STAB_MAP_N_KI)' - 1) * STAB_MAP_N_KD + ikd;
            if any(map_stable(slice_idx))
                plot(kd_vec(ikd), kp_vec(ikp), 'gs', 'MarkerSize', 4, 'MarkerFaceColor', 'g');
            else
                plot(kd_vec(ikd), kp_vec(ikp), 'rs', 'MarkerSize', 4, 'MarkerFaceColor', 'r');
            end
        end
    end
    if STAB_MAP_LOG_SCALE, set(gca, 'XScale', 'log', 'YScale', 'log'); end
    xlabel('Kd'); ylabel('Kp'); title('Kd vs Kp (any Ki)');
    grid on; hold off;

    % Ki vs Kp (collapse Kd: stable if stable for ANY Kd)
    subplot(1,3,3);
    hold on;
    for ikp = 1:STAB_MAP_N_KP
        for iki = 1:STAB_MAP_N_KI
            slice_idx = (ikp-1) * STAB_MAP_N_KI * STAB_MAP_N_KD ...
                        + (iki-1) * STAB_MAP_N_KD + (1:STAB_MAP_N_KD)';
            if any(map_stable(slice_idx))
                plot(ki_vec(iki), kp_vec(ikp), 'gs', 'MarkerSize', 4, 'MarkerFaceColor', 'g');
            else
                plot(ki_vec(iki), kp_vec(ikp), 'rs', 'MarkerSize', 4, 'MarkerFaceColor', 'r');
            end
        end
    end
    if STAB_MAP_LOG_SCALE, set(gca, 'XScale', 'log', 'YScale', 'log'); end
    xlabel('Ki'); ylabel('Kp'); title('Ki vs Kp (any Kd)');
    grid on; hold off;

    sgtitle(sprintf('2D Stability Projections  (PM \geq %.0f°, GM \geq %.0f dB)', ...
        STAB_MAP_MIN_PM_DEG, STAB_MAP_MIN_GM_DB));
end

%% ==================== LOCAL FUNCTIONS ==

function result = simple_optimize_pid(cfg)
    method = lower(cfg.optimizer.method);

    switch method
        case {'nelder-mead','fminsearch','nm'}
            result = simple_optimize_pid_nm(cfg);

        case 'pso'
            result = simple_optimize_pid_pso(cfg);

        otherwise
            error('Unknown optimizer method: %s. Use ''nelder-mead'' or ''pso''.', cfg.optimizer.method);
    end
end

function result = simple_optimize_pid_nm(cfg)
    cost_history_tracker('reset');

    opts = optimset( ...
        'Display', ternary(cfg.SIMPLE_OPT_VERBOSE, 'iter', 'off'), ...
        'MaxIter', cfg.OPT_MAX_ITER, ...
        'MaxFunEvals', 8*cfg.OPT_MAX_ITER, ...
        'TolX', 1e-3, ...
        'TolFun', 1e-3);

    nstarts = cfg.optimizer.nm_nstarts;
    xbest = [];
    best_cost = inf;

    lb = log10([cfg.OPT_KP_MIN, cfg.OPT_KI_MIN, cfg.OPT_KD_MIN]);
    ub = log10([cfg.OPT_KP_MAX, cfg.OPT_KI_MAX, cfg.OPT_KD_MAX]);

    x0_list = zeros(nstarts + 1, 3);
    x0_list(1,:) = 0.5 * (lb + ub);

    rng(cfg.optimizer.random_seed);
    for k = 2:(nstarts + 1)
        x0_list(k,:) = lb + rand(1,3) .* (ub - lb);
    end

    for k = 1:size(x0_list,1)
        x0 = x0_list(k,:);
        xtry = fminsearch(@(x) pid_margin_cost_multiload(x, cfg), x0, opts);
        ctry = pid_margin_cost_multiload(xtry, cfg);

        if ctry < best_cost
            best_cost = ctry;
            xbest = xtry;
        end
    end

    result = evaluate_pid_candidate_multiload(10.^xbest, cfg);
    result.best_cost = best_cost;
    result.optimizer_method = 'nelder-mead';
    result.convergence = cost_history_tracker('get');
end

function result = simple_optimize_pid_pso(cfg)
    cost_history_tracker('reset');

    if exist('particleswarm', 'file') ~= 2
        error(['particleswarm is not available. Install Global Optimization Toolbox ' ...
               'or switch OPT_METHOD to ''nelder-mead''.']);
    end

    lb = log10([cfg.OPT_KP_MIN, cfg.OPT_KI_MIN, cfg.OPT_KD_MIN]);
    ub = log10([cfg.OPT_KP_MAX, cfg.OPT_KI_MAX, cfg.OPT_KD_MAX]);

    rng(cfg.optimizer.random_seed);

    pso_opts = optimoptions('particleswarm', ...
        'Display', ternary(cfg.SIMPLE_OPT_VERBOSE, 'iter', 'off'), ...
        'SwarmSize', cfg.optimizer.pso_swarm_size, ...
        'MaxIterations', cfg.optimizer.pso_max_iter, ...
        'FunctionTolerance', 1e-3);

    [xbest, best_cost] = particleswarm( ...
        @(x) pid_margin_cost_multiload(x, cfg), ...
        3, lb, ub, pso_opts);

    if cfg.optimizer.pso_use_hybrid
        cost_history_tracker('mark');

        nm_opts = optimset( ...
            'Display', ternary(cfg.SIMPLE_OPT_VERBOSE, 'iter', 'off'), ...
            'MaxIter', cfg.OPT_MAX_ITER, ...
            'MaxFunEvals', 8*cfg.OPT_MAX_ITER, ...
            'TolX', 1e-3, ...
            'TolFun', 1e-3);

        xbest_nm = fminsearch(@(x) pid_margin_cost_multiload(x, cfg), xbest, nm_opts);
        best_cost_nm = pid_margin_cost_multiload(xbest_nm, cfg);

        if best_cost_nm < best_cost
            xbest = xbest_nm;
            best_cost = best_cost_nm;
        end
    end

    result = evaluate_pid_candidate_multiload(10.^xbest, cfg);
    result.best_cost = best_cost;
    result.optimizer_method = 'pso';
    result.convergence = cost_history_tracker('get');
end

function cost = pid_margin_cost_multiload(xlog, cfg)
    vals = 10.^xlog;
    rep = evaluate_pid_candidate_multiload(vals, cfg);

    overflow_pen = 0;
    if abs(rep.K1) > cfg.K1_MAX, overflow_pen = overflow_pen + 200; end
    if abs(rep.K2) > cfg.K2_MAX, overflow_pen = overflow_pen + 200; end
    if abs(rep.K3) > cfg.K3_MAX, overflow_pen = overflow_pen + 200; end

    if ~rep.valid
        cost = 1e6 + overflow_pen;
        return;
    end

    pm_min_use = min(rep.Pm_deg_min, cfg.OPT_PM_CAP);
    gm_min_use = min(rep.Gm_dB_min, cfg.OPT_GM_CAP_DB);
    pm_avg_use = min(rep.Pm_deg_avg, cfg.OPT_PM_CAP);
    gm_avg_use = min(rep.Gm_dB_avg, cfg.OPT_GM_CAP_DB);

    fc_pen = mean(abs(log10(max(rep.Wcp_rad_vec, 1) / (2*pi*cfg.fc))));
    spread_pen = std(rep.Pm_deg_vec) + 0.5*std(rep.Gm_dB_vec);

    score = cfg.OPT_W_PM_MIN * pm_min_use + ...
            cfg.OPT_W_GM_MIN * gm_min_use + ...
            cfg.OPT_W_PM_AVG * pm_avg_use + ...
            cfg.OPT_W_GM_AVG * gm_avg_use - ...
            cfg.OPT_W_FC * fc_pen - ...
            cfg.OPT_W_SPREAD * spread_pen;

    if isfield(cfg, 'OPT_ENABLE_LOADSTEP_METRIC') && cfg.OPT_ENABLE_LOADSTEP_METRIC
        slew_bonus = min(rep.LoadStepRecoverySlew_Vus_worst / max(cfg.OPT_LOADSTEP_SLEW_REF_VUS, eps), 10);
        settle_pen = rep.LoadStepSettle_us_worst / max(cfg.OPT_LOADSTEP_SETTLE_REF_US, eps);
        undershoot_pen = rep.LoadStepUndershoot_mV_worst / max(cfg.OPT_LOADSTEP_UNDERSHOOT_REF_MV, eps);

        score = score + ...
            cfg.OPT_W_LOADSTEP_SLEW * slew_bonus - ...
            cfg.OPT_W_LOADSTEP_SETTLE * settle_pen - ...
            cfg.OPT_W_LOADSTEP_UNDERSHOOT * undershoot_pen;
    end

    cost = -score + overflow_pen;

    cost_history_tracker('record', cost);
end

function rep = evaluate_pid_candidate_multiload(pid_vals, cfg)
    s = tf('s');

    Kp_target = pid_vals(1);
    Ki_target = pid_vals(2);
    Kd_target = pid_vals(3);

    valid = true;

    if Kp_target < cfg.OPT_KP_MIN || Kp_target > cfg.OPT_KP_MAX, valid = false; end
    if Ki_target < cfg.OPT_KI_MIN || Ki_target > cfg.OPT_KI_MAX, valid = false; end
    if Kd_target < cfg.OPT_KD_MIN || Kd_target > cfg.OPT_KD_MAX, valid = false; end

    Kp_prog = Kp_target / cfg.G_digital;
    Ki_prog = Ki_target / cfg.G_digital;
    Kd_prog = Kd_target / cfg.G_digital;

    P_real = Kp_prog;
    I_real = Ki_prog * cfg.Tctrl;
    D_real = Kd_prog / cfg.Tctrl;

    P_cmd = round(P_real);
    I_cmd = round(I_real);
    D_cmd = round(D_real);

    [K1, K2, K3] = lab_position_to_k123(P_cmd, I_cmd, D_cmd);
    [Kp_prog_q, Ki_prog_q, Kd_prog_q] = lab_position_to_pid(P_cmd, I_cmd, D_cmd, cfg.Tctrl);

    if abs(K1) > cfg.K1_MAX || abs(K2) > cfg.K2_MAX || abs(K3) > cfg.K3_MAX
        valid = false;
    end

    Kp_ctrl_eff = Kp_prog_q * cfg.G_digital;
    Ki_ctrl_eff = Ki_prog_q * cfg.G_digital;
    Kd_ctrl_eff = Kd_prog_q * cfg.G_digital;

    switch lower(cfg.controller_domain)
        case 'laplace'
            Hpid_i = Kp_prog_q + Ki_prog_q/s + (Kd_prog_q*s)/(cfg.Tf*s + 1);
            Hdig_i = minreal(cfg.G_ADC * cfg.Hadc * Hpid_i * cfg.G_digital * cfg.G_DAC * cfg.Hdac * cfg.Hblank * cfg.Delay);

        case 'z'
            Cz_i = tf([K1 K2 K3], [1 -1], cfg.Tctrl, 'Variable', 'z^-1');
            Hpid_i = d2c(cfg.G_digital * Cz_i, 'tustin');
            Hdig_i = minreal(cfg.G_ADC * cfg.Hadc * Hpid_i * cfg.G_DAC * cfg.Hdac * cfg.Hblank * cfg.Delay);

        otherwise
            error('cfg.controller_domain must be ''laplace'' or ''z''.');
    end

    nL = numel(cfg.Iload_vec);
    Gm_dB_vec = zeros(1, nL);
    Pm_deg_vec = zeros(1, nL);
    Wcp_rad_vec = zeros(1, nL);
    Fc_kHz_vec = zeros(1, nL);

    LoadStepRecoverySlew_Vus_vec = zeros(1, nL);
    LoadStepSettle_us_vec = inf(1, nL);
    LoadStepUndershoot_mV_vec = inf(1, nL);
    LoadStepEvalRan_vec = false(1, nL);

    for ii = 1:nL
        Gvc_i = build_pcm_plant_at_iload(cfg, cfg.Iload_vec(ii));
        Lloop_i = Hdig_i * Gvc_i * cfg.H;
        Tcl_i = minreal(feedback(Lloop_i, 1));

        [Gm, Pm, ~, Wcp] = margin(Lloop_i);

        closed_loop_stable = isstable(Tcl_i);
        margins_valid = ~(isempty(Gm) || isempty(Pm) || isempty(Wcp) || ...
                          isnan(Pm) || isnan(Wcp) || Wcp <= 0);

        if ~closed_loop_stable || ~margins_valid
            valid = false;
            Gm_dB_vec(ii) = -Inf;
            Pm_deg_vec(ii) = -Inf;
            Wcp_rad_vec(ii) = NaN;
            Fc_kHz_vec(ii) = NaN;
            continue;
        end

        if isinf(Gm)
            Gm_dB = cfg.OPT_GM_CAP_DB;
        else
            Gm_dB = 20*log10(max(Gm, eps));
        end

        Gm_dB_vec(ii) = Gm_dB;
        Pm_deg_vec(ii) = Pm;
        Wcp_rad_vec(ii) = Wcp;
        Fc_kHz_vec(ii) = Wcp / (2*pi*1e3);

        meets_margin_targets = (Pm >= cfg.OPT_MIN_PM_DEG) && (Gm_dB >= cfg.OPT_MIN_GM_DB);
        if ~meets_margin_targets
            valid = false;
        end

        run_loadstep = isfield(cfg, 'OPT_ENABLE_LOADSTEP_METRIC') && ...
                       cfg.OPT_ENABLE_LOADSTEP_METRIC && ...
                       meets_margin_targets;

        if run_loadstep
            ls = evaluate_loadstep_response_metric(cfg, Lloop_i, cfg.Iload_vec(ii), K1, K2, K3);
            LoadStepRecoverySlew_Vus_vec(ii) = ls.recovery_slew_vus;
            LoadStepSettle_us_vec(ii) = ls.settle_us;
            LoadStepUndershoot_mV_vec(ii) = ls.undershoot_mv;
            LoadStepEvalRan_vec(ii) = true;

            if ~ls.valid
                valid = false;
            end
        end
    end

    good_pm = Pm_deg_vec(isfinite(Pm_deg_vec));
    good_gm = Gm_dB_vec(isfinite(Gm_dB_vec));

    if isempty(good_pm) || isempty(good_gm)
        valid = false;
        Pm_deg_min = -Inf;
        Gm_dB_min = -Inf;
        Pm_deg_avg = -Inf;
        Gm_dB_avg = -Inf;
    else
        Pm_deg_min = min(good_pm);
        Gm_dB_min = min(good_gm);
        Pm_deg_avg = mean(good_pm);
        Gm_dB_avg = mean(good_gm);
    end

    rep = struct();
    rep.valid = valid;

    rep.Kp_target = Kp_target;
    rep.Ki_target = Ki_target;
    rep.Kd_target = Kd_target;

    rep.P_cmd = P_cmd;
    rep.I_cmd = I_cmd;
    rep.D_cmd = D_cmd;

    rep.K1 = K1;
    rep.K2 = K2;
    rep.K3 = K3;

    rep.Kp_ctrl_eff = Kp_ctrl_eff;
    rep.Ki_ctrl_eff = Ki_ctrl_eff;
    rep.Kd_ctrl_eff = Kd_ctrl_eff;

    rep.Iload_vec = cfg.Iload_vec;
    rep.Gm_dB_vec = Gm_dB_vec;
    rep.Pm_deg_vec = Pm_deg_vec;
    rep.Wcp_rad_vec = Wcp_rad_vec;
    rep.Fc_kHz_vec = Fc_kHz_vec;

    rep.Gm_dB_min = Gm_dB_min;
    rep.Pm_deg_min = Pm_deg_min;
    rep.Gm_dB_avg = Gm_dB_avg;
    rep.Pm_deg_avg = Pm_deg_avg;

    rep.LoadStepRecoverySlew_Vus_vec = LoadStepRecoverySlew_Vus_vec;
    rep.LoadStepSettle_us_vec = LoadStepSettle_us_vec;
    rep.LoadStepUndershoot_mV_vec = LoadStepUndershoot_mV_vec;
    rep.LoadStepEvalRan_vec = LoadStepEvalRan_vec;

    if any(isfinite(LoadStepRecoverySlew_Vus_vec))
        rep.LoadStepRecoverySlew_Vus_worst = min(LoadStepRecoverySlew_Vus_vec(isfinite(LoadStepRecoverySlew_Vus_vec)));
    else
        rep.LoadStepRecoverySlew_Vus_worst = 0;
    end

    if any(isfinite(LoadStepSettle_us_vec))
        rep.LoadStepSettle_us_worst = max(LoadStepSettle_us_vec(isfinite(LoadStepSettle_us_vec)));
    else
        rep.LoadStepSettle_us_worst = inf;
    end

    if any(isfinite(LoadStepUndershoot_mV_vec))
        rep.LoadStepUndershoot_mV_worst = max(LoadStepUndershoot_mV_vec(isfinite(LoadStepUndershoot_mV_vec)));
    else
        rep.LoadStepUndershoot_mV_worst = inf;
    end
end

function Gvc_i = build_pcm_plant_at_iload(cfg, Iload)
    s = tf('s');

    R = cfg.VOUT / Iload;
    wp = 1/(cfg.C*R) + (cfg.Ts*(cfg.mc*cfg.D_tag - 0.5)/(cfg.Lind*cfg.C));
    wn = pi/cfg.Ts;
    Qp = 1/(pi*(cfg.mc*cfg.D_tag - 0.5));

    Hdc = (R/cfg.Ri) * (1 / (1 + R*cfg.Ts*(cfg.mc*cfg.D_tag - 0.5)/cfg.Lind));
    Fp = (1 + s*cfg.C*cfg.Resr) / (1 + s/wp);
    Fh = 1 / (1 + s/(wn*Qp) + (s^2)/(wn^2));

    Gvc_i = minreal(Hdc * Fp * Fh);
end

function [Kp_out, Ki_out, Kd_out] = k123_to_pid(K1, K2, K3, Ts)
    Kd_out = K3 * Ts;
    Kp_out = -K2 - 2*K3;
    Ki_out = (K1 + K2 + K3) / Ts;
end

function [Kp_prog, Ki_prog, Kd_prog] = lab_position_to_pid(P_in, I_in, D_in, Ts)
    [K1_tmp, K2_tmp, K3_tmp] = lab_position_to_k123(P_in, I_in, D_in);
    [Kp_prog, Ki_prog, Kd_prog] = k123_to_pid(K1_tmp, K2_tmp, K3_tmp, Ts);
end

function [K1_out, K2_out, K3_out] = lab_position_to_k123(P_in, I_in, D_in)
    K1_out = P_in + I_in + D_in;
    K2_out = -P_in - 2*D_in;
    K3_out = D_in;
end

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end

function [is_stable, pm_min, gm_min_dB] = check_stability_fast(Kp_target, Ki_target, Kd_target, cfg)
    s = tf('s');
    is_stable = true;
    pm_min = inf;
    gm_min_dB = inf;

    Kp_prog = Kp_target / cfg.G_digital;
    Ki_prog = Ki_target / cfg.G_digital;
    Kd_prog = Kd_target / cfg.G_digital;

    P_cmd = round(Kp_prog);
    I_cmd = round(Ki_prog * cfg.Tctrl);
    D_cmd = round(Kd_prog / cfg.Tctrl);

    [K1, K2, K3] = lab_position_to_k123(P_cmd, I_cmd, D_cmd);
    if abs(K1) > cfg.K1_MAX || abs(K2) > cfg.K2_MAX || abs(K3) > cfg.K3_MAX
        is_stable = false;
        pm_min = -Inf;
        gm_min_dB = -Inf;
        return;
    end

    [Kp_prog_q, Ki_prog_q, Kd_prog_q] = lab_position_to_pid(P_cmd, I_cmd, D_cmd, cfg.Tctrl);

    switch lower(cfg.controller_domain)
        case 'laplace'
            Hpid_i = Kp_prog_q + Ki_prog_q/s + (Kd_prog_q*s)/(cfg.Tf*s + 1);
            Hdig_i = minreal(cfg.G_ADC * cfg.Hadc * Hpid_i * cfg.G_digital * cfg.G_DAC * cfg.Hdac * cfg.Hblank * cfg.Delay);
        case 'z'
            Cz_i = tf([K1 K2 K3], [1 -1], cfg.Tctrl, 'Variable', 'z^-1');
            Hpid_i = d2c(cfg.G_digital * Cz_i, 'tustin');
            Hdig_i = minreal(cfg.G_ADC * cfg.Hadc * Hpid_i * cfg.G_DAC * cfg.Hdac * cfg.Hblank * cfg.Delay);
    end

    for ii = 1:numel(cfg.STAB_MAP_ILOAD_VEC)
        Gvc_i = build_pcm_plant_at_iload(cfg, cfg.STAB_MAP_ILOAD_VEC(ii));
        Lloop_i = Hdig_i * Gvc_i * cfg.H;

        try
            Tcl_i = minreal(feedback(Lloop_i, 1));
            if ~isstable(Tcl_i)
                is_stable = false;
                pm_min = -Inf;
                gm_min_dB = -Inf;
                return;
            end

            [Gm, Pm, ~, Wcp] = margin(Lloop_i);

            if isempty(Gm) || isempty(Pm) || isempty(Wcp) || isnan(Pm) || isnan(Wcp) || Wcp <= 0
                is_stable = false;
                pm_min = -Inf;
                gm_min_dB = -Inf;
                return;
            end

            if isinf(Gm)
                Gm_dB = cfg.STAB_MAP_MIN_GM_DB + 10;
            else
                Gm_dB = 20*log10(max(Gm, eps));
            end

            pm_min = min(pm_min, Pm);
            gm_min_dB = min(gm_min_dB, Gm_dB);

            if Pm < cfg.STAB_MAP_MIN_PM_DEG || Gm_dB < cfg.STAB_MAP_MIN_GM_DB
                is_stable = false;
                return;
            end
        catch
            is_stable = false;
            pm_min = -Inf;
            gm_min_dB = -Inf;
            return;
        end
    end
end

function ls = evaluate_loadstep_response_metric(cfg, Lloop_i, Iload, K1, K2, K3)
    s = tf('s');

    ls = struct('valid', true, ...
                'recovery_slew_vus', 0, ...
                'settle_us', inf, ...
                'undershoot_mv', inf);

    Iload = max(Iload, 1e-6);
    R = cfg.VOUT / Iload;

    use_nonlinear = isfield(cfg, 'OPT_USE_NONLINEAR_PREDICTOR') && ...
                    cfg.OPT_USE_NONLINEAR_PREDICTOR;

    if use_nonlinear
        % ---- V5-style cycle-by-cycle nonlinear predictor ----
        Gvc_i = build_pcm_plant_at_iload(cfg, Iload);

        Zc = cfg.Resr + 1/(s*cfg.C);
        Zout_ol = minreal((Zc * R) / (Zc + R));
        Gload_i = minreal(-Zout_ol);   % passive output impedance only

        ls_cfg = struct();
        ls_cfg.Ts        = cfg.Tctrl;
        ls_cfg.Tctrl     = cfg.Tctrl;
        ls_cfg.tstop     = cfg.OPT_LOADSTEP_DELAY_S + cfg.OPT_LOADSTEP_TFINAL;
        ls_cfg.VOUT_NOM  = cfg.VOUT;
        ls_cfg.VOUT_INIT = cfg.OPT_LOADSTEP_VOUT_INIT;
        ls_cfg.beta      = cfg.beta;
        ls_cfg.VREF_FB   = cfg.beta * cfg.Vref;

        ls_cfg.K1 = K1;
        ls_cfg.K2 = K2;
        ls_cfg.K3 = K3;
        ls_cfg.SHIFT_BITS       = cfg.SHIFT_BITS;
        ls_cfg.PID_OUT_BITS     = cfg.PID_OUT_BITS;
        ls_cfg.PID_OUT_OFFSET   = cfg.PID_OUT_OFFSET;
        ls_cfg.PID_PRECLAMP_MIN = cfg.PID_PRECLAMP_MIN;
        ls_cfg.PID_PRECLAMP_MAX = cfg.PID_PRECLAMP_MAX;

        ls_cfg.N_ADC     = cfg.N_ADC;
        ls_cfg.VADC_MIN  = cfg.VADC_MIN;
        ls_cfg.VADC_MAX  = cfg.VADC_MAX;

        ls_cfg.N_DAC     = cfg.N_DAC;
        ls_cfg.V_FS_DAC  = cfg.V_FS_DAC;
        ls_cfg.wp_dac    = cfg.wp_dac;
        ls_cfg.DAC_OUT_MIN = cfg.DAC_OUT_MIN;
        ls_cfg.DAC_OUT_MAX = cfg.DAC_OUT_MAX;

        ls_cfg.use_quantized_adc_dac = cfg.USE_QUANTIZED_ADC_DAC;
        ls_cfg.use_preclamp_state    = cfg.USE_PRECLAMP_STATE;

        ls_cfg.load_step_enable = true;
        ls_cfg.load_step_i0     = Iload;
        ls_cfg.load_step_di     = cfg.OPT_LOADSTEP_DI_A;
        ls_cfg.load_step_delay  = cfg.OPT_LOADSTEP_DELAY_S;
        ls_cfg.load_step_rise   = cfg.OPT_LOADSTEP_RISE_S;
        ls_cfg.load_step_width  = cfg.OPT_LOADSTEP_WIDTH_S;
        ls_cfg.load_step_fall   = cfg.OPT_LOADSTEP_FALL_S;

        [t, vout, ~, ~, ~, ~, ~] = run_large_signal_predictor(Gvc_i, Gload_i, ls_cfg);

        if isempty(vout) || any(~isfinite(vout))
            ls.valid = false;
            return;
        end

        % Extract metrics from nonlinear waveform
        t = t(:);
        vout = vout(:);

        % Use pre-step window to get baseline Vout
        idx_pre = (t >= max(0, ls_cfg.load_step_delay - 1e-6)) & ...
                  (t < ls_cfg.load_step_delay);
        if any(idx_pre)
            vout_pre = mean(vout(idx_pre));
        else
            vout_pre = cfg.VOUT;
        end

        % Post-step deviation
        idx_post = (t >= ls_cfg.load_step_delay);
        y = vout(idx_post) - vout_pre;   % deviation from baseline
        t_post = t(idx_post) - ls_cfg.load_step_delay;  % time from step edge
    else
        % ---- Linear step() approach (original) ----
        Zc = cfg.Resr + 1/(s*cfg.C);
        Zout_ol = minreal(1 / (1/R + 1/Zc));
        Gload_i = minreal(-Zout_ol / (1 + Lloop_i));

        t_lin = linspace(0, cfg.OPT_LOADSTEP_TFINAL, cfg.OPT_LOADSTEP_NPTS);
        y = step(cfg.OPT_LOADSTEP_DI_A * Gload_i, t_lin);
        y = squeeze(y);

        if isempty(y) || any(~isfinite(y))
            ls.valid = false;
            return;
        end

        y = y(:);
        t_post = t_lin(:);
    end

    % ---- Common metric extraction ----
    undershoot_v = max(0, -min(y));
    [~, idx_min] = min(y);

    if idx_min < numel(y)
        dy = diff(y(idx_min:end));
        dt = diff(t_post(idx_min:end));
        recovery_slew_vps = max([0; dy ./ dt]);
    else
        recovery_slew_vps = 0;
    end

    band_v = max(cfg.OPT_LOADSTEP_ERR_BAND_V, 0.02 * max(abs(y)));
    idx_last = find(abs(y) > band_v, 1, 'last');

    if isempty(idx_last)
        settle_s = 0;
    else
        settle_s = t_post(idx_last);
    end

    ls.recovery_slew_vus = recovery_slew_vps / 1e6;
    ls.settle_us = settle_s * 1e6;
    ls.undershoot_mv = undershoot_v * 1e3;
end

function [t, vout, e_hist, u_hist, pid_hist, vdac_hist, iload_hist] = run_large_signal_predictor(Gvc, Gload, cfg)
    sysd_ctrl = c2d(ss(minreal(Gvc)), cfg.Tctrl, 'zoh');
    [Ad_ctrl, Bd_ctrl, Cd_ctrl, Dd_ctrl] = ssdata(sysd_ctrl);

    sysd_load = c2d(ss(minreal(Gload)), cfg.Tctrl, 'zoh');
    [Ad_load, Bd_load, Cd_load, Dd_load] = ssdata(sysd_load);

    N  = floor(cfg.tstop / cfg.Tctrl) + 1;
    nx_ctrl = size(Ad_ctrl, 1);
    nx_load = size(Ad_load, 1);

    t = (0:N-1) * cfg.Tctrl;
    vout = zeros(1, N);
    e_hist = zeros(1, N);
    u_hist = zeros(1, N);
    pid_hist = zeros(1, N);
    vdac_hist = zeros(1, N);
    iload_hist = zeros(1, N);

    dv0 = cfg.VOUT_INIT - cfg.VOUT_NOM;
    x_ctrl = zeros(nx_ctrl, 1);
    x_load = zeros(nx_load, 1);
    if ~isempty(Cd_ctrl)
        x_ctrl = pinv(Cd_ctrl) * dv0;
    end

    e1 = 0;
    e2 = 0;
    u_state = 0;

    pid_code_min = 0;
    pid_code_max = 2^(cfg.PID_OUT_BITS) - 1;
    pid_offset   = cfg.PID_OUT_OFFSET;

    pid_delta_min = -pid_offset;
    pid_delta_max = pid_code_max - pid_offset;

    u_min = cfg.PID_PRECLAMP_MIN;
    u_max = cfg.PID_PRECLAMP_MAX;

    dac_code_min = 0;
    dac_code_max = 2^(cfg.N_DAC) - 1;
    dac_offset   = 2^(cfg.N_DAC-1);

    a_dac = exp(-cfg.wp_dac * cfg.Tctrl);
    b_dac = 1 - a_dac;

    v_dac_mid = cfg.DAC_OUT_MIN + ...
        (dac_offset / dac_code_max) * (cfg.DAC_OUT_MAX - cfg.DAC_OUT_MIN);
    v_dac_mid = min(max(v_dac_mid, cfg.DAC_OUT_MIN), cfg.DAC_OUT_MAX);

    v_dac_filt = v_dac_mid;

    for k = 1:N
        di_load = load_step_profile(t(k), cfg);
        iload_hist(k) = cfg.load_step_i0 + di_load;

        dvout_ctrl = Cd_ctrl*x_ctrl;
        if ~isempty(Dd_ctrl)
            dvout_ctrl = dvout_ctrl + Dd_ctrl*0;
        end
        dvout_load = Cd_load*x_load;
        if ~isempty(Dd_load)
            dvout_load = dvout_load + Dd_load*di_load;
        end

        dvout = double(dvout_ctrl(1) + dvout_load(1));
        vout(k) = cfg.VOUT_NOM + dvout;

        vfb = cfg.beta * vout(k);

        if cfg.use_quantized_adc_dac
            adc_code = quantize_adc_code(vfb, cfg.VADC_MIN, cfg.VADC_MAX, cfg.N_ADC);
            ref_code = quantize_adc_code(cfg.VREF_FB, cfg.VADC_MIN, cfg.VADC_MAX, cfg.N_ADC);
            e0 = double(ref_code - adc_code);
        else
            e0 = cfg.VREF_FB - vfb;
        end

        u_pre = u_state + cfg.K1*e0 + cfg.K2*e1 + cfg.K3*e2;
        u_sat = min(max(u_pre, u_min), u_max);

        u_state = u_sat;

        pid_delta = round(u_sat / 2^(cfg.SHIFT_BITS));
        pid_delta = min(max(pid_delta, pid_delta_min), pid_delta_max);

        pid_code = pid_delta + pid_offset;
        pid_code = min(max(pid_code, pid_code_min), pid_code_max);

        dac_delta = pid_code - pid_offset;
        dac_code  = dac_delta + dac_offset;
        dac_code  = min(max(dac_code, dac_code_min), dac_code_max);

        v_dac_cmd = cfg.DAC_OUT_MIN + ...
            (dac_code / dac_code_max) * (cfg.DAC_OUT_MAX - cfg.DAC_OUT_MIN);
        v_dac_cmd = min(max(v_dac_cmd, cfg.DAC_OUT_MIN), cfg.DAC_OUT_MAX);

        v_dac_filt = a_dac * v_dac_filt + b_dac * v_dac_cmd;

        u_plant = v_dac_filt - v_dac_mid;

        x_ctrl = Ad_ctrl*x_ctrl + Bd_ctrl*u_plant;
        x_load = Ad_load*x_load + Bd_load*di_load;

        e_hist(k) = e0;
        u_hist(k) = u_sat;
        pid_hist(k) = pid_code;
        vdac_hist(k) = v_dac_filt;

        e2 = e1;
        e1 = e0;
    end
end

function di = load_step_profile(t, cfg)
    if ~cfg.load_step_enable
        di = 0;
        return;
    end

    di = 0;

    if t < cfg.load_step_delay
        return;
    end

    rise_end = cfg.load_step_delay + cfg.load_step_rise;

    if t < rise_end
        di = cfg.load_step_di * (t - cfg.load_step_delay) / max(cfg.load_step_rise, eps);
        return;
    end

    if isinf(cfg.load_step_width)
        di = cfg.load_step_di;
        return;
    end

    high_end = rise_end + cfg.load_step_width;
    fall_end = high_end + cfg.load_step_fall;

    if t < high_end
        di = cfg.load_step_di;
    elseif t < fall_end
        di = cfg.load_step_di * (1 - (t - high_end) / max(cfg.load_step_fall, eps));
    else
        di = 0;
    end
end

function code = quantize_adc_code(vin, vmin, vmax, nbits)
    code_max = 2^nbits - 1;
    vin_clip = min(max(vin, vmin), vmax);
    code = round((vin_clip - vmin) / (vmax - vmin) * code_max);
    code = min(max(code, 0), code_max);
end

function varargout = cost_history_tracker(action, cost_val)
    persistent hist_t hist_cost hist_best t0 best_so_far n_evals mark_eval mark_t

    switch action
        case 'reset'
            hist_t    = [];
            hist_cost = [];
            hist_best = [];
            best_so_far = inf;
            n_evals = 0;
            mark_eval = [];
            mark_t    = [];
            t0 = tic;

        case 'record'
            n_evals = n_evals + 1;
            elapsed = toc(t0);
            if cost_val < best_so_far
                best_so_far = cost_val;
            end
            hist_t(end+1)    = elapsed;
            hist_cost(end+1) = cost_val;
            hist_best(end+1) = best_so_far;

        case 'mark'
            mark_eval = n_evals;
            mark_t    = toc(t0);

        case 'get'
            out = struct();
            out.elapsed_s  = hist_t;
            out.cost       = hist_cost;
            out.best_cost  = hist_best;
            out.n_evals    = n_evals;
            out.total_time = toc(t0);
            out.mark_eval  = mark_eval;
            out.mark_t     = mark_t;
            varargout{1} = out;
    end
end
