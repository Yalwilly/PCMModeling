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
K3_MAG_BITS = 12;   % 10->1023, 11->2047, 12->4095

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
OPT_W_SPREAD   = 0;

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

OPT_LOADSTEP_DI_A           = 2;      % load step amplitude [A]
OPT_LOADSTEP_TFINAL         = 40e-6;    % simulation window [s]
OPT_LOADSTEP_NPTS           = 400;      % keep modest for optimizer speed
OPT_LOADSTEP_ERR_BAND_V     = 2e-3;     % settle band [V]

OPT_W_LOADSTEP_SLEW         = 12;     % reward fast recovery slew
OPT_W_LOADSTEP_SETTLE       = 12;      % penalize slow settling
OPT_W_LOADSTEP_UNDERSHOOT   = 12;      % penalize deep undershoot

OPT_LOADSTEP_SLEW_REF_VUS   = 0.002;    % reference recovery slew [V/us]
OPT_LOADSTEP_SETTLE_REF_US  = 5;      % reference settling time [us]
OPT_LOADSTEP_UNDERSHOOT_REF_MV = 30;  % reference undershoot [mV]


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

%% ==================== RUN OPTIMIZATION ====================

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
end

%% ==================== LOCAL FUNCTIONS ====================

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
end

function result = simple_optimize_pid_pso(cfg)
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
            ls = evaluate_loadstep_response_metric(cfg, Lloop_i, cfg.Iload_vec(ii));
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

function ls = evaluate_loadstep_response_metric(cfg, Lloop_i, Iload)
    s = tf('s');

    ls = struct('valid', true, ...
                'recovery_slew_vus', 0, ...
                'settle_us', inf, ...
                'undershoot_mv', inf);

    Iload = max(Iload, 1e-6);
    R = cfg.VOUT / Iload;

    % Simple output-node disturbance model:
    % load current step -> output voltage disturbance
    Zc = cfg.Resr + 1/(s*cfg.C);
    Zout_ol = minreal(1 / (1/R + 1/Zc));

    % Closed-loop load disturbance transfer approximation
    Gload_i = minreal(-Zout_ol / (1 + Lloop_i));   % [V/A]

    t = linspace(0, cfg.OPT_LOADSTEP_TFINAL, cfg.OPT_LOADSTEP_NPTS);
    y = step(cfg.OPT_LOADSTEP_DI_A * Gload_i, t);
    y = squeeze(y);

    if isempty(y) || any(~isfinite(y))
        ls.valid = false;
        return;
    end

    y = y(:);
    t = t(:);

    undershoot_v = max(0, -min(y));
    [~, idx_min] = min(y);

    if idx_min < numel(y)
        dy = diff(y(idx_min:end));
        dt = diff(t(idx_min:end));
        recovery_slew_vps = max([0; dy ./ dt]);
    else
        recovery_slew_vps = 0;
    end

    band_v = max(cfg.OPT_LOADSTEP_ERR_BAND_V, 0.02 * max(abs(y)));
    idx_last = find(abs(y) > band_v, 1, 'last');

    if isempty(idx_last)
        settle_s = 0;
    else
        settle_s = t(idx_last);
    end

    ls.recovery_slew_vus = recovery_slew_vps / 1e6;
    ls.settle_us = settle_s * 1e6;
    ls.undershoot_mv = undershoot_v * 1e3;
end

