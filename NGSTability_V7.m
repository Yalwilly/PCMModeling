% filepath: NGSTability_V7.m
%% ====== NGSTability_V7.m ======
% PCM Buck Converter Stability Analysis + PID Optimization
% Merged V5 (analysis) + V6 (optimization) with fixes:
%   [Fix #1]  All executable code BEFORE local functions
%   [Fix #2]  Removed debug K1/K2/K3 overwrite
%   [Fix #3]  Consistent Tctrl normalization everywhere
%   [Fix #4]  Tf decoupled from fc (fixed physical constant)
%   [Fix #5]  Inductor added to load-step Zout model
%   [Fix #6]  Overshoot metric added to load-step evaluation
%   [Fix #7]  Pre-compute Hdig*H once (cache outside load loop)
%   [Fix #8]  Pre-compute plant array before optimizer runs
%   [Fix #9]  Reduced redundant minreal calls
%   [Fix #10] Early termination on margin failure
%   [Fix #13] Average recovery slew (min → band crossing)
%   [Fix #16] Weighted load points
%   [Fix #17] OPT_W_SPREAD enabled (default 0.5)
%   [Fix #18] Adaptive overflow penalty (gradient near boundary)
%   [Fix #19] Warm-start with known good solutions
%   [Fix #20] Merged analysis + optimization with RUN_MODE selector
%   [Fix #21] Input validation assertions
%   [Fix #22] fdig prominently flagged for verification

clear; clc; close all;

%% ==================== RUN MODE ====================
% 'analysis'  — Bode, step response, large-signal predictor only
% 'optimize'  — PID optimization only
% 'both'      — Run analysis first, then optimize
RUN_MODE = 'both';

%% ==================== USER SETTINGS ====================

% Controller model
CONTROLLER_DOMAIN = 'laplace';   % 'laplace' or 'z'

% RTL scaling / coefficient limits
SHIFT_BITS = 6;
G_digital  = 2^(-SHIFT_BITS);   % 1/64

K1_MAG_BITS = 19;
K2_MAG_BITS = 19;
K3_MAG_BITS = 12;

K1_MAX = 2^K1_MAG_BITS - 1;
K2_MAX = 2^K2_MAG_BITS - 1;
K3_MAX = 2^K3_MAG_BITS - 1;

% Feedback
beta = 0.5;
H    = beta;

% Converter parameters
VOUT = 0.8;
VIN  = 3.3;
Lind = 0.47e-6;
Vref = 0.8;

fs   = 6e6;
Ts   = 1/fs;
fc   = fs/10;            % target crossover frequency

% [Fix #4] Tf is a fixed physical constant, NOT coupled to fc
Tf   = Ts/10;            % derivative filter time constant

% [Fix #22] <<< VERIFY: 256 MHz or 128 MHz per current RTL >>>
fdig  = 256e6;
Tctrl = 1/fdig;

% Current sensing
GI     = (1/640)*(10/64)*(1/4);
Rsense = 4e3;
Ri     = GI * Rsense;

% Duty / slope compensation
D     = VOUT/VIN;
D_tag = 1 - D;

se = 0.7e6;
sn = (VIN - VOUT)/Lind;
sf = VOUT/Lind;
mc = 1 + se/sn;

% Output filter
C    = 22e-6;
Resr = 3e-3;

% ADC / DAC
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
    adc_dac_mode_text = 'small-signal ADC/DAC gains';
else
    G_ADC = 1;
    G_DAC = 1;
    adc_dac_mode_text = 'unity ADC/DAC gains';
end
G_adc_dac = G_ADC * G_DAC;
G_total   = G_adc_dac * G_digital;

% Delays
wp_dac  = 2*pi*500e3;
Td      = 8e-9;
Td_ADC  = 5e-9;
Tblank  = 15e-9;

% DAC output limits
DAC_VDD      = 1.8;
DAC_NMOS_VTH = 0.2;
DAC_OUT_MIN  = 0.0;
DAC_OUT_MAX  = DAC_VDD - DAC_NMOS_VTH;  % 1.6 V

%% ==================== ANALYSIS SETTINGS ====================

PLOT_BODE_GRAPHS         = true;
PLOT_SMALL_SIGNAL_GRAPHS = true;
PLOT_LARGE_SIGNAL_GRAPHS = true;
PHASE_YLIM               = [-270 90];

RUN_LARGE_SIGNAL_PREDICTOR = true;
LARGE_SIGNAL_TSTOP         = 100e-6;
LARGE_SIGNAL_VOUT_INIT     = 0.2;
USE_QUANTIZED_ADC_DAC      = true;
USE_PRECLAMP_STATE         = true;

PID_OUT_BITS       = 12;
PID_OUT_OFFSET     = 2^(PID_OUT_BITS-1);          % 2048
PID_PRECLAMP_MIN   = -2048 * 2^SHIFT_BITS;        % -131072
PID_PRECLAMP_MAX   =  2047 * 2^SHIFT_BITS;        %  131008
PLOT_BINARY_CODE_AXIS = false;

% Target controller PID gains (effective, after >>6)
Kp_target = 18.3028;
Ki_target = 7.01895e+06;
Kd_target = 6.23528e-08;

% Analysis load current
Iload_analysis = 100e-3;

%% ==================== OPTIMIZATION SETTINGS ====================

SIMPLE_OPT_VERBOSE = true;

OPT_I_LOAD_VEC = [0.001 0.10 0.30 0.50 0.70 0.90 1.10 2 3];

% [Fix #16] Per-load weights (emphasize mid-range operating points)
OPT_I_LOAD_WEIGHTS = [0.5 1 1 2 2 2 1 1 0.5];

OPT_MIN_PM_DEG = 35;
OPT_MIN_GM_DB  = 6;

OPT_W_PM_MIN  = 2;
OPT_W_GM_MIN  = 1;
OPT_W_PM_AVG  = 0.15;
OPT_W_GM_AVG  = 0.10;
OPT_W_FC      = 1;
OPT_W_SPREAD  = 0.5;    % [Fix #17] enabled — penalize PM/GM variance across loads

OPT_PM_CAP    = 85;
OPT_GM_CAP_DB = 40;
OPT_MAX_ITER  = 300;

OPT_KP_MIN = G_digital;             % 0.015625
OPT_KP_MAX = 10e3;
OPT_KI_MIN = 4e6;
OPT_KI_MAX = 5e8;
OPT_KD_MIN = 1 * Tctrl * G_digital;
OPT_KD_MAX = K3_MAX * Tctrl * G_digital;

% Load-step metric
OPT_ENABLE_LOADSTEP_METRIC = true;

OPT_LOADSTEP_DI_A           = 2;
OPT_LOADSTEP_TFINAL         = 40e-6;
OPT_LOADSTEP_NPTS           = 400;
OPT_LOADSTEP_ERR_BAND_V     = 2e-3;

OPT_W_LOADSTEP_SLEW         = 12;
OPT_W_LOADSTEP_SETTLE       = 12;
OPT_W_LOADSTEP_UNDERSHOOT   = 12;
OPT_W_LOADSTEP_OVERSHOOT    = 8;    % [Fix #6] new overshoot penalty

OPT_LOADSTEP_SLEW_REF_VUS       = 0.002;
OPT_LOADSTEP_SETTLE_REF_US      = 5;
OPT_LOADSTEP_UNDERSHOOT_REF_MV  = 30;
OPT_LOADSTEP_OVERSHOOT_REF_MV   = 20;  % [Fix #6] overshoot reference

% Optimizer selection
OPT_METHOD      = 'pso';   % 'nelder-mead' or 'pso'
OPT_RANDOM_SEED = 1;

OPT_NM_NSTARTS      = 24;
OPT_PSO_SWARM_SIZE  = 40;
OPT_PSO_MAX_ITER    = 80;
OPT_PSO_USE_HYBRID  = true;

% [Fix #19] Warm-start seeds (known good Kp_eff, Ki_eff, Kd_eff)
OPT_WARM_START = [
    32.3125,  8.0e6, 1.14e-7;   % AMS Set 1
    50.516,   1.0e7, 1.25e-7;   % AMS Set 2
    18.3028,  7.02e6, 6.24e-8;  % V5 target
];

%% ==================== INPUT VALIDATION ====================
% [Fix #21]
assert(VOUT < VIN,   'VOUT must be less than VIN for a buck converter.');
assert(fs > 0,       'Switching frequency must be positive.');
assert(fdig > 0,     'Digital clock frequency must be positive.');
assert(Lind > 0,     'Inductance must be positive.');
assert(C > 0,        'Capacitance must be positive.');
assert(K3_MAG_BITS >= 10, 'K3 bit width too narrow for meaningful Kd.');

%% ==================== PREPARE FIXED BLOCKS ====================

s = tf('s');

[numBlank, denBlank] = pade(Tblank, 2);
Hblank = tf(numBlank, denBlank);

[numD, denD] = pade(Td, 2);
Delay = tf(numD, denD);

[numADC, denADC] = pade(Td_ADC, 2);
Hadc = tf(numADC, denADC);

Hdac = 1 / (1 + s/wp_dac);

%% ==================== ANALYSIS MODE ====================

run_analysis = strcmpi(RUN_MODE, 'analysis') || strcmpi(RUN_MODE, 'both');
run_optimize = strcmpi(RUN_MODE, 'optimize') || strcmpi(RUN_MODE, 'both');

if run_analysis
    fprintf('\n========================================\n');
    fprintf('    ANALYSIS MODE\n');
    fprintf('========================================\n');

    % === ADC/DAC report ===
    fprintf('\n===== ADC / DAC MODEL SELECTION =====\n');
    fprintf('  USE_SMALL_SIGNAL_ADC_DAC = %d (%s)\n', USE_SMALL_SIGNAL_ADC_DAC, adc_dac_mode_text);
    fprintf('  G_ADC     = %.6e\n', G_ADC);
    fprintf('  G_DAC     = %.6e\n', G_DAC);
    fprintf('  G_adc_dac = %.6e\n', G_adc_dac);
    fprintf('  G_total   = %.6e\n', G_total);

    % === Plant at analysis load ===
    R = VOUT / Iload_analysis;

    wp_plant = 1/(C*R) + (Ts*(mc*D_tag - 0.5)/(Lind*C));
    wn_plant = pi/Ts;
    Qp_plant = 1/(pi*(mc*D_tag - 0.5));

    Hdc = (R/Ri) * (1 / (1 + R*Ts*(mc*D_tag - 0.5)/Lind));
    Fp  = (1 + s*C*Resr) / (1 + s/wp_plant);
    Fh  = 1 / (1 + s/(wn_plant*Qp_plant) + (s^2)/(wn_plant^2));

    Gvc = Hdc * Fp * Fh;

    % === Analog compensator (reference) ===
    R1_comp = sqrt(1 - 4*(fc^2)*(Ts^2) + 16*(fc^4)*(Ts^4));
    R2_comp = sqrt(1 + (39.48*(C^2)*(fc^2)*(Lind^2)*(R^2)) / ((Lind + 0.32*R*Ts)^2));

    wz_comp = 2*pi*fc/5;
    wp2_comp = 1/(C*Resr);
    wp1_comp = (1.23*fc*Ri*R1_comp*R2_comp*(Lind + 0.32*R*Ts)) / (Lind*R);

    Hc = (wp1_comp/s) * (1 + s/wz_comp) / (1 + s/wp2_comp);

    % === Programmed PID gains ===
    Kp_prog = Kp_target / G_digital;
    Ki_prog = Ki_target / G_digital;
    Kd_prog = Kd_target / G_digital;

    P_real = Kp_prog;
    I_real = Ki_prog * Tctrl;
    D_real = Kd_prog / Tctrl;

    P_cmd = round(P_real);
    I_cmd = round(I_real);
    D_cmd = round(D_real);

    [K1, K2, K3] = lab_position_to_k123(P_cmd, I_cmd, D_cmd);
    [Kp_prog_q, Ki_prog_q, Kd_prog_q] = lab_position_to_pid(P_cmd, I_cmd, D_cmd, Tctrl);

    Kp_ctrl_eff = Kp_prog_q * G_digital;
    Ki_ctrl_eff = Ki_prog_q * G_digital;
    Kd_ctrl_eff = Kd_prog_q * G_digital;

    fprintf('\nIdeal lab positions before rounding:\n');
    fprintf('  P_real = %.6f\n', P_real);
    fprintf('  I_real = %.6f\n', I_real);
    fprintf('  D_real = %.6f\n', D_real);

    fprintf('\nQuantized lab positions:\n');
    fprintf('  Position[0] (P) = %d\n', P_cmd);
    fprintf('  Position[1] (I) = %d\n', I_cmd);
    fprintf('  Position[2] (D) = %d\n', D_cmd);

    fprintf('\nRTL coefficients from quantized positions:\n');
    fprintf('  K1 = %d   (max: %d) -> %s\n', K1, K1_MAX, regcheck(K1, K1_MAX));
    fprintf('  K2 = %d   (max: %d) -> %s\n', K2, K2_MAX, regcheck(K2, K2_MAX));
    fprintf('  K3 = %d   (max: %d) -> %s\n', K3, K3_MAX, regcheck(K3, K3_MAX));

    fprintf('\nEffective controller gains after >>6:\n');
    fprintf('  Kp_ctrl_eff = %.6f\n', Kp_ctrl_eff);
    fprintf('  Ki_ctrl_eff = %.6e\n', Ki_ctrl_eff);
    fprintf('  Kd_ctrl_eff = %.6e\n', Kd_ctrl_eff);

    % === Stability checks ===
    Alpha = (sf - se)/(sn + se);
    if Alpha < 1
        disp('Alpha < 1: FB loop is stable.');
    else
        disp('Alpha >= 1: FB loop NOT stable.');
    end
    if se > 0.5*sf
        disp('Stable for all D (se > 0.5*sf).');
    else
        disp('Not stable for all D — increase slope compensation.');
    end

    % === Build digital controller ===
    switch lower(CONTROLLER_DOMAIN)
        case 'laplace'
            Hpid = Kp_prog_q + Ki_prog_q/s + (Kd_prog_q*s)/(Tf*s + 1);
            Hdig = minreal(G_ADC * Hadc * Hpid * G_digital * G_DAC * Hdac * Hblank * Delay);
            controller_title = 'Digital Controller in Laplace Domain';
            controller_phase_name = 'RTL PID(Laplace)';

        case 'z'
            Cz = tf([K1 K2 K3], [1 -1], Tctrl, 'Variable', 'z^-1');
            Hpid = d2c(G_digital * Cz, 'tustin');
            Hdig = minreal(G_ADC * Hadc * Hpid * G_DAC * Hdac * Hblank * Delay);
            controller_title = 'Digital Controller from Z Domain';
            controller_phase_name = 'RTL PID(Z)';

        otherwise
            error('CONTROLLER_DOMAIN must be ''laplace'' or ''z''.');
    end

    fprintf('\n===== CONTROLLER MODEL: %s =====\n', CONTROLLER_DOMAIN);

    % === Loop gains and margins ===
    Lloop = Hdig * Gvc * H;
    Tloop = feedback(Lloop, 1);
    Lc    = Hc * Gvc * H;
    Tc    = feedback(Lc, 1);

    [Gm_dig, Pm_dig, Wcg_dig, Wcp_dig] = margin(Lloop);
    [Gm_ana, Pm_ana, Wcg_ana, Wcp_ana] = margin(Lc);

    fprintf('\n===== STABILITY SUMMARY (Iload = %.3f A) =====\n', Iload_analysis);
    fprintf('Digital Loop:\n');
    fprintf('  Gain Margin:  %.1f dB at %.1f kHz\n', 20*log10(Gm_dig), Wcg_dig/(2*pi*1e3));
    fprintf('  Phase Margin: %.1f deg at %.1f kHz\n', Pm_dig, Wcp_dig/(2*pi*1e3));
    fprintf('Analog Loop:\n');
    fprintf('  Gain Margin:  %.1f dB at %.1f kHz\n', 20*log10(Gm_ana), Wcg_ana/(2*pi*1e3));
    fprintf('  Phase Margin: %.1f deg at %.1f kHz\n', Pm_ana, Wcp_ana/(2*pi*1e3));
    fprintf('Phase Margin Loss (dig vs ana): %.1f deg\n', Pm_ana - Pm_dig);

    % === Bode plots ===
    if PLOT_BODE_GRAPHS
        w = 2*pi*logspace(0, 8, 2000);

        figure; margin(Gvc, w);    grid on; title('Plant: Gvc = Vout/Vc');
        figure; margin(Hdig, w);   grid on; title('Digital Compensator Chain');
        figure; margin(Lloop, w);  grid on; title('Digital Loop Gain');

        opts = bodeoptions;
        opts.Grid = 'on';
        opts.PhaseWrapping = 'on';
        opts.PhaseWrappingBranch = -180;
        opts.FreqUnits = 'Hz';
        opts.XLimMode = 'manual';
        opts.XLim = [1 1e8];

        figure;
        bodeplot(Hc, Hpid, w, opts);
        title('Compensator: Analog Hc vs Digital Hpid');
        legend('Analog Hc', 'Digital Hpid', 'Location', 'best');

        figure;
        bodeplot(Lc, Lloop, w, opts);
        title('Loop Gain: Analog vs Digital');
        legend('Analog Loop', 'Digital Loop', 'Location', 'best');
    end

    % === Step response ===
    if PLOT_SMALL_SIGNAL_GRAPHS
        figure;
        subplot(2,1,1); step(Tloop); grid on; title('Digital Closed-Loop Step');
        subplot(2,1,2); step(Tc);    grid on; title('Analog Closed-Loop Step');
    end

    % === Phase budget ===
    fprintf('\n===== PHASE BUDGET AT fc=%.0f kHz =====\n', fc/1e3);
    blocks = {Hadc, Hpid, Hdac, Hblank, Delay, Gvc*H};
    names  = {'ADC', controller_phase_name, 'DAC', 'Blanking', 'CompDelay', 'Plant*H'};
    total_phase = 0;
    for k = 1:length(blocks)
        [~, ph] = bode(blocks{k}, 2*pi*fc);
        ph = squeeze(ph);
        fprintf('  %-12s: %+7.1f deg\n', names{k}, ph);
        total_phase = total_phase + ph;
    end
    fprintf('  %-12s: %+7.1f deg\n', 'TOTAL', total_phase);
    fprintf('  %-12s: %+7.1f deg\n', 'Phase Margin', total_phase + 180);

    % === Lab position report ===
    [P_lab, I_lab, D_lab] = k123_to_lab_position(K1, K2, K3);
    fprintf('\n===== LAB POSITION REPORT =====\n');
    fprintf('  K1=%d, K2=%d, K3=%d\n', K1, K2, K3);
    fprintf('  Position[0](P)=%d, Position[1](I)=%d, Position[2](D)=%d\n', ...
        round(P_lab), round(I_lab), round(D_lab));
    fprintf('  ChipTC.FullChip.Power.SetPIDCofficients(1, %d, %d, %d);\n', ...
        round(P_lab), round(I_lab), round(D_lab));

    % === Large-signal predictor ===
    if RUN_LARGE_SIGNAL_PREDICTOR
        ls_cfg = struct;
        ls_cfg.Ts     = Tctrl;   % [Fix #3] consistent Tctrl normalization
        ls_cfg.Tctrl  = Tctrl;
        ls_cfg.tstop  = LARGE_SIGNAL_TSTOP;

        ls_cfg.VOUT_NOM  = VOUT;
        ls_cfg.VOUT_INIT = LARGE_SIGNAL_VOUT_INIT;
        ls_cfg.beta      = beta;
        ls_cfg.VREF_FB   = beta * Vref;

        ls_cfg.K1 = K1;
        ls_cfg.K2 = K2;
        ls_cfg.K3 = K3;
        ls_cfg.SHIFT_BITS       = SHIFT_BITS;
        ls_cfg.PID_OUT_BITS     = PID_OUT_BITS;
        ls_cfg.PID_OUT_OFFSET   = PID_OUT_OFFSET;
        ls_cfg.PID_PRECLAMP_MIN = PID_PRECLAMP_MIN;
        ls_cfg.PID_PRECLAMP_MAX = PID_PRECLAMP_MAX;

        ls_cfg.N_ADC    = N_ADC;
        ls_cfg.VADC_MIN = VADC_MIN;
        ls_cfg.VADC_MAX = VADC_MAX;

        ls_cfg.N_DAC    = N_DAC;
        ls_cfg.V_FS_DAC = V_FS_DAC;
        ls_cfg.wp_dac   = wp_dac;
        ls_cfg.DAC_OUT_MIN = DAC_OUT_MIN;
        ls_cfg.DAC_OUT_MAX = DAC_OUT_MAX;

        ls_cfg.use_quantized_adc_dac = USE_QUANTIZED_ADC_DAC;
        ls_cfg.use_preclamp_state    = USE_PRECLAMP_STATE;

        [t_ls, vout_ls, e_hist, u_hist, pid_hist, vdac_hist] = run_large_signal_predictor(Gvc, ls_cfg);

        fprintf('\n===== LARGE-SIGNAL PREDICTOR =====\n');
        fprintf('  Tctrl:     %.3f ns (%.1f MHz)\n', Tctrl*1e9, 1/Tctrl/1e6);
        fprintf('  Init Vout: %.4f V\n', LARGE_SIGNAL_VOUT_INIT);
        fprintf('  Final Vout: %.4f V\n', vout_ls(end));
        fprintf('  Peak/Min Vout: %.4f / %.4f V\n', max(vout_ls), min(vout_ls));
        fprintf('  DAC range: %.4f – %.4f V\n', min(vdac_hist), max(vdac_hist));

        if PLOT_LARGE_SIGNAL_GRAPHS
            figure;
            subplot(5,1,1);
            plot(t_ls*1e6, vout_ls, 'LineWidth', 1.2); grid on;
            ylabel('Vout (V)'); title('Large-Signal Predictor'); ylim([0 0.9]);

            subplot(5,1,2);
            plot(t_ls*1e6, e_hist, 'LineWidth', 1.2); grid on;
            ylabel('Error');

            subplot(5,1,3);
            plot(t_ls*1e6, u_hist, 'LineWidth', 1.2); grid on;
            ylabel('u state');

            subplot(5,1,4);
            plot(t_ls*1e6, pid_hist, 'LineWidth', 1.2); grid on;
            ylabel('PID code');

            subplot(5,1,5);
            plot(t_ls*1e6, vdac_hist, 'LineWidth', 1.2); grid on;
            xlabel('Time (\mus)'); ylabel('DAC out (V)');
        end
    end
end  % run_analysis


%% ==================== OPTIMIZATION MODE ====================

if run_optimize
    fprintf('\n========================================\n');
    fprintf('    OPTIMIZATION MODE\n');
    fprintf('========================================\n');

    % === Build opt_cfg struct ===
    opt_cfg = struct();

    opt_cfg.Tctrl     = Tctrl;
    opt_cfg.Ts        = Ts;
    opt_cfg.Tf        = Tf;
    opt_cfg.fc        = fc;

    opt_cfg.G_digital = G_digital;
    opt_cfg.G_ADC     = G_ADC;
    opt_cfg.G_DAC     = G_DAC;

    opt_cfg.H      = H;
    opt_cfg.Hadc   = Hadc;
    opt_cfg.Hdac   = Hdac;
    opt_cfg.Hblank = Hblank;
    opt_cfg.Delay  = Delay;
    opt_cfg.controller_domain = CONTROLLER_DOMAIN;

    opt_cfg.VOUT = VOUT;
    opt_cfg.VIN  = VIN;
    opt_cfg.Lind = Lind;
    opt_cfg.C    = C;
    opt_cfg.Resr = Resr;
    opt_cfg.Ri   = Ri;
    opt_cfg.mc   = mc;
    opt_cfg.D_tag = D_tag;

    opt_cfg.Iload_vec     = OPT_I_LOAD_VEC;
    opt_cfg.Iload_weights = OPT_I_LOAD_WEIGHTS;

    opt_cfg.OPT_MIN_PM_DEG = OPT_MIN_PM_DEG;
    opt_cfg.OPT_MIN_GM_DB  = OPT_MIN_GM_DB;

    opt_cfg.OPT_W_PM_MIN  = OPT_W_PM_MIN;
    opt_cfg.OPT_W_GM_MIN  = OPT_W_GM_MIN;
    opt_cfg.OPT_W_PM_AVG  = OPT_W_PM_AVG;
    opt_cfg.OPT_W_GM_AVG  = OPT_W_GM_AVG;
    opt_cfg.OPT_W_FC      = OPT_W_FC;
    opt_cfg.OPT_W_SPREAD  = OPT_W_SPREAD;

    opt_cfg.OPT_PM_CAP    = OPT_PM_CAP;
    opt_cfg.OPT_GM_CAP_DB = OPT_GM_CAP_DB;
    opt_cfg.OPT_MAX_ITER  = OPT_MAX_ITER;

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
    opt_cfg.optimizer.method         = OPT_METHOD;
    opt_cfg.optimizer.random_seed    = OPT_RANDOM_SEED;
    opt_cfg.optimizer.nm_nstarts     = OPT_NM_NSTARTS;
    opt_cfg.optimizer.pso_swarm_size = OPT_PSO_SWARM_SIZE;
    opt_cfg.optimizer.pso_max_iter   = OPT_PSO_MAX_ITER;
    opt_cfg.optimizer.pso_use_hybrid = OPT_PSO_USE_HYBRID;

    opt_cfg.OPT_ENABLE_LOADSTEP_METRIC = OPT_ENABLE_LOADSTEP_METRIC;

    opt_cfg.OPT_LOADSTEP_DI_A       = OPT_LOADSTEP_DI_A;
    opt_cfg.OPT_LOADSTEP_TFINAL     = OPT_LOADSTEP_TFINAL;
    opt_cfg.OPT_LOADSTEP_NPTS       = OPT_LOADSTEP_NPTS;
    opt_cfg.OPT_LOADSTEP_ERR_BAND_V = OPT_LOADSTEP_ERR_BAND_V;

    opt_cfg.OPT_W_LOADSTEP_SLEW       = OPT_W_LOADSTEP_SLEW;
    opt_cfg.OPT_W_LOADSTEP_SETTLE     = OPT_W_LOADSTEP_SETTLE;
    opt_cfg.OPT_W_LOADSTEP_UNDERSHOOT = OPT_W_LOADSTEP_UNDERSHOOT;
    opt_cfg.OPT_W_LOADSTEP_OVERSHOOT  = OPT_W_LOADSTEP_OVERSHOOT;

    opt_cfg.OPT_LOADSTEP_SLEW_REF_VUS      = OPT_LOADSTEP_SLEW_REF_VUS;
    opt_cfg.OPT_LOADSTEP_SETTLE_REF_US     = OPT_LOADSTEP_SETTLE_REF_US;
    opt_cfg.OPT_LOADSTEP_UNDERSHOOT_REF_MV = OPT_LOADSTEP_UNDERSHOOT_REF_MV;
    opt_cfg.OPT_LOADSTEP_OVERSHOOT_REF_MV  = OPT_LOADSTEP_OVERSHOOT_REF_MV;

    opt_cfg.warm_start = OPT_WARM_START;

    % [Fix #8] Pre-compute plant transfer functions for all load points
    fprintf('Pre-computing plants for %d load points...', numel(OPT_I_LOAD_VEC));
    opt_cfg.Gvc_array = cell(1, numel(OPT_I_LOAD_VEC));
    for ii = 1:numel(OPT_I_LOAD_VEC)
        opt_cfg.Gvc_array{ii} = build_pcm_plant_at_iload(opt_cfg, OPT_I_LOAD_VEC(ii));
    end
    fprintf(' done.\n');

    % === Run optimizer ===
    opt_result = simple_optimize_pid(opt_cfg);

    % === Print results ===
    fprintf('\n===== OPTIMIZATION RESULT (%s) =====\n', opt_result.optimizer_method);

    fprintf('\nSuggested target gains:\n');
    fprintf('  Kp = %.6g,  Ki = %.6g,  Kd = %.6g\n', ...
        opt_result.Kp_target, opt_result.Ki_target, opt_result.Kd_target);

    fprintf('\nEffective gains after quantization + >>6:\n');
    fprintf('  Kp = %.6g,  Ki = %.6g,  Kd = %.6g\n', ...
        opt_result.Kp_ctrl_eff, opt_result.Ki_ctrl_eff, opt_result.Kd_ctrl_eff);

    fprintf('\nLab:  P=%d, I=%d, D=%d\n', opt_result.P_cmd, opt_result.I_cmd, opt_result.D_cmd);
    fprintf('RTL:  K1=%d (max %d), K2=%d (max %d), K3=%d (max %d)\n', ...
        opt_result.K1, K1_MAX, opt_result.K2, K2_MAX, opt_result.K3, K3_MAX);

    fprintf('\nMargins (worst / avg):\n');
    fprintf('  PM = %.2f / %.2f deg\n', opt_result.Pm_deg_min, opt_result.Pm_deg_avg);
    fprintf('  GM = %.2f / %.2f dB\n',  opt_result.Gm_dB_min,  opt_result.Gm_dB_avg);

    fprintf('\nLoad-step (worst case):\n');
    fprintf('  Slew=%.4f V/us, Settle=%.3f us, Under=%.3f mV, Over=%.3f mV\n', ...
        opt_result.LoadStepRecoverySlew_Vus_worst, ...
        opt_result.LoadStepSettle_us_worst, ...
        opt_result.LoadStepUndershoot_mV_worst, ...
        opt_result.LoadStepOvershoot_mV_worst);

    fprintf('\nPer-load detail:\n');
    for ii = 1:numel(opt_result.Iload_vec)
        fprintf('  I=%.3fA | GM=%6.2fdB | PM=%6.2fdeg | Fc=%8.2fkHz | Slew=%7.4f | Ts=%7.3fus | US=%7.3fmV | OS=%7.3fmV\n', ...
            opt_result.Iload_vec(ii), ...
            opt_result.Gm_dB_vec(ii), ...
            opt_result.Pm_deg_vec(ii), ...
            opt_result.Fc_kHz_vec(ii), ...
            opt_result.LoadStepRecoverySlew_Vus_vec(ii), ...
            opt_result.LoadStepSettle_us_vec(ii), ...
            opt_result.LoadStepUndershoot_mV_vec(ii), ...
            opt_result.LoadStepOvershoot_mV_vec(ii));
    end

    fprintf('\nProgramming:\n');
    fprintf('  ChipTC.FullChip.Power.SetPIDCofficients(1, %d, %d, %d);\n', ...
        opt_result.P_cmd, opt_result.I_cmd, opt_result.D_cmd);
end  % run_optimize


%% #################### LOCAL FUNCTIONS ####################
% [Fix #1] ALL local functions are below this line. No executable code after.

function result = simple_optimize_pid(cfg)
    method = lower(cfg.optimizer.method);
    switch method
        case {'nelder-mead','fminsearch','nm'}
            result = simple_optimize_pid_nm(cfg);
        case 'pso'
            result = simple_optimize_pid_pso(cfg);
        otherwise
            error('Unknown optimizer: %s', cfg.optimizer.method);
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

    % Build start points: centroid + warm-start + random
    n_warm = size(cfg.warm_start, 1);
    x0_list = zeros(1 + n_warm + nstarts, 3);
    x0_list(1,:) = 0.5 * (lb + ub);                       % centroid

    for k = 1:n_warm                                       % [Fix #19]
        ws = log10(cfg.warm_start(k,:));
        x0_list(1 + k, :) = max(lb, min(ub, ws));
    end

    rng(cfg.optimizer.random_seed);
    for k = 1:nstarts
        x0_list(1 + n_warm + k, :) = lb + rand(1,3) .* (ub - lb);
    end

    for k = 1:size(x0_list, 1)
        xtry = fminsearch(@(x) pid_margin_cost_multiload(x, cfg), x0_list(k,:), opts);
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
        error('particleswarm not available. Use ''nelder-mead'' or install Global Optimization Toolbox.');
    end

    lb = log10([cfg.OPT_KP_MIN, cfg.OPT_KI_MIN, cfg.OPT_KD_MIN]);
    ub = log10([cfg.OPT_KP_MAX, cfg.OPT_KI_MAX, cfg.OPT_KD_MAX]);

    % [Fix #19] Seed initial swarm with warm-start points
    n_warm = size(cfg.warm_start, 1);
    if n_warm > 0
        init_pts = zeros(n_warm, 3);
        for k = 1:n_warm
            init_pts(k,:) = max(lb, min(ub, log10(cfg.warm_start(k,:))));
        end
    else
        init_pts = [];
    end

    rng(cfg.optimizer.random_seed);

    pso_opts = optimoptions('particleswarm', ...
        'Display', ternary(cfg.SIMPLE_OPT_VERBOSE, 'iter', 'off'), ...
        'SwarmSize', cfg.optimizer.pso_swarm_size, ...
        'MaxIterations', cfg.optimizer.pso_max_iter, ...
        'FunctionTolerance', 1e-3);

    if ~isempty(init_pts)
        pso_opts.InitialSwarmMatrix = init_pts;
    end

    [xbest, best_cost] = particleswarm( ...
        @(x) pid_margin_cost_multiload(x, cfg), 3, lb, ub, pso_opts);

    if cfg.optimizer.pso_use_hybrid
        nm_opts = optimset( ...
            'Display', ternary(cfg.SIMPLE_OPT_VERBOSE, 'iter', 'off'), ...
            'MaxIter', cfg.OPT_MAX_ITER, ...
            'MaxFunEvals', 8*cfg.OPT_MAX_ITER, ...
            'TolX', 1e-3, 'TolFun', 1e-3);

        xbest_nm = fminsearch(@(x) pid_margin_cost_multiload(x, cfg), xbest, nm_opts);
        cost_nm  = pid_margin_cost_multiload(xbest_nm, cfg);
        if cost_nm < best_cost
            xbest = xbest_nm;
            best_cost = cost_nm;
        end
    end

    result = evaluate_pid_candidate_multiload(10.^xbest, cfg);
    result.best_cost = best_cost;
    result.optimizer_method = 'pso';
end

function cost = pid_margin_cost_multiload(xlog, cfg)
    vals = 10.^xlog;
    rep  = evaluate_pid_candidate_multiload(vals, cfg);

    % [Fix #18] Adaptive overflow penalty — scales with how far over the limit
    overflow_pen = 0;
    if abs(rep.K1) > cfg.K1_MAX
        overflow_pen = overflow_pen + 200 * (abs(rep.K1) / cfg.K1_MAX);
    end
    if abs(rep.K2) > cfg.K2_MAX
        overflow_pen = overflow_pen + 200 * (abs(rep.K2) / cfg.K2_MAX);
    end
    if abs(rep.K3) > cfg.K3_MAX
        overflow_pen = overflow_pen + 200 * (abs(rep.K3) / cfg.K3_MAX);
    end

    if ~rep.valid
        cost = 1e6 + overflow_pen;
        return;
    end

    pm_min_use = min(rep.Pm_deg_min, cfg.OPT_PM_CAP);
    gm_min_use = min(rep.Gm_dB_min,  cfg.OPT_GM_CAP_DB);
    pm_avg_use = min(rep.Pm_deg_avg, cfg.OPT_PM_CAP);
    gm_avg_use = min(rep.Gm_dB_avg,  cfg.OPT_GM_CAP_DB);

    fc_pen     = mean(abs(log10(max(rep.Wcp_rad_vec, 1) / (2*pi*cfg.fc))));
    spread_pen = std(rep.Pm_deg_vec) + 0.5*std(rep.Gm_dB_vec);

    score = cfg.OPT_W_PM_MIN * pm_min_use + ...
            cfg.OPT_W_GM_MIN * gm_min_use + ...
            cfg.OPT_W_PM_AVG * pm_avg_use + ...
            cfg.OPT_W_GM_AVG * gm_avg_use - ...
            cfg.OPT_W_FC     * fc_pen     - ...
            cfg.OPT_W_SPREAD * spread_pen;

    % Load-step terms
    if cfg.OPT_ENABLE_LOADSTEP_METRIC
        slew_bonus     = min(rep.LoadStepRecoverySlew_Vus_worst / max(cfg.OPT_LOADSTEP_SLEW_REF_VUS, eps), 10);
        settle_pen_ls  = rep.LoadStepSettle_us_worst       / max(cfg.OPT_LOADSTEP_SETTLE_REF_US, eps);
        under_pen      = rep.LoadStepUndershoot_mV_worst   / max(cfg.OPT_LOADSTEP_UNDERSHOOT_REF_MV, eps);
        over_pen       = rep.LoadStepOvershoot_mV_worst    / max(cfg.OPT_LOADSTEP_OVERSHOOT_REF_MV, eps);

        score = score + ...
            cfg.OPT_W_LOADSTEP_SLEW       * slew_bonus    - ...
            cfg.OPT_W_LOADSTEP_SETTLE     * settle_pen_ls - ...
            cfg.OPT_W_LOADSTEP_UNDERSHOOT * under_pen     - ...
            cfg.OPT_W_LOADSTEP_OVERSHOOT  * over_pen;
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

    % Build digital controller ONCE (same for all loads)
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

    % [Fix #7] Pre-compute Hdig * H once
    HdigH = Hdig_i * cfg.H;

    nL = numel(cfg.Iload_vec);
    Gm_dB_vec  = zeros(1, nL);
    Pm_deg_vec = zeros(1, nL);
    Wcp_rad_vec = zeros(1, nL);
    Fc_kHz_vec  = zeros(1, nL);

    LoadStepRecoverySlew_Vus_vec = zeros(1, nL);
    LoadStepSettle_us_vec        = inf(1, nL);
    LoadStepUndershoot_mV_vec    = inf(1, nL);
    LoadStepOvershoot_mV_vec     = inf(1, nL);
    LoadStepEvalRan_vec          = false(1, nL);

    for ii = 1:nL
        % [Fix #8] Use pre-computed plant if available
        if isfield(cfg, 'Gvc_array') && numel(cfg.Gvc_array) >= ii
            Gvc_i = cfg.Gvc_array{ii};
        else
            Gvc_i = build_pcm_plant_at_iload(cfg, cfg.Iload_vec(ii));
        end

        Lloop_i = HdigH * Gvc_i;

        [Gm, Pm, ~, Wcp] = margin(Lloop_i);

        margins_ok = ~(isempty(Gm) || isempty(Pm) || isempty(Wcp) || ...
                       isnan(Pm) || isnan(Wcp) || Wcp <= 0);

        if ~margins_ok
            valid = false;
            Gm_dB_vec(ii) = -Inf;
            Pm_deg_vec(ii) = -Inf;
            Wcp_rad_vec(ii) = NaN;
            Fc_kHz_vec(ii)  = NaN;
            continue;   % [Fix #10] skip remaining work
        end

        % [Fix #9] Only check stability via margins, skip minreal(feedback)
        if isinf(Gm)
            Gm_dB = cfg.OPT_GM_CAP_DB;
        else
            Gm_dB = 20*log10(max(Gm, eps));
        end

        Gm_dB_vec(ii)   = Gm_dB;
        Pm_deg_vec(ii)   = Pm;
        Wcp_rad_vec(ii)  = Wcp;
        Fc_kHz_vec(ii)   = Wcp / (2*pi*1e3);

        meets_targets = (Pm >= cfg.OPT_MIN_PM_DEG) && (Gm_dB >= cfg.OPT_MIN_GM_DB);
        if ~meets_targets
            valid = false;
        end

        % Load-step evaluation
        if cfg.OPT_ENABLE_LOADSTEP_METRIC && meets_targets
            ls = evaluate_loadstep_response_metric(cfg, Lloop_i, cfg.Iload_vec(ii));
            LoadStepRecoverySlew_Vus_vec(ii) = ls.recovery_slew_vus;
            LoadStepSettle_us_vec(ii)        = ls.settle_us;
            LoadStepUndershoot_mV_vec(ii)    = ls.undershoot_mv;
            LoadStepOvershoot_mV_vec(ii)     = ls.overshoot_mv;
            LoadStepEvalRan_vec(ii)          = true;
            if ~ls.valid, valid = false; end
        end
    end

    % Aggregate worst-case metrics
    good_pm = Pm_deg_vec(isfinite(Pm_deg_vec));
    good_gm = Gm_dB_vec(isfinite(Gm_dB_vec));

    if isempty(good_pm) || isempty(good_gm)
        valid = false;
        Pm_deg_min = -Inf; Gm_dB_min = -Inf;
        Pm_deg_avg = -Inf; Gm_dB_avg = -Inf;
    else
        % [Fix #16] Weighted average
        w = cfg.Iload_weights(isfinite(Pm_deg_vec));
        w = w / sum(w);
        Pm_deg_min = min(good_pm);
        Gm_dB_min  = min(good_gm);
        Pm_deg_avg = sum(w .* good_pm);
        Gm_dB_avg  = sum(w .* good_gm);
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

    rep.Iload_vec   = cfg.Iload_vec;
    rep.Gm_dB_vec   = Gm_dB_vec;
    rep.Pm_deg_vec  = Pm_deg_vec;
    rep.Wcp_rad_vec = Wcp_rad_vec;
    rep.Fc_kHz_vec  = Fc_kHz_vec;

    rep.Gm_dB_min  = Gm_dB_min;
    rep.Pm_deg_min = Pm_deg_min;
    rep.Gm_dB_avg  = Gm_dB_avg;
    rep.Pm_deg_avg = Pm_deg_avg;

    rep.LoadStepRecoverySlew_Vus_vec = LoadStepRecoverySlew_Vus_vec;
    rep.LoadStepSettle_us_vec        = LoadStepSettle_us_vec;
    rep.LoadStepUndershoot_mV_vec    = LoadStepUndershoot_mV_vec;
    rep.LoadStepOvershoot_mV_vec     = LoadStepOvershoot_mV_vec;
    rep.LoadStepEvalRan_vec          = LoadStepEvalRan_vec;

    fin_slew  = LoadStepRecoverySlew_Vus_vec(isfinite(LoadStepRecoverySlew_Vus_vec) & LoadStepEvalRan_vec);
    fin_settl = LoadStepSettle_us_vec(isfinite(LoadStepSettle_us_vec) & LoadStepEvalRan_vec);
    fin_under = LoadStepUndershoot_mV_vec(isfinite(LoadStepUndershoot_mV_vec) & LoadStepEvalRan_vec);
    fin_over  = LoadStepOvershoot_mV_vec(isfinite(LoadStepOvershoot_mV_vec) & LoadStepEvalRan_vec);

    rep.LoadStepRecoverySlew_Vus_worst = ternary(~isempty(fin_slew),  min(fin_slew),  0);
    rep.LoadStepSettle_us_worst        = ternary(~isempty(fin_settl), max(fin_settl), inf);
    rep.LoadStepUndershoot_mV_worst    = ternary(~isempty(fin_under), max(fin_under), inf);
    rep.LoadStepOvershoot_mV_worst     = ternary(~isempty(fin_over),  max(fin_over),  inf);
end

function Gvc_i = build_pcm_plant_at_iload(cfg, Iload)
    s = tf('s');
    Iload = max(Iload, 1e-6);   % guard against division by zero
    R  = cfg.VOUT / Iload;
    wp = 1/(cfg.C*R) + (cfg.Ts*(cfg.mc*cfg.D_tag - 0.5)/(cfg.Lind*cfg.C));
    wn = pi/cfg.Ts;
    Qp = 1/(pi*(cfg.mc*cfg.D_tag - 0.5));

    Hdc = (R/cfg.Ri) * (1 / (1 + R*cfg.Ts*(cfg.mc*cfg.D_tag - 0.5)/cfg.Lind));
    Fp  = (1 + s*cfg.C*cfg.Resr) / (1 + s/wp);
    Fh  = 1 / (1 + s/(wn*Qp) + (s^2)/(wn^2));

    Gvc_i = Hdc * Fp * Fh;
end

function ls = evaluate_loadstep_response_metric(cfg, Lloop_i, Iload)
    s = tf('s');

    ls = struct('valid', true, ...
                'recovery_slew_vus', 0, ...
                'settle_us', inf, ...
                'undershoot_mv', inf, ...
                'overshoot_mv', inf);      % [Fix #6]

    Iload = max(Iload, 1e-6);
    R = cfg.VOUT / Iload;

    % [Fix #5] Include inductor in output impedance
    ZL = s * cfg.Lind;
    Zc = cfg.Resr + 1/(s*cfg.C);
    Zrc = R * Zc / (R + Zc);          % R || Zc
    Zout_ol = minreal(ZL * Zrc / (ZL + Zrc));  % L series with (R || Zc)

    Gload_i = minreal(-Zout_ol / (1 + Lloop_i));

    t = linspace(0, cfg.OPT_LOADSTEP_TFINAL, cfg.OPT_LOADSTEP_NPTS);
    y = step(cfg.OPT_LOADSTEP_DI_A * Gload_i, t);
    y = squeeze(y);

    if isempty(y) || any(~isfinite(y))
        ls.valid = false;
        return;
    end

    y = y(:);
    t = t(:);

    % Undershoot = max negative deviation
    undershoot_v = max(0, -min(y));
    [~, idx_min] = min(y);

    % [Fix #6] Overshoot = max positive deviation (ring above nominal)
    overshoot_v = max(0, max(y));

    % [Fix #13] Average recovery slew: slope from min point to first band crossing
    band_v = max(cfg.OPT_LOADSTEP_ERR_BAND_V, 0.02 * max(abs(y)));

    if idx_min < numel(y)
        y_after = y(idx_min:end);
        t_after = t(idx_min:end);
        idx_cross = find(y_after > -band_v, 1, 'first');
        if ~isempty(idx_cross) && idx_cross > 1
            dt_rec = t_after(idx_cross) - t_after(1);
            dy_rec = y_after(idx_cross) - y_after(1);
            recovery_slew_vps = dy_rec / max(dt_rec, eps);
        else
            recovery_slew_vps = 0;
        end
    else
        recovery_slew_vps = 0;
    end

    % Settling time
    idx_last = find(abs(y) > band_v, 1, 'last');
    if isempty(idx_last)
        settle_s = 0;
    else
        settle_s = t(idx_last);
    end

    ls.recovery_slew_vus = recovery_slew_vps / 1e6;
    ls.settle_us         = settle_s * 1e6;
    ls.undershoot_mv     = undershoot_v * 1e3;
    ls.overshoot_mv      = overshoot_v * 1e3;
end

%% ==================== LARGE-SIGNAL PREDICTOR ====================

function [t, vout, e_hist, u_hist, pid_hist, vdac_hist] = run_large_signal_predictor(Gvc, cfg)
    sysd = c2d(ss(minreal(Gvc)), cfg.Tctrl, 'zoh');
    [Ad, Bd, Cd, Dd] = ssdata(sysd);

    N  = floor(cfg.tstop / cfg.Tctrl) + 1;
    nx = size(Ad, 1);

    t        = (0:N-1) * cfg.Tctrl;
    vout     = zeros(1, N);
    e_hist   = zeros(1, N);
    u_hist   = zeros(1, N);
    pid_hist = zeros(1, N);
    vdac_hist = zeros(1, N);

    dv0 = cfg.VOUT_INIT - cfg.VOUT_NOM;
    x   = zeros(nx, 1);
    if ~isempty(Cd)
        x = pinv(Cd) * dv0;
    end

    e1 = 0; e2 = 0; u_state = 0;

    pid_code_min = 0;
    pid_code_max = 2^(cfg.PID_OUT_BITS) - 1;
    pid_offset   = cfg.PID_OUT_OFFSET;
    pid_delta_min = -pid_offset;
    pid_delta_max = pid_code_max - pid_offset;

    u_min = cfg.PID_PRECLAMP_MIN;
    u_max = cfg.PID_PRECLAMP_MAX;

    dac_code_min = 0;
    dac_code_max = 2^(cfg.N_DAC) - 1;
    dac_offset   = 2^(cfg.N_DAC - 1);

    a_dac = exp(-cfg.wp_dac * cfg.Tctrl);
    b_dac = 1 - a_dac;

    v_dac_mid = cfg.DAC_OUT_MIN + ...
        (dac_offset / dac_code_max) * (cfg.DAC_OUT_MAX - cfg.DAC_OUT_MIN);
    v_dac_mid = min(max(v_dac_mid, cfg.DAC_OUT_MIN), cfg.DAC_OUT_MAX);
    v_dac_filt = v_dac_mid;

    for k = 1:N
        dvout = Cd*x;
        if ~isempty(Dd), dvout = dvout + Dd*0; end
        dvout   = double(dvout(1));
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
        pid_code  = pid_delta + pid_offset;
        pid_code  = min(max(pid_code, pid_code_min), pid_code_max);

        dac_delta = pid_code - pid_offset;
        dac_code  = dac_delta + dac_offset;
        dac_code  = min(max(dac_code, dac_code_min), dac_code_max);

        v_dac_cmd = cfg.DAC_OUT_MIN + ...
            (dac_code / dac_code_max) * (cfg.DAC_OUT_MAX - cfg.DAC_OUT_MIN);
        v_dac_cmd = min(max(v_dac_cmd, cfg.DAC_OUT_MIN), cfg.DAC_OUT_MAX);

        v_dac_filt = a_dac * v_dac_filt + b_dac * v_dac_cmd;

        u_plant = v_dac_filt - v_dac_mid;
        x = Ad*x + Bd*u_plant;

        e_hist(k)    = e0;
        u_hist(k)    = u_sat;
        pid_hist(k)  = pid_code;
        vdac_hist(k) = v_dac_filt;

        e2 = e1;
        e1 = e0;
    end
end

%% ==================== CONVERSION UTILITIES ====================

function [Kp_out, Ki_out, Kd_out] = k123_to_pid(K1, K2, K3, Ts)
    Kd_out = K3 * Ts;
    Kp_out = -K2 - 2*K3;
    Ki_out = (K1 + K2 + K3) / Ts;
end

function [K1_out, K2_out, K3_out] = pid_to_k123(Kp, Ki, Kd, Ts)
    K1_out = Kp + Ki*Ts + Kd/Ts;
    K2_out = -Kp - 2*Kd/Ts;
    K3_out = Kd/Ts;
end

function [Kp_prog, Ki_prog, Kd_prog] = lab_position_to_pid(P_in, I_in, D_in, Ts)
    [K1_tmp, K2_tmp, K3_tmp] = lab_position_to_k123(P_in, I_in, D_in);
    [Kp_prog, Ki_prog, Kd_prog] = k123_to_pid(K1_tmp, K2_tmp, K3_tmp, Ts);
end

function [Kp_eff, Ki_eff, Kd_eff] = lab_position_to_shifted_pid(P_in, I_in, D_in, Ts, G_digital)
    [Kp_prog, Ki_prog, Kd_prog] = lab_position_to_pid(P_in, I_in, D_in, Ts);
    Kp_eff = Kp_prog * G_digital;
    Ki_eff = Ki_prog * G_digital;
    Kd_eff = Kd_prog * G_digital;
end

function [K1_out, K2_out, K3_out] = lab_position_to_k123(P_in, I_in, D_in)
    K1_out = P_in + I_in + D_in;
    K2_out = -P_in - 2*D_in;
    K3_out = D_in;
end

function [P_out, I_out, D_out] = k123_to_lab_position(K1, K2, K3)
    D_out = K3;
    P_out = -K2 - 2*K3;
    I_out = K1 + K2 + K3;
end

function code = quantize_adc_code(vin, vmin, vmax, nbits)
    code_max = 2^nbits - 1;
    vin_clip = min(max(vin, vmin), vmax);
    code = round((vin_clip - vmin) / (vmax - vmin) * code_max);
    code = min(max(code, 0), code_max);
end

function result = regcheck(value, max_abs)
    if abs(round(value)) <= max_abs
        result = sprintf('OK (%.1f%%)', abs(value)/max_abs*100);
    else
        result = 'OVERFLOW!';
    end
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
