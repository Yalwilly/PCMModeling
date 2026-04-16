% filepath: c:\Users\yalwilly\OneDrive - Intel Corporation\Documents\yazn\work\Next Gen DC2DC\PCM Modeling\PCMModeling\PCMModeling\NGSTability_V5.m
%% ====== NGSTability_V5.m ======
% PCM Buck Converter Stability Analysis
% Cleaned model:
% - ADC gain = 1
% - DAC gain = 1
% - Digital scaling = 1/64 from RTL >>6
% - Controller modeled from RTL K1/K2/K3
% - Quantized coefficients included in the model

clear; close all; clc;
s = tf('s');

PLOT_BODE_GRAPHS = true;
PLOT_SMALL_SIGNAL_GRAPHS = true;
PLOT_LARGE_SIGNAL_GRAPHS = true;
CONTROLLER_DOMAIN = 'laplace';       % 'laplace' or 'z'
USE_SMALL_SIGNAL_ADC_DAC = false;
PHASE_YLIM = [-270 90];

SHIFT_BITS = 6;                % RTL >>6

RUN_LARGE_SIGNAL_PREDICTOR = true;
LARGE_SIGNAL_TSTOP = 300e-6;    % s
LARGE_SIGNAL_VOUT_INIT = 0.7;   % V, initial output for predictor
USE_QUANTIZED_ADC_DAC = true;   % exact ADC/DAC quantization in predictor
USE_PRECLAMP_STATE = true;      % true matches RTL that stores pre-clamp state
PID_OUT_BITS = 12;              % PID output width after >>6
PID_OUT_OFFSET = 2^(PID_OUT_BITS-1); % 2048 midscale offset, output range 0..4095
PID_PRECLAMP_MIN = -2048 * 2^SHIFT_BITS; % RTL MIN_VALUE = (-2048)<<6 = -131072
PID_PRECLAMP_MAX =  2047 * 2^SHIFT_BITS; % RTL MAX_VALUE = ( 2047)<<6 =  131008
PLOT_BINARY_CODE_AXIS = false;  % show PID/DAC code y-axis in binary

DAC_VDD = 1.8;                  % V
DAC_NMOS_VTH = 0.2;             % V
DAC_OUT_MIN = 0.0;              % V
DAC_OUT_MAX = DAC_VDD - DAC_NMOS_VTH; % NMOS-limited max

%% ==================== FEEDBACK NETWORK ====================
beta = 0.5;
H = beta;

%% ==================== CONVERTER PARAMETERS ====================
VOUT = 0.8;
VIN  = 3.3;
Lind = 0.47e-6;
Vref = 0.8;

fs   = 6e6;
Ts   = 1/fs;
fc   = fs/10;
Tf   = 1/(10*fc);
fdig = 256e6;
Tdig = 1/fdig;

Tctrl = Tdig;   % PID / RTL update period

%% ==================== CURRENT SENSING ====================
GI     = (1/640)*(10/64)*(1/4);
Rsense = 4e3;
Ri     = GI * Rsense;

%% ==================== DUTY CYCLE ====================
D     = VOUT/VIN;
D_tag = 1 - D;

%% ==================== SLOPE COMPENSATION ====================
se   = 0.7e6;
sn   = (VIN - VOUT)/Lind;
sf   = VOUT/Lind;
seff = Ri*(VIN - VOUT)/Lind + se;

Alpha = (sf - se)/(sn + se);
mc    = 1 + se/sn;

%% ==================== OUTPUT FILTER ====================
Iload= 100e-3;
R=VOUT/Iload ;
C    = 22e-6;
Resr = 3e-3;

%% ==================== LARGE-SIGNAL LOAD STEP PROFILE ====================
LOAD_STEP_ENABLE = true;
LOAD_STEP_I0     = Iload;      % A, nominal load current
LOAD_STEP_DI     = 1;        % A, load step magnitude
LOAD_STEP_DELAY  = 150e-6;      % s, delay before step starts
LOAD_STEP_RISE   = 100e-9;     % s, rising edge time
LOAD_STEP_WIDTH  = inf;        % s, inf = one-way step
LOAD_STEP_FALL   = 100e-9;     % s, falling edge time for finite pulse

%% ==================== PCM PLANT PARAMETERS ====================
wp = 1/(C*R) + (Ts*(mc*D_tag - 0.5)/(Lind*C));
wn = pi/Ts;
Qp = 1/(pi*(mc*D_tag - 0.5));

%% ==================== ANALOG COMPENSATOR (REFERENCE) ====================
R1 = sqrt(1 - 4*(fc^2)*(Ts^2) + 16*(fc^4)*(Ts^4));
R2 = sqrt(1 + (39.48*(C^2)*(fc^2)*(Lind^2)*(R^2)) / ((Lind + 0.32*R*Ts)^2));

wz_comp = 2*pi*fc/5;
wp2     = 1/(C*Resr);
wp1     = (1.23*fc*Ri*R1*R2*(Lind + 0.32*R*Ts)) / (Lind*R);

Hdc = (R/Ri) * (1 / (1 + R*Ts*(mc*D_tag - 0.5)/Lind));
Fp  = (1 + s*C*Resr) / (1 + s/wp);
Fh  = 1 / (1 + s/(wn*Qp) + (s^2)/(wn^2));
Hc  = (wp1/s) * (1 + s/wz_comp) / (1 + s/wp2);

%% ==================== RTL DIGITAL GAIN MODEL ====================
G_digital  = 2^(-SHIFT_BITS);   % RTL >>6

% ADC / DAC definitions
N_ADC    = 4;
VADC_MIN = 0.36;                % V
VADC_MAX = 0.44;                % V
V_FS_ADC = VADC_MAX - VADC_MIN; % 0.08 V

N_DAC    = 12;
V_FS_DAC = 1.2;                 % V

if USE_SMALL_SIGNAL_ADC_DAC
    % Small-signal linear gains
    G_ADC = (2^N_ADC) / V_FS_ADC;   % codes/V
    G_DAC = V_FS_DAC / (2^N_DAC);   % V/code
    adc_dac_mode_text = 'small-signal ADC/DAC gains';
else
    % Normalized unity gains
    G_ADC = 1;
    G_DAC = 1;
    adc_dac_mode_text = 'unity ADC/DAC gains';
end

G_adc_dac = G_ADC * G_DAC;          % interface gain only
G_total   = G_adc_dac * G_digital;  % full chain gain

fprintf('\n===== ADC / DAC MODEL SELECTION =====\n');
fprintf('  USE_SMALL_SIGNAL_ADC_DAC = %d (%s)\n', USE_SMALL_SIGNAL_ADC_DAC, adc_dac_mode_text);
fprintf('  G_ADC     = %.6e\n', G_ADC);
fprintf('  G_DAC     = %.6e\n', G_DAC);
fprintf('  G_adc_dac = %.6e\n', G_adc_dac);
fprintf('  G_total   = %.6e\n', G_total);

%% ==================== PID GAIN SETS (multi-set overlay) ====================
% Each row: {Kp_target, Ki_target, Kd_target, 'Label'}
% Add/remove rows to compare different gain configurations on the same plots.
PID_GAIN_SETS = {
    18.3028,    8e+06,       6.243896e-08, 'Set 1: Kp=18.3 Ki=8M';
    32.312500,  1.600000e+07, 5.712891e-08, 'Set 2: Kp=32.3 Ki=16M';
    50.515625,  2.000000e+07, 6.243896e-08, 'Set 3: Kp=50.5 Ki=20M';
};
LARGE_SIGNAL_SET_IDX = 1;  % which gain set to use for large-signal predictor
N_SETS = size(PID_GAIN_SETS, 1);

%% ==================== DIGITAL BLOCK PARAMETERS ====================
wp_dac  = 2*pi*500e3;
Td      = 8e-9;
Td_ADC  = 5e-9;
Tblank  = 20e-9;

%% ==================== STABILITY CHECKS ====================
if Alpha < 1
    disp('Alpha is less than 1 - the FB loop is stable');
else
    disp('Alpha is greater than 1 - the FB loop isnt stable');
end

if se > 0.5*sf
    disp('The buck converter stable for all D');
else
    disp('The buck converter isnt stable for all D - increase ma');
end

%% ==================== DIGITAL BLOCKS ====================
[numBlank, denBlank] = pade(Tblank, 2);
Hblank = tf(numBlank, denBlank);

[numD, denD] = pade(Td, 2);
Delay = tf(numD, denD);

[numADC, denADC] = pade(Td_ADC, 2);
Hadc = tf(numADC, denADC);

Hdac = 1 / (1 + s/wp_dac);

%% ==================== PLANT (gain-independent) ====================
Gvc   = minreal(Hdc * Fp * Fh);
Zc = Resr + 1/(s*C);
Rload_ls = VOUT / max(LOAD_STEP_I0, 1e-6);
Zout_ol = minreal((Zc * Rload_ls) / (Zc + Rload_ls));
Gload = minreal(-Zout_ol);   % dVout / dIload at the output node

Lc = Hc * Gvc * H;
Tc = feedback(Lc, 1);
[Gm_ana, Pm_ana, Wcg_ana, Wcp_ana] = margin(Lc);

%% ==================== PROCESS ALL PID GAIN SETS ====================
fprintf('\n===== CONTROLLER MODEL SELECTION =====\n');
fprintf('  CONTROLLER_DOMAIN = %s\n', CONTROLLER_DOMAIN);

Hpid_all   = cell(1, N_SETS);
Hdig_all   = cell(1, N_SETS);
Lloop_all  = cell(1, N_SETS);
Tloop_all  = cell(1, N_SETS);
K_all      = zeros(N_SETS, 3);   % K1, K2, K3
P_cmd_all  = zeros(N_SETS, 3);   % P_cmd, I_cmd, D_cmd
Pm_dig_all = zeros(1, N_SETS);
Gm_dig_all = zeros(1, N_SETS);
Wcg_dig_all = zeros(1, N_SETS);
Wcp_dig_all = zeros(1, N_SETS);
Kp_ctrl_eff_all = zeros(1, N_SETS);
Ki_ctrl_eff_all = zeros(1, N_SETS);
Kd_ctrl_eff_all = zeros(1, N_SETS);
labels     = cell(1, N_SETS);

for iset = 1:N_SETS
    Kp_target = PID_GAIN_SETS{iset, 1};
    Ki_target = PID_GAIN_SETS{iset, 2};
    Kd_target = PID_GAIN_SETS{iset, 3};
    labels{iset} = PID_GAIN_SETS{iset, 4};

    % Programmed gains before RTL >>6
    Kp_prog = Kp_target / G_digital;
    Ki_prog = Ki_target / G_digital;
    Kd_prog = Kd_target / G_digital;

    % Lab-style discrete P/I/D positions
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

    switch lower(CONTROLLER_DOMAIN)
        case 'laplace'
            Hpid_i = Kp_prog_q + Ki_prog_q/s + (Kd_prog_q*s)/(Tf*s + 1);
            Hdig_i = minreal(G_ADC * Hadc * Hpid_i * G_digital * G_DAC * Hdac * Hblank * Delay);
            controller_phase_name = 'RTL PID(Laplace)';
        case 'z'
            Cz_i = tf([K1 K2 K3], [1 -1], Tctrl, 'Variable', 'z^-1');
            Hpid_i = d2c(G_digital * Cz_i, 'tustin');
            Hdig_i = minreal(G_ADC * Hadc * Hpid_i * G_DAC * Hdac * Hblank * Delay);
            controller_phase_name = 'RTL PID(Z)';
        otherwise
            error('CONTROLLER_DOMAIN must be ''laplace'' or ''z''.');
    end

    Lloop_i = Hdig_i * Gvc * H;
    Tloop_i = feedback(Lloop_i, 1);

    [Gm_i, Pm_i, Wcg_i, Wcp_i] = margin(Lloop_i);

    % Store results
    Hpid_all{iset}   = Hpid_i;
    Hdig_all{iset}   = Hdig_i;
    Lloop_all{iset}  = Lloop_i;
    Tloop_all{iset}  = Tloop_i;
    K_all(iset,:)    = [K1 K2 K3];
    P_cmd_all(iset,:) = [P_cmd I_cmd D_cmd];
    Pm_dig_all(iset) = Pm_i;
    Gm_dig_all(iset) = Gm_i;
    Wcg_dig_all(iset) = Wcg_i;
    Wcp_dig_all(iset) = Wcp_i;
    Kp_ctrl_eff_all(iset) = Kp_ctrl_eff;
    Ki_ctrl_eff_all(iset) = Ki_ctrl_eff;
    Kd_ctrl_eff_all(iset) = Kd_ctrl_eff;

    fprintf('\n----- %s -----\n', labels{iset});
    fprintf('  Kp=%.4f  Ki=%.4e  Kd=%.4e\n', Kp_target, Ki_target, Kd_target);
    fprintf('  K1=%d  K2=%d  K3=%d\n', K1, K2, K3);
    fprintf('  Gain Margin:  %.1f dB at %.1f kHz\n', 20*log10(Gm_i), Wcg_i/(2*pi*1e3));
    fprintf('  Phase Margin: %.1f deg at %.1f kHz\n', Pm_i, Wcp_i/(2*pi*1e3));
    fprintf('  Crossover:    %.1f kHz\n', Wcp_i/(2*pi*1e3));
end

% Keep backward-compat scalars from the selected large-signal set
K1 = K_all(LARGE_SIGNAL_SET_IDX, 1);
K2 = K_all(LARGE_SIGNAL_SET_IDX, 2);
K3 = K_all(LARGE_SIGNAL_SET_IDX, 3);

fprintf('\n===== ADC/DAC/DIGITAL GAIN BUDGET =====\n');
fprintf('  Mode:                  %s\n', adc_dac_mode_text);
fprintf('  ADC gain:              %.6e\n', G_ADC);
fprintf('  Digital scaling (>>6): %.6e\n', G_digital);
fprintf('  DAC gain:              %.6e\n', G_DAC);
fprintf('  Total gain:            %.6e (%.1f dB)\n', G_total, 20*log10(G_total));

fprintf('\nAnalog Reference Loop:\n');
fprintf('  Gain Margin:  %.1f dB at %.1f kHz\n', 20*log10(Gm_ana), Wcg_ana/(2*pi*1e3));
fprintf('  Phase Margin: %.1f deg at %.1f kHz\n', Pm_ana, Wcp_ana/(2*pi*1e3));
fprintf('  Crossover:    %.1f kHz\n', Wcp_ana/(2*pi*1e3));

%% ==================== BODE PLOTS ====================
if PLOT_BODE_GRAPHS
    w = 2*pi*logspace(0, 8, 2000);

    % Plant & analog reference (gain-independent)
    figure; margin(Gvc, w);    grid on; title('Plant: Gvc = Vout/Vc');
    figure; margin(Hadc, w);   grid on; title('ADC: Hadc');
    figure; margin(Hc, w);     grid on; title('Analog Compensator Hc');
    figure; margin(Lc, w);     grid on; title('Analog Loop Gain');

    % Comparison plots with identical phase branch and limits
    opts = bodeoptions;
    opts.Grid = 'on';
    opts.PhaseWrapping = 'on';
    opts.PhaseWrappingBranch = -180;
    opts.FreqUnits = 'Hz';
    opts.XLimMode = 'manual';
    opts.XLim = [1 1e8];
    plot_colors = lines(N_SETS);

    % Build margin-annotated labels for closed-loop plots
    labels_margin = cell(1, N_SETS);
    for iset = 1:N_SETS
        labels_margin{iset} = sprintf('%s (PM=%.1f° GM=%.1fdB)', ...
            labels{iset}, Pm_dig_all(iset), 20*log10(Gm_dig_all(iset)));
    end
    label_ana_margin = sprintf('Analog (PM=%.1f° GM=%.1fdB)', Pm_ana, 20*log10(Gm_ana));

    % Digital Compensator Chain — all sets overlaid
    figure;
    bodeplot(Hdig_all{:}, w, opts);
    legend(labels, 'Location', 'best');
    title('Digital Compensator Chain — All Sets');

    % Digital Loop Gain — all sets overlaid
    figure;
    bodeplot(Lloop_all{:}, w, opts);
    legend(labels_margin, 'Location', 'best');
    title('Digital Loop Gain — All Sets');

    % PID Controller — all sets overlaid
    figure;
    bodeplot(Hpid_all{:}, w, opts);
    legend(labels, 'Location', 'best');
    title('PID Controller Hpid — All Sets');

    % Compensator comparison: Analog Hc vs all digital Hpid sets
    figure;
    bodeplot(Hc, Hpid_all{:}, w, opts);
    legend(['Analog Hc', labels], 'Location', 'best');
    title('Compensator Comparison: Analog Hc vs Digital Hpid Sets');

    % Loop gain comparison: Analog vs all digital sets
    figure;
    bodeplot(Lc, Lloop_all{:}, w, opts);
    legend([{label_ana_margin}, labels_margin], 'Location', 'best');
    title('Loop Gain Comparison: Analog vs Digital Sets');
end

%% ==================== STEP RESPONSE ====================
if PLOT_SMALL_SIGNAL_GRAPHS
    figure;
    subplot(2,1,1);
    step(Tloop_all{:});
    grid on;
    legend(labels_margin, 'Location', 'best');
    title('Digital Closed-Loop Step Response — All Sets');

    subplot(2,1,2);
    step(Tc);
    grid on;
    title('Analog Closed-Loop Step Response');
end

%% ==================== PHASE BUDGET ====================
for iset = 1:N_SETS
    fprintf('\n===== PHASE BUDGET AT CROSSOVER (%.0f kHz) — %s =====\n', fc/1e3, labels{iset});

    blocks = {Hadc, Hpid_all{iset}, Hdac, Hblank, Delay, Gvc*H};
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
end

%% ==================== NOTES ====================
fprintf('\n===== NOTES =====\n');
fprintf('1. This file models the RTL controller using K1/K2/K3.\n');
fprintf('2. The >>6 is represented explicitly as G_digital = 1/64.\n');
fprintf('3. ADC/DAC mode: %s.\n', adc_dac_mode_text);
fprintf('4. For AMS load-step correlation, a nonlinear cycle-by-cycle model is still recommended.\n');

%% ==================== LAB POSITION REPORT (all sets) ====================
for iset = 1:N_SETS
    K1_s = K_all(iset,1); K2_s = K_all(iset,2); K3_s = K_all(iset,3);
    [P_lab, I_lab, D_lab] = k123_to_lab_position(K1_s, K2_s, K3_s);
    [Kp_eff_from_rtl, Ki_eff_from_rtl, Kd_eff_from_rtl] = ...
        lab_position_to_shifted_pid(P_lab, I_lab, D_lab, Tctrl, G_digital);

    fprintf('\n===== LAB POSITION REPORT — %s =====\n', labels{iset});
    fprintf('  K1 = %d   K2 = %d   K3 = %d\n', K1_s, K2_s, K3_s);
    fprintf('  Position[0] (P) = %d\n', round(P_lab));
    fprintf('  Position[1] (I) = %d\n', round(I_lab));
    fprintf('  Position[2] (D) = %d\n', round(D_lab));
    fprintf('  Kp_eff = %.6f   Ki_eff = %.6e   Kd_eff = %.6e\n', ...
        Kp_eff_from_rtl, Ki_eff_from_rtl, Kd_eff_from_rtl);
    fprintf('  ChipTC.FullChip.Power.SetPIDCofficients(1, %d, %d, %d);\n', ...
        round(P_lab), round(I_lab), round(D_lab));
end

P_in = 1;
I_in = 1;
D_in = 1;

[K1_in, K2_in, K3_in] = lab_position_to_k123(P_in, I_in, D_in);
[Kp_eff_lab, Ki_eff_lab, Kd_eff_lab] = ...
    lab_position_to_shifted_pid(P_in, I_in, D_in, Tctrl, G_digital);

fprintf('\nUser-entered lab positions:\n');
fprintf('  Position[0] (P) = %d\n', P_in);
fprintf('  Position[1] (I) = %d\n', I_in);
fprintf('  Position[2] (D) = %d\n', D_in);

fprintf('\nCorresponding RTL coefficients:\n');
fprintf('  K1 = %d\n', K1_in);
fprintf('  K2 = %d\n', K2_in);
fprintf('  K3 = %d\n', K3_in);

fprintf('\nEffective controller PID from user-entered lab positions (Tctrl-based, after >>6):\n');
fprintf('  Kp_eff = %.6f\n', Kp_eff_lab);
fprintf('  Ki_eff = %.6e\n', Ki_eff_lab);
fprintf('  Kd_eff = %.6e\n', Kd_eff_lab);

%K1=3008;
%K2=-3940;
%K3=936;

%K1=4261;
%K2=-5279;
%K3=1023;


[P_real, I_real, D_real] = k123_to_pid(K1, K2, K3, Tctrl);

Kp_eff=P_real*G_digital;
Ki_eff=I_real*G_digital;
Kd_eff=D_real*G_digital;

fprintf('\nEffective controller PID from K1,K2,K3 (Tctrl-based, after >>6):\n');
fprintf('  Kp_eff = %.6f\n', Kp_eff);
fprintf('  Ki_eff = %.6e\n', Ki_eff);
fprintf('  Kd_eff = %.6e\n', Kd_eff);



%% ---------------------------------------------------------- %%%

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

function result = regcheck(value, max_abs)
    if abs(round(value)) <= max_abs
        result = sprintf('OK (%.1f%%)', abs(value)/max_abs*100);
    else
        result = 'OVERFLOW!';
    end
end

function force_bode_phase_ylim(figHandle, phase_ylim)
    ax = findall(figHandle, 'Type', 'axes');
    for k = 1:numel(ax)
        ylab = get(get(ax(k), 'YLabel'), 'String');

        if iscell(ylab)
            ylab = strjoin(ylab, ' ');
        end

        if ischar(ylab) || isstring(ylab)
            ylab_txt = lower(string(ylab));
            if contains(ylab_txt, "phase")
                ylim(ax(k), phase_ylim);
            end
        end
    end
end

function apply_binary_yaxis(ax, nbits)
    ylims = ylim(ax);
    y1 = ceil(ylims(1));
    y2 = floor(ylims(2));

    if y1 == y2
        ticks = y1;
    else
        ticks = unique(round(linspace(y1, y2, 7)));
    end

    labels = cell(size(ticks));
    for i = 1:numel(ticks)
        labels{i} = signed_code_to_bin(ticks(i), nbits);
    end

    set(ax, 'YTick', ticks, 'YTickLabel', labels);
end

function b = signed_code_to_bin(val, nbits)
    modval = mod(round(val), 2^nbits);
    b = dec2bin(modval, nbits);
end

%% ==================== LARGE-SIGNAL PREDICTOR ====================
% Uses gain set index LARGE_SIGNAL_SET_IDX (K1/K2/K3 already selected above)
if RUN_LARGE_SIGNAL_PREDICTOR
    fprintf('\n===== LARGE-SIGNAL PREDICTOR using %s =====\n', labels{LARGE_SIGNAL_SET_IDX});
    ls_cfg = struct;
    ls_cfg.Ts = Tctrl;           % keep K1/K2/K3 normalization on switching period
    ls_cfg.Tctrl = Tctrl;     % PID control-output refresh rate = 256 MHz
    ls_cfg.tstop = LARGE_SIGNAL_TSTOP;

    ls_cfg.VOUT_NOM = VOUT;
    ls_cfg.VOUT_INIT = LARGE_SIGNAL_VOUT_INIT;
    ls_cfg.beta = beta;
    ls_cfg.VREF_FB = beta * Vref;

    ls_cfg.K1 = K1;
    ls_cfg.K2 = K2;
    ls_cfg.K3 = K3;
    ls_cfg.SHIFT_BITS = SHIFT_BITS;
    ls_cfg.PID_OUT_BITS = PID_OUT_BITS;
    ls_cfg.PID_OUT_OFFSET = PID_OUT_OFFSET;
    ls_cfg.PID_PRECLAMP_MIN = PID_PRECLAMP_MIN;
    ls_cfg.PID_PRECLAMP_MAX = PID_PRECLAMP_MAX;

    ls_cfg.N_ADC = N_ADC;
    ls_cfg.VADC_MIN = VADC_MIN;
    ls_cfg.VADC_MAX = VADC_MAX;

    ls_cfg.N_DAC = N_DAC;
    ls_cfg.V_FS_DAC = V_FS_DAC;
    ls_cfg.wp_dac = wp_dac;
    ls_cfg.DAC_OUT_MIN = DAC_OUT_MIN;
    ls_cfg.DAC_OUT_MAX = DAC_OUT_MAX;

    ls_cfg.use_quantized_adc_dac = USE_QUANTIZED_ADC_DAC;
    ls_cfg.use_preclamp_state = USE_PRECLAMP_STATE;

    ls_cfg.load_step_enable = LOAD_STEP_ENABLE;
    ls_cfg.load_step_i0 = LOAD_STEP_I0;
    ls_cfg.load_step_di = LOAD_STEP_DI;
    ls_cfg.load_step_delay = LOAD_STEP_DELAY;
    ls_cfg.load_step_rise = LOAD_STEP_RISE;
    ls_cfg.load_step_width = LOAD_STEP_WIDTH;
    ls_cfg.load_step_fall = LOAD_STEP_FALL;
    
    [t, vout, e_hist, u_hist, pid_hist, vdac_hist, iload_hist] = ...
        run_large_signal_predictor(Gvc, Gload, ls_cfg);

    fprintf('\n===== LARGE-SIGNAL PREDICTOR =====\n');
    fprintf('  K1/K2/K3 normalization Ts: %.3f ns\n', ls_cfg.Ts*1e9);
    fprintf('  PID output refresh Tctrl:  %.3f ns (%.1f MHz)\n', ls_cfg.Tctrl*1e9, 1/ls_cfg.Tctrl/1e6);
    fprintf('  Initial Vout:      %.4f V\n', LARGE_SIGNAL_VOUT_INIT);
    fprintf('  Final Vout:        %.4f V\n', vout(end));
    fprintf('  Peak Vout:         %.4f V\n', max(vout));
    fprintf('  Min Vout:          %.4f V\n', min(vout));
    fprintf('  Peak error signal: %.1f\n', max(abs(e_hist)));
    fprintf('  Peak PID code:     %.1f\n', max(abs(pid_hist - PID_OUT_OFFSET)));
    fprintf('  DAC output range:  %.4f V to %.4f V\n', min(vdac_hist), max(vdac_hist));

    if ls_cfg.load_step_enable
        pre_window_s = 2e-6;
        post_window_s = 20e-6;
        idx_pre = (t >= max(0, ls_cfg.load_step_delay - pre_window_s)) & ...
                  (t < ls_cfg.load_step_delay);
        idx_post = (t >= ls_cfg.load_step_delay) & ...
                   (t <= min(ls_cfg.tstop, ls_cfg.load_step_delay + post_window_s));

        if any(idx_pre)
            vout_pre = mean(vout(idx_pre));
        else
            vout_pre = vout(1);
        end

        if any(idx_post)
            vout_min_post = min(vout(idx_post));
            [~, idx_local_min] = min(vout(idx_post));
            t_post = t(idx_post);
            t_min_post = t_post(idx_local_min);
        else
            vout_min_post = min(vout);
            t_min_post = t(1);
        end

        fprintf('  Load step Iload:   %.3f A -> %.3f A at %.1f us\n', ...
            ls_cfg.load_step_i0, ls_cfg.load_step_i0 + ls_cfg.load_step_di, ...
            ls_cfg.load_step_delay*1e6);
        fprintf('  Pre-step Vout:     %.4f V\n', vout_pre);
        fprintf('  Post-step min:     %.4f V at %.2f us\n', vout_min_post, t_min_post*1e6);
        fprintf('  Load-step dip:     %.2f mV\n', (vout_pre - vout_min_post)*1e3);
    end

    if PLOT_LARGE_SIGNAL_GRAPHS
        figure;
        subplot(6,1,1);
        plot(t*1e6, vout, 'LineWidth', 1.2);
        grid on;
        ylabel('Vout (V)');
        title('Large-Signal Predictor');
        ylim([0 0.9]);

        subplot(6,1,2);
        plot(t*1e6, e_hist, 'LineWidth', 1.2);
        grid on;
        ylabel('Error');

        subplot(6,1,3);
        plot(t*1e6, u_hist, 'LineWidth', 1.2);
        grid on;
        ylabel('u state');

        subplot(6,1,4);
        plot(t*1e6, pid_hist, 'LineWidth', 1.2);
        grid on;
        ylabel('PID code');
        if PLOT_BINARY_CODE_AXIS
            apply_binary_yaxis(gca, PID_OUT_BITS);
        end

        subplot(6,1,5);
        plot(t*1e6, vdac_hist, 'LineWidth', 1.2);
        grid on;
        ylabel('DAC out (V)');
        if PLOT_BINARY_CODE_AXIS
            apply_binary_yaxis(gca, N_DAC);
        end

        subplot(6,1,6);
        plot(t*1e6, iload_hist*1e3, 'LineWidth', 1.2);
        grid on;
        xlabel('Time (\mus)');
        ylabel('Iload (mA)');
        title('Load Step Profile');

        if ls_cfg.load_step_enable
            t_zoom_start = max(0, ls_cfg.load_step_delay - 5e-6);
            t_zoom_end = min(ls_cfg.tstop, ls_cfg.load_step_delay + 20e-6);
            idx_zoom = (t >= t_zoom_start) & (t <= t_zoom_end);

            if any(idx_zoom)
                figure;
                subplot(2,1,1);
                plot(t(idx_zoom)*1e6, vout(idx_zoom), 'LineWidth', 1.2);
                grid on;
                ylabel('Vout (V)');
                title('Load-Step Zoom');

                subplot(2,1,2);
                yyaxis left;
                plot(t(idx_zoom)*1e6, (vout(idx_zoom) - vout_pre)*1e3, 'LineWidth', 1.2);
                ylabel('dVout (mV)');
                yyaxis right;
                plot(t(idx_zoom)*1e6, iload_hist(idx_zoom)*1e3, 'LineWidth', 1.2);
                ylabel('Iload (mA)');
                grid on;
                xlabel('Time (\mus)');
            end
        end
    end
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

    % 500 kHz analog DAC filter, discretized at Tctrl
    a_dac = exp(-cfg.wp_dac * cfg.Tctrl);
    b_dac = 1 - a_dac;

    % Midscale analog output for zero small-signal injection
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

        % Unsigned sigma-delta / DAC code
        dac_delta = pid_code - pid_offset;
        dac_code  = dac_delta + dac_offset;
        dac_code  = min(max(dac_code, dac_code_min), dac_code_max);

        % Convert DAC code to analog voltage, NMOS-limited top end
        v_dac_cmd = cfg.DAC_OUT_MIN + ...
            (dac_code / dac_code_max) * (cfg.DAC_OUT_MAX - cfg.DAC_OUT_MIN);
        v_dac_cmd = min(max(v_dac_cmd, cfg.DAC_OUT_MIN), cfg.DAC_OUT_MAX);

        % Apply 500 kHz analog DAC filter
        v_dac_filt = a_dac * v_dac_filt + b_dac * v_dac_cmd;

        % Small-signal plant input is deviation from midscale analog DAC output
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

function [P_out, I_out, D_out] = k123_to_lab_position(K1, K2, K3)
    D_out = K3;
    P_out = -K2 - 2*K3;
    I_out = K1 + K2 + K3;
end

function [K1_out, K2_out, K3_out] = lab_position_to_k123(P_in, I_in, D_in)
    K1_out = P_in + I_in + D_in;
    K2_out = -P_in - 2*D_in;
    K3_out = D_in;
end