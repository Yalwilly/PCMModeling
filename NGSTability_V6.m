% filepath: c:\Users\yalwilly\OneDrive - Intel Corporation\Documents\yazn\work\Next Gen DC2DC\PCM Modeling\PCMModeling\PCMModeling\NGSTability_V6.m
%% ====== NGSTability_V6.m ======
% PCM Buck Converter Stability Analysis
% Cleaned model:
% - ADC gain = 1
% - DAC gain = 1
% - Digital scaling = 1/64 from RTL >>6
% - Controller modeled from RTL K1/K2/K3
% - Quantized coefficients included in the model

clear; close all; clc;
s = tf('s');

PLOT_BODE_GRAPHS = false;
PLOT_SMALL_SIGNAL_GRAPHS = false;
PLOT_LARGE_SIGNAL_GRAPHS = false;
CONTROLLER_DOMAIN = 'z';       % 'laplace' or 'z'
USE_SMALL_SIGNAL_ADC_DAC = false;
PHASE_YLIM = [-270 90];

SHIFT_BITS = 6;                % RTL >>6

RUN_LARGE_SIGNAL_PREDICTOR = true;
LARGE_SIGNAL_TSTOP = 100e-6;    % s
LARGE_SIGNAL_VOUT_INIT = 0.2;   % V, initial output for predictor
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
Rsense = 8e3;
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
Iload= 700e-3;
R=VOUT/Iload ;
C    = 22e-6;
Resr = 3e-3;

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

%% ==================== TARGET CONTROLLER PID GAINS ====================
% These are the desired controller gains after RTL >>6 only.
% ADC and DAC stay as separate transfer-function gains in Hdig.
Kp_target = 23.15;
Ki_target = 4e+06;
Kd_target = 6.24e-08;
  
% Programmed gains before RTL >>6
Kp_prog = Kp_target / G_digital;
Ki_prog = Ki_target / G_digital;
Kd_prog = Kd_target / G_digital;

% Convert desired programmed PID into lab-style discrete P/I/D positions
P_real = Kp_prog;
I_real = Ki_prog * Tctrl;
D_real = Kd_prog / Tctrl;

% Quantize at the position level, since this is what is effectively programmed
P_cmd = round(P_real);
I_cmd = round(I_real);
D_cmd = round(D_real);

% Convert quantized positions to RTL coefficients
[K1, K2, K3] = lab_position_to_k123(P_cmd, I_cmd, D_cmd);

% Recover actual programmed PID gains from the quantized positions
[Kp_prog_q, Ki_prog_q, Kd_prog_q] = lab_position_to_pid(P_cmd, I_cmd, D_cmd, Tctrl);

% Effective controller gains after RTL >>6 only
Kp_ctrl_eff = Kp_prog_q * G_digital;
Ki_ctrl_eff = Ki_prog_q * G_digital;
Kd_ctrl_eff = Kd_prog_q * G_digital;

% Optional: total path-scaled values for reporting only
Kp_path_eff = Kp_ctrl_eff * G_adc_dac;
Ki_path_eff = Ki_ctrl_eff * G_adc_dac;
Kd_path_eff = Kd_ctrl_eff * G_adc_dac;

fprintf('\nIdeal lab positions before rounding:\n');
fprintf('  P_real = %.6f\n', P_real);
fprintf('  I_real = %.6f\n', I_real);
fprintf('  D_real = %.6f\n', D_real);

fprintf('\nQuantized lab positions:\n');
fprintf('  Position[0] (P) = %d\n', P_cmd);
fprintf('  Position[1] (I) = %d\n', I_cmd);
fprintf('  Position[2] (D) = %d\n', D_cmd);

fprintf('\nRTL coefficients from quantized positions:\n');
fprintf('  K1 = %d   (20-bit max: 524287) -> %s\n', K1, regcheck(K1, 2^19-1));
fprintf('  K2 = %d   (20-bit max: 524287) -> %s\n', K2, regcheck(K2, 2^19-1));
fprintf('  K3 = %d   (11-bit max: 1023)   -> %s\n', K3, regcheck(K3, 2^10-1));

fprintf('\nRecovered programmed gains from quantized K1/K2/K3:\n');
fprintf('  Kp_prog_q = %.6f\n', Kp_prog_q);
fprintf('  Ki_prog_q = %.6e\n', Ki_prog_q);
fprintf('  Kd_prog_q = %.6e\n', Kd_prog_q);

fprintf('\nEffective controller gains after >>6 only:\n');
fprintf('  Kp_ctrl_eff = %.6f\n', Kp_ctrl_eff);
fprintf('  Ki_ctrl_eff = %.6e\n', Ki_ctrl_eff);
fprintf('  Kd_ctrl_eff = %.6e\n', Kd_ctrl_eff);

fprintf('\nPath-scaled values after ADC/DAC/>>6 (report only):\n');
fprintf('  Kp_path_eff = %.6e\n', Kp_path_eff);
fprintf('  Ki_path_eff = %.6e\n', Ki_path_eff);
fprintf('  Kd_path_eff = %.6e\n', Kd_path_eff);

%% ==================== DIGITAL BLOCK PARAMETERS ====================
wp_dac  = 2*pi*500e3;
Td      = 4e-9;
Td_ADC  = 5e-9;
Tblank  = 15e-9;
%% ==================== SIMPLE PID OPTIMIZATION SETTINGS ====================
RUN_SIMPLE_PID_OPT = true;      % suggestion only, does not overwrite active gains
SIMPLE_OPT_VERBOSE = true;

% Robust multi-load optimization corners
%OPT_I_LOAD_VEC = [0.001 0.10 0.30 0.50 0.70 0.90 1.10 2 3];   % A
OPT_I_LOAD_VEC = [0.01 0.4];   % A
% Hard stability requirements for every load corner
OPT_MIN_PM_DEG = 20;
OPT_MIN_GM_DB  = 3;

% Objective weights
OPT_W_PM_MIN   = 0.15;          % maximize worst-case PM
OPT_W_GM_MIN   = 0.1;          % maximize worst-case GM
OPT_W_PM_AVG   = 0.15;          % slight preference for good average PM
OPT_W_GM_AVG   = 0.10;          % slight preference for good average GM
OPT_W_FC       = 80;          % penalize crossover moving away from fc
OPT_W_SPREAD   = 0.20;          % penalize load sensitivity

OPT_PM_CAP     = 85;
OPT_GM_CAP_DB  = 40;
OPT_MAX_ITER   = 150;

OPT_KP_MIN = 0.015625;
OPT_KP_MAX = 2e2;
OPT_KI_MIN = 4e8;
OPT_KI_MAX = 5e8;
OPT_KD_MIN = 6.103516e-11;
OPT_KD_MAX = 6.250000e-08;
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

switch lower(CONTROLLER_DOMAIN)
    case 'laplace'
        % Continuous PID using recovered programmed gains
        Hpid = Kp_prog_q + Ki_prog_q/s + (Kd_prog_q*s)/(Tf*s + 1);

        % Complete digital chain
        Hdig = minreal(G_ADC * Hadc * Hpid * G_digital * G_DAC * Hdac * Hblank * Delay);
        controller_title = 'Digital Controller in Laplace Domain';
        controller_phase_name = 'RTL PID(Laplace)';

    case 'z'
        % Exact RTL controller in z-domain
        Cz = tf([K1 K2 K3], [1 -1], Tctrl, 'Variable', 'z^-1');

        % Convert only for plotting/continuous loop combination
        Hpid = d2c(G_digital * Cz, 'tustin');

        % G_digital already absorbed into Hpid
        Hdig = minreal(G_ADC * Hadc * Hpid * G_DAC * Hdac * Hblank * Delay);
        controller_title = 'Digital Controller from Z Domain';
        controller_phase_name = 'RTL PID(Z)';

    otherwise
        error('CONTROLLER_DOMAIN must be ''laplace'' or ''z''.');
end

fprintf('\n===== CONTROLLER MODEL SELECTION =====\n');
fprintf('  CONTROLLER_DOMAIN = %s\n', CONTROLLER_DOMAIN);

%% ==================== PLANT & LOOP GAINS ====================
Gvc   = minreal(Hdc * Fp * Fh);
Lloop = Hdig * Gvc * H;
Tloop = feedback(Lloop, 1);

Lc = Hc * Gvc * H;
Tc = feedback(Lc, 1);

%% ==================== STABILITY MARGINS ====================
[Gm_dig, Pm_dig, Wcg_dig, Wcp_dig] = margin(Lloop);
[Gm_ana, Pm_ana, Wcg_ana, Wcp_ana] = margin(Lc);

fprintf('\n===== ADC/DAC/DIGITAL GAIN BUDGET =====\n');
fprintf('  Mode:                  %s\n', adc_dac_mode_text);
fprintf('  ADC gain:              %.6e\n', G_ADC);
fprintf('  Digital scaling (>>6): %.6e\n', G_digital);
fprintf('  DAC gain:              %.6e\n', G_DAC);
fprintf('  Total gain:            %.6e (%.1f dB)\n', G_total, 20*log10(G_total));

fprintf('\n===== STABILITY SUMMARY =====\n');
fprintf('Digital Loop:\n');
fprintf('  Gain Margin:  %.1f dB at %.1f kHz\n', 20*log10(Gm_dig), Wcg_dig/(2*pi*1e3));
fprintf('  Phase Margin: %.1f deg at %.1f kHz\n', Pm_dig, Wcp_dig/(2*pi*1e3));
fprintf('  Crossover:    %.1f kHz\n', Wcp_dig/(2*pi*1e3));

fprintf('Analog Loop:\n');
fprintf('  Gain Margin:  %.1f dB at %.1f kHz\n', 20*log10(Gm_ana), Wcg_ana/(2*pi*1e3));
fprintf('  Phase Margin: %.1f deg at %.1f kHz\n', Pm_ana, Wcp_ana/(2*pi*1e3));
fprintf('  Crossover:    %.1f kHz\n', Wcp_ana/(2*pi*1e3));

fprintf('Phase Margin Loss (digital vs analog): %.1f deg\n', Pm_ana - Pm_dig);

%% ==================== BODE PLOTS ====================
if PLOT_BODE_GRAPHS
    w = 2*pi*logspace(0, 8, 2000);

    % Original plot windows
    figure; margin(Gvc, w);    grid on; title('Plant: Gvc = Vout/Vc');
    figure; margin(Hadc, w);   grid on; title('ADC: Hadc');
    figure; margin(Hpid, w);   grid on; title(controller_title);
    figure; margin(Hdig, w);   grid on; title('Digital Compensator Chain');
    figure; margin(Hc, w);     grid on; title('Analog Compensator Hc');
    figure; margin(Lc, w);     grid on; title('Analog Loop Gain');
    figure; margin(Lloop, w);  grid on; title('Digital Loop Gain');

    % Comparison plots with identical phase branch and limits
    opts = bodeoptions;
    opts.Grid = 'on';
    opts.PhaseWrapping = 'on';
    opts.PhaseWrappingBranch = -180;
    opts.FreqUnits = 'Hz';
    opts.XLimMode = 'manual';
    opts.XLim = [1 1e8];

    figure;
    bodeplot(Hc, Hpid, w, opts);
    title('Compensator Comparison: Analog Hc vs Digital Hpid');
    legend('Analog Hc', 'Digital Hpid', 'Location', 'best');
    % force_bode_phase_ylim(gcf, PHASE_YLIM);

    figure;
    bodeplot(Lc, Lloop, w, opts);
    title('Loop Gain Comparison: Analog vs Digital');
    legend('Analog Loop', 'Digital Loop', 'Location', 'best');
    % force_bode_phase_ylim(gcf, PHASE_YLIM);
end

%% ==================== STEP RESPONSE ====================
if PLOT_SMALL_SIGNAL_GRAPHS
    figure;
    subplot(2,1,1);
    step(Tloop);
    grid on;
    title('Digital Closed-Loop Step Response');

    subplot(2,1,2);
    step(Tc);
    grid on;
    title('Analog Closed-Loop Step Response');
end

%% ==================== PHASE BUDGET ====================
fprintf('\n===== PHASE BUDGET AT CROSSOVER (%.0f kHz) =====\n', fc/1e3);

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

%% ==================== NOTES ====================
fprintf('\n===== NOTES =====\n');
fprintf('1. This file models the RTL controller using K1/K2/K3.\n');
fprintf('2. The >>6 is represented explicitly as G_digital = 1/64.\n');
fprintf('3. ADC/DAC mode: %s.\n', adc_dac_mode_text);
fprintf('4. For AMS load-step correlation, a nonlinear cycle-by-cycle model is still recommended.\n');

%% ==================== LAB POSITION REPORT ====================
[P_lab, I_lab, D_lab] = k123_to_lab_position(K1, K2, K3);
[Kp_eff_from_rtl, Ki_eff_from_rtl, Kd_eff_from_rtl] = ...
    lab_position_to_shifted_pid(P_lab, I_lab, D_lab, Tctrl, G_digital);

fprintf('\n===== LAB POSITION REPORT =====\n');
fprintf('From current RTL coefficients:\n');
fprintf('  K1 = %d\n', K1);
fprintf('  K2 = %d\n', K2);
fprintf('  K3 = %d\n', K3);

fprintf('\nRecovered lab positions:\n');
fprintf('  Position[0] (P) = %d\n', round(P_lab));
fprintf('  Position[1] (I) = %d\n', round(I_lab));
fprintf('  Position[2] (D) = %d\n', round(D_lab));

fprintf('\nEffective controller PID from recovered lab positions (Tctrl-based, after >>6):\n');
fprintf('  Kp_eff = %.6f\n', Kp_eff_from_rtl);
fprintf('  Ki_eff = %.6e\n', Ki_eff_from_rtl);
fprintf('  Kd_eff = %.6e\n', Kd_eff_from_rtl);

fprintf('\nProgramming line using lab positions:\n');
fprintf('  ChipTC.FullChip.Power.SetPIDCofficients(1, %d, %d, %d);\n', ...
    round(P_lab), round(I_lab), round(D_lab));

P_in = 1;
I_in = 1;
D_in = 1024;

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
if RUN_LARGE_SIGNAL_PREDICTOR
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

    [t, vout, e_hist, u_hist, pid_hist, vdac_hist] = run_large_signal_predictor(Gvc, ls_cfg);

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

    if PLOT_LARGE_SIGNAL_GRAPHS
        figure;
        subplot(5,1,1);
        plot(t*1e6, vout, 'LineWidth', 1.2);
        grid on;
        ylabel('Vout (V)');
        title('Large-Signal Predictor');

        subplot(5,1,2);
        plot(t*1e6, e_hist, 'LineWidth', 1.2);
        grid on;
        ylabel('Error');

        subplot(5,1,3);
        plot(t*1e6, u_hist, 'LineWidth', 1.2);
        grid on;
        ylabel('u state');

        subplot(5,1,4);
        plot(t*1e6, pid_hist, 'LineWidth', 1.2);
        grid on;
        ylabel('PID code');
        if PLOT_BINARY_CODE_AXIS
            apply_binary_yaxis(gca, PID_OUT_BITS);
        end

        subplot(5,1,5);
        plot(t*1e6, vdac_hist, 'LineWidth', 1.2);
        grid on;
        xlabel('Time (\mus)');
        ylabel('DAC out (V)');
        if PLOT_BINARY_CODE_AXIS
            apply_binary_yaxis(gca, N_DAC);
        end
    end
end

function [t, vout, e_hist, u_hist, pid_hist, vdac_hist] = run_large_signal_predictor(Gvc, cfg)
    sysd = c2d(ss(minreal(Gvc)), cfg.Tctrl, 'zoh');
    [Ad, Bd, Cd, Dd] = ssdata(sysd);

    N  = floor(cfg.tstop / cfg.Tctrl) + 1;
    nx = size(Ad, 1);

    t = (0:N-1) * cfg.Tctrl;
    vout = zeros(1, N);
    e_hist = zeros(1, N);
    u_hist = zeros(1, N);
    pid_hist = zeros(1, N);
    vdac_hist = zeros(1, N);

    dv0 = cfg.VOUT_INIT - cfg.VOUT_NOM;
    x = zeros(nx, 1);
    if ~isempty(Cd)
        x = pinv(Cd) * dv0;
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
        dvout = Cd*x;
        if ~isempty(Dd)
            dvout = dvout + Dd*0;
        end
        dvout = double(dvout(1));
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

        x = Ad*x + Bd*u_plant;

        e_hist(k) = e0;
        u_hist(k) = u_sat;
        pid_hist(k) = pid_code;
        vdac_hist(k) = v_dac_filt;

        e2 = e1;
        e1 = e0;
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

%% ==================== SIMPLE PID OPTIMIZATION ====================
if RUN_SIMPLE_PID_OPT
    opt_cfg = struct;
    opt_cfg.Kp0 = Kp_target;
    opt_cfg.Ki0 = Ki_target;
    opt_cfg.Kd0 = Kd_target;

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

    % Plant/build parameters for load sweep
    opt_cfg.VOUT = VOUT;
    opt_cfg.Lind = Lind;
    opt_cfg.C = C;
    opt_cfg.Resr = Resr;
    opt_cfg.Ri = Ri;
    opt_cfg.mc = mc;
    opt_cfg.D_tag = D_tag;

    % Multi-load settings
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

    opt_cfg.SIMPLE_OPT_VERBOSE = SIMPLE_OPT_VERBOSE;

    opt_result = simple_optimize_pid(opt_cfg);

    fprintf('\n===== ROBUST MULTI-ILOAD PID OPTIMIZATION RESULT =====\n');
    fprintf('Suggested targets:\n');
    fprintf('  Kp_target = %.6g\n', opt_result.Kp_target);
    fprintf('  Ki_target = %.6g\n', opt_result.Ki_target);
    fprintf('  Kd_target = %.6g\n', opt_result.Kd_target);

    fprintf('\nSuggested lab positions:\n');
    fprintf('  P = %d\n', opt_result.P_cmd);
    fprintf('  I = %d\n', opt_result.I_cmd);
    fprintf('  D = %d\n', opt_result.D_cmd);

    fprintf('\nSuggested RTL coefficients:\n');
    fprintf('  K1 = %d\n', opt_result.K1);
    fprintf('  K2 = %d\n', opt_result.K2);
    fprintf('  K3 = %d\n', opt_result.K3);

    fprintf('\nWorst-case margins across Iload sweep:\n');
    fprintf('  Gain Margin:  %.2f dB\n', opt_result.Gm_dB_min);
    fprintf('  Phase Margin: %.2f deg\n', opt_result.Pm_deg_min);

    fprintf('\nAverage margins across Iload sweep:\n');
    fprintf('  Gain Margin:  %.2f dB\n', opt_result.Gm_dB_avg);
    fprintf('  Phase Margin: %.2f deg\n', opt_result.Pm_deg_avg);

    fprintf('\nPer-load summary:\n');
    for ii = 1:numel(opt_result.Iload_vec)
        fprintf('  Iload = %.3f A  |  GM = %6.2f dB  PM = %6.2f deg  Fc = %8.2f kHz\n', ...
            opt_result.Iload_vec(ii), opt_result.Gm_dB_vec(ii), ...
            opt_result.Pm_deg_vec(ii), opt_result.Fc_kHz_vec(ii));
    end

    fprintf('\nProgramming line:\n');
    fprintf('  ChipTC.FullChip.Power.SetPIDCofficients(1, %d, %d, %d);\n', ...
        opt_result.P_cmd, opt_result.I_cmd, opt_result.D_cmd);
end

function result = simple_optimize_pid(cfg)
    x0 = log10([cfg.Kp0, cfg.Ki0, max(cfg.Kd0, cfg.OPT_KD_MIN)]);

    opts = optimset( ...
        'Display', ternary(cfg.SIMPLE_OPT_VERBOSE, 'iter', 'off'), ...
        'MaxIter', cfg.OPT_MAX_ITER, ...
        'MaxFunEvals', 8*cfg.OPT_MAX_ITER, ...
        'TolX', 1e-3, ...
        'TolFun', 1e-3);

    xbest = fminsearch(@(x) pid_margin_cost_multiload(x, cfg), x0, opts);
    result = evaluate_pid_candidate_multiload(10.^xbest, cfg);
end

function cost = pid_margin_cost_multiload(xlog, cfg)
    vals = 10.^xlog;
    rep = evaluate_pid_candidate_multiload(vals, cfg);

    overflow_pen = 0;
    if abs(rep.K1) > 2^19-1, overflow_pen = overflow_pen + 200; end
    if abs(rep.K2) > 2^19-1, overflow_pen = overflow_pen + 200; end
    if abs(rep.K3) > 2^10-1, overflow_pen = overflow_pen + 200; end

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

    switch lower(cfg.controller_domain)
        case 'laplace'
            Hpid = Kp_prog_q + Ki_prog_q/s + (Kd_prog_q*s)/(cfg.Tf*s + 1);
            Hdig = minreal(cfg.G_ADC * cfg.Hadc * Hpid * cfg.G_digital * cfg.G_DAC * cfg.Hdac * cfg.Hblank * cfg.Delay);

        case 'z'
            Cz = tf([K1 K2 K3], [1 -1], cfg.Tctrl, 'Variable', 'z^-1');
            Hpid = d2c(cfg.G_digital * Cz, 'tustin');
            Hdig = minreal(cfg.G_ADC * cfg.Hadc * Hpid * cfg.G_DAC * cfg.Hdac * cfg.Hblank * cfg.Delay);

        otherwise
            error('Unsupported controller domain.');
    end

    nL = numel(cfg.Iload_vec);
    Gm_dB_vec = zeros(1, nL);
    Pm_deg_vec = zeros(1, nL);
    Wcp_rad_vec = zeros(1, nL);
    Fc_kHz_vec = zeros(1, nL);

    for ii = 1:nL
        Gvc_i = build_pcm_plant_at_iload(cfg, cfg.Iload_vec(ii));
        Lloop_i = Hdig * Gvc_i * cfg.H;

        [Gm, Pm, ~, Wcp] = margin(Lloop_i);

        if isempty(Gm) || isempty(Pm) || isempty(Wcp) || isnan(Pm) || isnan(Wcp) || Wcp <= 0
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

        if Pm < cfg.OPT_MIN_PM_DEG || Gm_dB < cfg.OPT_MIN_GM_DB
            valid = false;
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

    rep = struct;
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

    rep.Iload_vec = cfg.Iload_vec;
    rep.Gm_dB_vec = Gm_dB_vec;
    rep.Pm_deg_vec = Pm_deg_vec;
    rep.Wcp_rad_vec = Wcp_rad_vec;
    rep.Fc_kHz_vec = Fc_kHz_vec;

    rep.Gm_dB_min = Gm_dB_min;
    rep.Pm_deg_min = Pm_deg_min;
    rep.Gm_dB_avg = Gm_dB_avg;
    rep.Pm_deg_avg = Pm_deg_avg;
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

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end