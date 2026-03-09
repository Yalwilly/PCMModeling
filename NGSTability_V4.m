%% ====== NGSTability_V4.m ======
% PCM Buck Converter Stability Analysis
% Compares digital compensator (Hdig) vs. analog Type-II (Hc)
% Author: Yazn Al-Willy
% Date:   2026-03-08
clear; close all; clc;
s = tf('s');

%% ==================== FEEDBACK NETWORK ====================
beta = 0.5;     % Vfb divider
H = beta;

%% ==================== CONVERTER PARAMETERS ====================
VOUT = 0.8;         % V
VIN  = 3.3;         % V
Lind = 0.47e-6;     % H  (renamed to avoid shadowing by loop gain)
Vref = 0.8;         % V

fs   = 6e6;         % switching frequency (Hz)
Ts   = 1/fs;        % switching period (s)
fc   = fs/10;       % target crossover frequency (Hz)
Tf   = 1/(10*fc);   % derivative filter time constant (s)
fdig = 256e6;       % digital clock frequency (Hz)
Tdig = 1/fdig;      % digital clock period (s)

%% ==================== CURRENT SENSING ====================
GI     = (1/640)*(10/64)*(1/4);   % Current gain chain
Rsense = 8e3;                      % Sense resistance (Ohm)
Ri     = GI * Rsense;              % Total current sense gain

%% ==================== DUTY CYCLE ====================
D     = VOUT/VIN;
D_tag = 1 - D;

%% ==================== SLOPE COMPENSATION ====================
se   = 0.7e6;                       % Artificial ramp (V/s)
sn   = (VIN - VOUT)/Lind;           % Inductor on-slope (A/s)
sf   = VOUT/Lind;                    % Inductor off-slope (A/s)
seff = Ri*(VIN - VOUT)/Lind + se;   % Effective Vramp slope (V/s)

Alpha = (sf - se)/(sn + se);
mc    = 1 + se/sn;                   % Compensation ratio

%% ==================== OUTPUT FILTER ====================
R    = 1e-3;        % Load resistance (Ohm)
C    = 22e-6;       % Output capacitance (F)
Resr = 10e-3;       % ESR (Ohm)

%% ==================== PCM PLANT PARAMETERS ====================
wp = 1/(C*R) + (Ts*(mc*D_tag - 0.5)/(Lind*C));   % dominant pole (rad/s)
wn = pi/Ts;                                        % sampling double-pole freq (rad/s)
Qp = 1/(pi*(mc*D_tag - 0.5));                      % quality factor of Fh

%% ==================== ANALOG COMPENSATOR (Hc) - TYPE II ====================
R1 = sqrt( 1 - 4*(fc^2)*(Ts^2) + 16*(fc^4)*(Ts^4) );
R2 = sqrt( 1 + (39.48*(C^2)*(fc^2)*(Lind^2)*(R^2)) / ( (Lind + 0.32*R*Ts)^2 ) );

wz_comp = 2*pi*fc/5;              % compensator zero (rad/s)
wp2     = 1/(C*Resr);             % compensator pole — cancels ESR zero (rad/s)
wp1     = (1.23*fc*Ri*R1*R2*(Lind + 0.32*R*Ts)) / (Lind*R);   % integrator gain

% FIX: Hdc parenthesis bug — was (1/1 + ...) instead of 1/(1 + ...)
Hdc = (R/Ri) * (1 / (1 + R*Ts*(mc*D_tag - 0.5)/Lind));
Fp  = (1 + s*C*Resr) / (1 + s/wp);
Fh  = 1 / (1 + s/(wn*Qp) + (s^2)/(wn^2));

Hc = (wp1/s) * (1 + s/wz_comp) / (1 + s/wp2);

%% ==================== ESR ZERO (for reference) ====================
wz_esr = 1/(C*Resr);              % ESR zero frequency (rad/s)

%% ==================== DIGITAL PID GAINS ====================
Kp = 50;       % Proportional gain
Ki = 80e07;    % Integral gain (continuous, units: 1/s)
Kd = 0;        % Derivative gain (continuous, units: s)

% Discrete PID coefficients (velocity form, backward Euler)
% FIX: K2 and K3 must include /Ts scaling
K1 = Kp + Ki*Ts + Kd/Ts;    % = Kp + Ki*Ts + Kd/Ts
K2 = -Kp - 2*Kd/Ts;         % FIX: was -Kp - 2*Kd (missing /Ts)
K3 = Kd/Ts;                 % FIX: was Kd (missing /Ts)

fprintf('\n===== DISCRETE PID COEFFICIENTS =====\n');
fprintf('K1 = Kp + Ki*Ts + Kd/Ts = %.4f\n', K1);
fprintf('K2 = -Kp - 2*Kd/Ts      = %.4f\n', K2);
fprintf('K3 = Kd/Ts               = %.4f\n', K3);

%% ==================== DIGITAL BLOCK PARAMETERS ====================
wp_dac  = 2*pi*500e3;   % DAC reconstruction pole (rad/s)
T256    = 3.9e-9;        % 256 MHz digital clock period (s)
wc      = 2*pi*fc;       % crossover frequency (rad/s)
Td      = 4e-9;          % computational/propagation delay (s)
Td_ADC  = 3.9e-9;        % ADC hardware delay (s)

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
% Blanking delay (Pade approximation)
Tblank = 15.2e-9;
[numBlank, denBlank] = pade(Tblank, 2);
Hblank = tf(numBlank, denBlank);

% Computational delay (Pade approximation)
[numD, denD] = pade(Td, 2);
Delay = tf(numD, denD);

% ZOH Model (using Pade for exponential delay)
[numZ, denZ] = pade(T256, 1);
Hzoh = (1 - tf(numZ, denZ)) / (T256*s);

% FIX: ADC delay — use Pade instead of exp(-s*Td_ADC) with tf objects
[numADC, denADC] = pade(Td_ADC, 2);
Hadc_delay = tf(numADC, denADC);
Hadc = Hzoh * Hadc_delay;

% PID compensator (continuous-time form)
Hpid = Kp + Ki/s + (Kd*s)/(Tf*s + 1);

% DAC reconstruction filter
Hdac = Vref / (1 + s/wp_dac);

% Complete digital compensator chain
Hdig = minreal(Hadc * Hpid * Hdac * Hblank * Delay);

%% ==================== PLANT & LOOP GAINS ====================
Gvc = minreal(Hdc * Fp * Fh);    % Vout/Vc (PCM control-to-output)

Lloop = Hdig * Gvc * H;          % Digital loop gain
Tloop = feedback(Lloop, 1);      % Digital closed-loop

Lc = Hc * Gvc * H;               % Analog loop gain
Tc = feedback(Lc, 1);            % Analog closed-loop

%% ==================== STABILITY MARGINS ====================
[Gm_dig, Pm_dig, Wcg_dig, Wcp_dig] = margin(Lloop);
[Gm_ana, Pm_ana, Wcg_ana, Wcp_ana] = margin(Lc);

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
w = 2*pi*logspace(0, 8, 2000);   % 1 Hz to 100 MHz, 2000 points

figure; margin(Gvc, w);    grid on; title('Plant: Gvc = Vout/Vc');
figure; margin(Hadc, w);   grid on; title('ADC: Hadc (ZOH + delay)');
figure; margin(Hpid, w);   grid on; title('PID: Hpid');
figure; margin(Hdig, w);   grid on; title('Digital Compensator: Hadc \rightarrow Hpid \rightarrow Hdac');
figure; margin(Hc, w);     grid on; title('Desired Analog Compensator: Hc (Type II)');
figure; margin(Lc, w);     grid on; title('Analog Loop Gain: Lc = Hc \cdot Gvc \cdot H');
figure; margin(Lloop, w);  grid on; title('Digital Loop Gain: Lloop = Hdig \cdot Gvc \cdot H');

%% ==================== REFERENCE STEP RESPONSE ====================
figure('Name', 'Reference Step Response');
subplot(2,1,1)
step(Tloop); grid on; title('Digital Closed-Loop Step Response (Tloop)');
subplot(2,1,2)
step(Tc);    grid on; title('Analog Closed-Loop Step Response (Tc)');

%% ==================== LOAD STEP RESPONSE ANALYSIS ====================
% Load transient parameters
dI_load   = 1.6;          % Load step magnitude (A)
t_rise    = 100e-9;        % Load step rise/edge time (s)
I_load0   = 0.4;           % Initial (startup) load current (A)
R_load0   = VOUT/I_load0;  % Initial load resistance = 0.8/0.4 = 2.0 Ohm
R_load1   = VOUT/(I_load0 + dI_load);  % Final load resistance = 0.8/2.0 = 0.4 Ohm

fprintf('\n===== LOAD STEP PARAMETERS =====\n');
fprintf('Initial current:  %.1f mA  (R_load = %.3f Ohm)\n', I_load0*1e3, R_load0);
fprintf('Step magnitude:   %.1f A\n', dI_load);
fprintf('Final current:    %.1f A   (R_load = %.3f Ohm)\n', I_load0+dI_load, R_load1);
fprintf('Edge time:        %.0f ns\n', t_rise*1e9);

%% ---- Open-Loop Output Impedance ----
% Capacitor impedance: ESR in series with capacitance
Zc = Resr + 1/(s*C);

% For PCM control, the inductor behaves as a current source
% so open-loop Zout is the capacitor in parallel with load
Zout_ol_passive = Zc * R / (Zc + R);

% PCM modifies impedance through the current sense loop
Zout_ol = Zout_ol_passive * Fp * Fh;

%% ---- Closed-Loop Output Impedance ----
% Zout_cl = Zout_ol / (1 + T(s))
Zout_cl_dig = minreal(Zout_ol / (1 + Lloop));
Zout_cl_ana = minreal(Zout_ol / (1 + Lc));

%% ---- Voltage Deviation Transfer Functions ----
% dVout(s) = Zout_cl(s) × dIload
% For step() input, multiply by the step amplitude
dVout_dig = minreal(Zout_cl_dig * dI_load);
dVout_ana = minreal(Zout_cl_ana * dI_load);

% With finite rise time: model load step edge
% Iload(s) = dI / (s × (1 + s×t_rise))
% dVout_edge(s) = Zout_cl(s) × dI / (1 + s×t_rise)
dVout_edge_dig = minreal(Zout_cl_dig * dI_load / (1 + s*t_rise));
dVout_edge_ana = minreal(Zout_cl_ana * dI_load / (1 + s*t_rise));

%% ---- Output Impedance Bode Plot ----
figure('Name', 'Output Impedance', 'Position', [100 100 900 600]);
bodemag(Zout_ol, w); hold on;
bodemag(Zout_cl_dig, w);
bodemag(Zout_cl_ana, w);
grid on;
title('Output Impedance: Open-Loop vs Closed-Loop');
legend('Z_{out,OL}', 'Z_{out,CL} (Digital PID)', 'Z_{out,CL} (Analog Type-II)', ...
       'Location', 'southwest');

%% ==================== CUSTOM LOAD STEP PROFILE ====================
% Time parameters
t_end     = 70e-6;          % Total simulation time (70 µs)
dt        = 1e-9;           % Time resolution (1 ns)
t_sim     = (0:dt:t_end)';  % Time vector (column)
N_pts     = length(t_sim);

% Load step events
t_step_pos = 30e-6;         % Positive load step at 30 µs
t_step_neg = 50e-6;         % Negative load step at 50 µs

% Build load current profile with finite edge time
I_load = zeros(N_pts, 1) + I_load0;   % Start at 400 mA

for i = 1:N_pts
    t = t_sim(i);
    if t >= t_step_pos && t < t_step_neg
        % Positive step: ramp up with t_rise edge
        dt_from_step = t - t_step_pos;
        if dt_from_step < t_rise
            I_load(i) = I_load0 + dI_load * (dt_from_step / t_rise);
        else
            I_load(i) = I_load0 + dI_load;
        end
    elseif t >= t_step_neg
        % Negative step: ramp back down with t_rise edge
        dt_from_step = t - t_step_neg;
        if dt_from_step < t_rise
            I_load(i) = (I_load0 + dI_load) - dI_load * (dt_from_step / t_rise);
        else
            I_load(i) = I_load0;
        end
    end
end

% Perturbation current (input to Zout_cl)
% dI = deviation from initial steady-state
dI_perturbation = I_load - I_load0;

fprintf('\n===== CUSTOM LOAD STEP PROFILE =====\n');
fprintf('t = 0–30 µs:   I_load = %.0f mA (steady)\n', I_load0*1e3);
fprintf('t = 30 µs:     +%.1f A step (%.0f ns edge)\n', dI_load, t_rise*1e9);
fprintf('t = 30–50 µs:  I_load = %.1f A (steady)\n', I_load0+dI_load);
fprintf('t = 50 µs:     -%.1f A step (%.0f ns edge)\n', dI_load, t_rise*1e9);
fprintf('t = 50–70 µs:  I_load = %.0f mA (steady)\n', I_load0*1e3);

%% ---- Simulate using lsim ----
% Convert to state-space for lsim (faster for large vectors)
Zout_cl_dig_ss = ss(Zout_cl_dig);
Zout_cl_ana_ss = ss(Zout_cl_ana);

fprintf('Simulating digital loop response...\n');
y_dig_custom = lsim(Zout_cl_dig_ss, dI_perturbation, t_sim);

fprintf('Simulating analog loop response...\n');
y_ana_custom = lsim(Zout_cl_ana_ss, dI_perturbation, t_sim);

%% ---- Figure 1: Full Load Step Response ----
figure('Name', 'Load Step: +30µs / -50µs', 'Position', [50 50 1400 900]);

% --- Load current profile ---
subplot(3,2,1);
plot(t_sim*1e6, I_load*1e3, 'k', 'LineWidth', 2);
xlabel('Time (\mus)'); ylabel('I_{load} (mA)');
title('Load Current Profile');
grid on; xlim([0 70]);
yline(I_load0*1e3, 'b:', sprintf('%.0f mA', I_load0*1e3));
yline((I_load0+dI_load)*1e3, 'r:', sprintf('%.0f mA', (I_load0+dI_load)*1e3));
xline(30, 'g--', '+step'); xline(50, 'm--', '-step');

% --- ΔVout (mV) ---
subplot(3,2,2);
plot(t_sim*1e6, y_dig_custom*1e3, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim*1e6, y_ana_custom*1e3, 'r--', 'LineWidth', 1.5);
xlabel('Time (\mus)'); ylabel('\DeltaVout (mV)');
title('Output Voltage Deviation');
legend('Digital PID', 'Analog Type-II', 'Location', 'best');
grid on; xlim([0 70]);
yline(VOUT*0.03*1e3, 'g--', '+3%');
yline(-VOUT*0.03*1e3, 'g--', '-3%');
yline(0, 'k:');
xline(30, 'g--'); xline(50, 'm--');

% --- Absolute Vout ---
subplot(3,2,3);
plot(t_sim*1e6, VOUT + y_dig_custom, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim*1e6, VOUT + y_ana_custom, 'r--', 'LineWidth', 1.5);
xlabel('Time (\mus)'); ylabel('Vout (V)');
title('Absolute Output Voltage');
legend('Digital PID', 'Analog Type-II', 'Location', 'best');
grid on; xlim([0 70]);
yline(VOUT, 'k:', 'Nominal');
yline(VOUT*1.03, 'g--', '+3%'); yline(VOUT*0.97, 'g--', '-3%');
yline(VOUT*1.05, 'm:', '+5%'); yline(VOUT*0.95, 'm:', '-5%');
xline(30, 'g--'); xline(50, 'm--');

% --- Zoomed: Positive step (28–40 µs) ---
subplot(3,2,4);
idx_pos = t_sim >= 28e-6 & t_sim <= 40e-6;
plot(t_sim(idx_pos)*1e6, y_dig_custom(idx_pos)*1e3, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim(idx_pos)*1e6, y_ana_custom(idx_pos)*1e3, 'r--', 'LineWidth', 1.5);
xlabel('Time (\mus)'); ylabel('\DeltaVout (mV)');
title('Zoomed: Positive Step (28–40 \mus)');
legend('Digital PID', 'Analog Type-II', 'Location', 'southeast');
grid on;
xline(30, 'k:', 't_{step+}');
xline(30 + t_rise*1e6, 'k:', 't_{edge}');
yline(VOUT*0.03*1e3, 'g--'); yline(-VOUT*0.03*1e3, 'g--');

% --- Zoomed: Negative step (48–60 µs) ---
subplot(3,2,5);
idx_neg = t_sim >= 48e-6 & t_sim <= 60e-6;
plot(t_sim(idx_neg)*1e6, y_dig_custom(idx_neg)*1e3, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim(idx_neg)*1e6, y_ana_custom(idx_neg)*1e3, 'r--', 'LineWidth', 1.5);
xlabel('Time (\mus)'); ylabel('\DeltaVout (mV)');
title('Zoomed: Negative Step (48–60 \mus)');
legend('Digital PID', 'Analog Type-II', 'Location', 'northeast');
grid on;
xline(50, 'k:', 't_{step-}');
xline(50 + t_rise*1e6, 'k:', 't_{edge}');
yline(VOUT*0.03*1e3, 'g--'); yline(-VOUT*0.03*1e3, 'g--');

% --- Combined: ΔVout + Load on same plot ---
subplot(3,2,6);
yyaxis left;
plot(t_sim*1e6, y_dig_custom*1e3, 'b', 'LineWidth', 1.5);
ylabel('\DeltaVout (mV)');
yyaxis right;
plot(t_sim*1e6, I_load, 'k--', 'LineWidth', 1.2);
ylabel('I_{load} (A)');
xlabel('Time (\mus)');
title('Voltage Deviation vs Load Current');
grid on; xlim([0 70]);

sgtitle(sprintf('Load Transient: %.0f mA \\leftrightarrow %.1f A  |  +step@30\\mus, -step@50\\mus  |  Vout=%.1fV, fs=%.0fMHz', ...
    I_load0*1e3, I_load0+dI_load, VOUT, fs/1e6), 'FontSize', 13, 'FontWeight', 'bold');

%% ==================== LOAD STEP PERFORMANCE METRICS ====================
% Positive step region (30–50 µs)
idx_pos_full = t_sim >= t_step_pos & t_sim < t_step_neg;
y_dig_pos = y_dig_custom(idx_pos_full);
y_ana_pos = y_ana_custom(idx_pos_full);
t_pos     = t_sim(idx_pos_full);

% Negative step region (50–70 µs)
idx_neg_full = t_sim >= t_step_neg;
y_dig_neg = y_dig_custom(idx_neg_full);
y_ana_neg = y_ana_custom(idx_neg_full);
t_neg     = t_sim(idx_neg_full);

% Peak deviations - positive step
dVpeak_dig_pos = max(abs(y_dig_pos));
dVpeak_ana_pos = max(abs(y_ana_pos));
undershoot_dig_pos = min(y_dig_pos);
undershoot_ana_pos = min(y_ana_pos);
overshoot_dig_pos  = max(y_dig_pos);
overshoot_ana_pos  = max(y_ana_pos);

% Peak deviations - negative step
dVpeak_dig_neg = max(abs(y_dig_neg));
dVpeak_ana_neg = max(abs(y_ana_neg));
overshoot_dig_neg  = max(y_dig_neg);   % Vout goes UP on negative load step
overshoot_ana_neg  = max(y_ana_neg);
undershoot_dig_neg = min(y_dig_neg);
undershoot_ana_neg = min(y_ana_neg);

% Settling time (2% band) - positive step
settle_band = 0.02 * VOUT;
idx_s_dig_pos = find(abs(y_dig_pos) > settle_band, 1, 'last');
idx_s_ana_pos = find(abs(y_ana_pos) > settle_band, 1, 'last');
if ~isempty(idx_s_dig_pos)
    t_settle_dig_pos = t_pos(idx_s_dig_pos) - t_step_pos;
else
    t_settle_dig_pos = 0;
end
if ~isempty(idx_s_ana_pos)
    t_settle_ana_pos = t_pos(idx_s_ana_pos) - t_step_pos;
else
    t_settle_ana_pos = 0;
end

% Settling time (2% band) - negative step
% For negative step, the reference is zero again (dI goes back to 0)
idx_s_dig_neg = find(abs(y_dig_neg) > settle_band, 1, 'last');
idx_s_ana_neg = find(abs(y_ana_neg) > settle_band, 1, 'last');
if ~isempty(idx_s_dig_neg)
    t_settle_dig_neg = t_neg(idx_s_dig_neg) - t_step_neg;
else
    t_settle_dig_neg = 0;
end
if ~isempty(idx_s_ana_neg)
    t_settle_ana_neg = t_neg(idx_s_ana_neg) - t_step_neg;
else
    t_settle_ana_neg = 0;
end

% Analytical estimates
t_response = 1/(2*pi*fc);
dV_esr  = dI_load * Resr;
dV_cap  = dI_load * t_response / C;
dV_Ldi  = Lind * dI_load / t_rise;

fprintf('\n===================================================================\n');
fprintf('         LOAD STEP PERFORMANCE SUMMARY\n');
fprintf('         +%.1fA @ 30µs  /  -%.1fA @ 50µs  /  edge=%.0fns\n', dI_load, dI_load, t_rise*1e9);
fprintf('===================================================================\n');
fprintf('Vout = %.3f V  |  I_load: %.0f mA ↔ %.1f A\n', VOUT, I_load0*1e3, I_load0+dI_load);
fprintf('-------------------------------------------------------------------\n');

fprintf('\n--- POSITIVE STEP (+%.1f A @ 30 µs) ---\n', dI_load);
fprintf('                        Digital PID       Analog Type-II\n');
fprintf('  Peak |ΔVout|:         %.1f mV           %.1f mV\n', dVpeak_dig_pos*1e3, dVpeak_ana_pos*1e3);
fprintf('  Peak %% deviation:     %.2f%%              %.2f%%\n', dVpeak_dig_pos/VOUT*100, dVpeak_ana_pos/VOUT*100);
fprintf('  Undershoot (drop):    %.1f mV           %.1f mV\n', undershoot_dig_pos*1e3, undershoot_ana_pos*1e3);
fprintf('  Overshoot (recovery): %.1f mV           %.1f mV\n', overshoot_dig_pos*1e3, overshoot_ana_pos*1e3);
fprintf('  2%% settling time:     %.2f µs           %.2f µs\n', t_settle_dig_pos*1e6, t_settle_ana_pos*1e6);

fprintf('\n--- NEGATIVE STEP (-%.1f A @ 50 µs) ---\n', dI_load);
fprintf('                        Digital PID       Analog Type-II\n');
fprintf('  Peak |ΔVout|:         %.1f mV           %.1f mV\n', dVpeak_dig_neg*1e3, dVpeak_ana_neg*1e3);
fprintf('  Peak %% deviation:     %.2f%%              %.2f%%\n', dVpeak_dig_neg/VOUT*100, dVpeak_ana_neg/VOUT*100);
fprintf('  Overshoot (spike):    %.1f mV           %.1f mV\n', overshoot_dig_neg*1e3, overshoot_ana_neg*1e3);
fprintf('  Undershoot (recov):   %.1f mV           %.1f mV\n', undershoot_dig_neg*1e3, undershoot_ana_neg*1e3);
fprintf('  2%% settling time:     %.2f µs           %.2f µs\n', t_settle_dig_neg*1e6, t_settle_ana_neg*1e6);

fprintf('\n--- ANALYTICAL ESTIMATES ---\n');
fprintf('  ESR voltage drop:     %.1f mV  (%.1f A × %.0f mΩ)\n', dV_esr*1e3, dI_load, Resr*1e3);
fprintf('  Capacitive sag:       %.1f mV  (ΔI × t_resp / C)\n', dV_cap*1e3);
fprintf('  Inductive kick:       %.1f mV  (L × ΔI / t_edge)\n', dV_Ldi*1e3);

fprintf('\n--- SPEC CHECK (worst of pos/neg) ---\n');
dVpeak_worst_dig = max(dVpeak_dig_pos, dVpeak_dig_neg);
dVpeak_worst_ana = max(dVpeak_ana_pos, dVpeak_ana_neg);
spec_pct = [1 2 3 5 10];
for i = 1:length(spec_pct)
    spec_mV = VOUT * spec_pct(i)/100 * 1e3;
    pass_dig = dVpeak_worst_dig*1e3 < spec_mV;
    pass_ana = dVpeak_worst_ana*1e3 < spec_mV;
    if pass_dig; s_dig = 'PASS'; else; s_dig = 'FAIL'; end
    if pass_ana; s_ana = 'PASS'; else; s_ana = 'FAIL'; end
    fprintf('  ±%d%% (±%.0f mV):  Digital=%s  Analog=%s\n', ...
        spec_pct(i), spec_mV, s_dig, s_ana);
end

fprintf('===================================================================\n');

%% ==================== COMBINED SUMMARY FIGURE ====================
figure('Name', 'Complete Summary', 'Position', [50 50 1400 900]);

% Row 1: Bode
subplot(3,3,1);
bodemag(Zout_ol, w); hold on;
bodemag(Zout_cl_dig, w);
bodemag(Zout_cl_ana, w);
grid on; title('|Z_{out}(f)|');
legend('OL', 'Dig', 'Ana', 'Location', 'southwest', 'FontSize', 7);

subplot(3,3,2);
margin(Lloop, w); grid on; title('Digital Loop Gain');

subplot(3,3,3);
margin(Lc, w); grid on; title('Analog Loop Gain');

% Row 2: Full transient
subplot(3,3,4);
plot(t_sim*1e6, I_load*1e3, 'k', 'LineWidth', 2);
xlabel('Time (\mus)'); ylabel('I_{load} (mA)');
title('Load Current'); grid on; xlim([0 70]);

subplot(3,3,5);
plot(t_sim*1e6, y_dig_custom*1e3, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim*1e6, y_ana_custom*1e3, 'r--', 'LineWidth', 1.5);
xlabel('Time (\mus)'); ylabel('\DeltaVout (mV)');
title('\DeltaVout'); grid on; xlim([0 70]);
legend('Dig', 'Ana', 'FontSize', 7);

subplot(3,3,6);
plot(t_sim*1e6, VOUT + y_dig_custom, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim*1e6, VOUT + y_ana_custom, 'r--', 'LineWidth', 1.5);
xlabel('Time (\mus)'); ylabel('Vout (V)');
title('Absolute Vout'); grid on; xlim([0 70]);
yline(VOUT, 'k:'); yline(VOUT*1.03, 'g--'); yline(VOUT*0.97, 'g--');

% Row 3: Zoomed
subplot(3,3,7);
plot(t_sim(idx_pos)*1e6, y_dig_custom(idx_pos)*1e3, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim(idx_pos)*1e6, y_ana_custom(idx_pos)*1e3, 'r--', 'LineWidth', 1.5);
xlabel('Time (\mus)'); ylabel('\DeltaVout (mV)');
title('Zoom: +Step'); grid on;
yline(VOUT*0.03*1e3, 'g--'); yline(-VOUT*0.03*1e3, 'g--');

subplot(3,3,8);
plot(t_sim(idx_neg)*1e6, y_dig_custom(idx_neg)*1e3, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim(idx_neg)*1e6, y_ana_custom(idx_neg)*1e3, 'r--', 'LineWidth', 1.5);
xlabel('Time (\mus)'); ylabel('\DeltaVout (mV)');
title('Zoom: -Step'); grid on;
yline(VOUT*0.03*1e3, 'g--'); yline(-VOUT*0.03*1e3, 'g--');

subplot(3,3,9);
step(Tloop); hold on; step(Tc);
grid on; title('Ref Step Response');
legend('Digital', 'Analog', 'FontSize', 7);

sgtitle(sprintf('PCM Buck: %.0fmA \\leftrightarrow %.1fA  |  +@30\\mus  -@50\\mus  |  Vout=%.1fV  fs=%.0fMHz  fc=%.0fkHz', ...
    I_load0*1e3, I_load0+dI_load, VOUT, fs/1e6, fc/1e3), ...
    'FontSize', 13, 'FontWeight', 'bold');

%% ---------------------------------------------------------- %%