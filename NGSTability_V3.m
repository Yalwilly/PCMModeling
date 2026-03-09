%% ====== NGSTability_V3.m ======
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
R    = 400e-3;        % Load resistance (Ohm)
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
Ki = 1e07;     % Integral gain
Kd = 1e-06;        % Derivative gain

% Discrete PID coefficients (for reference/documentation)
K1 = Kp + Ki*Ts + Kd/Ts;
K2 = -Kp - 2*Kd/Ts;
K3 = Kd/Ts;

%% ==================== DIGITAL BLOCK PARAMETERS ====================
wp_dac  = 2*pi*500e3;   % DAC reconstruction pole (rad/s)
T256    = 3.9e-9;        % 256 MHz digital clock period (s)
wc      = 2*pi*fc;       % crossover frequency (rad/s)
Td      = 4e-9;          % computational/propagation delay (s)
Td_ADC  = 5e-9;          % ADC hardware delay (s)

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
Tblank = 15e-9;
[numBlank, denBlank] = pade(Tblank, 2);
Hblank = tf(numBlank, denBlank);

% Computational delay (Pade approximation)
[numD, denD] = pade(Td, 2);
Delay = tf(numD, denD);

% ZOH Model — use SWITCHING PERIOD, not digital clock period
% The PID output updates the comparator threshold once per switching cycle.
% T256 only applies to the ADC oversampling, not the control update rate.
[numZ_sw, denZ_sw] = pade(Ts/2, 2);         % half-period transport delay
Hzoh_sw = (1 - tf(numZ_sw, denZ_sw)) / (Ts*s);  % ZOH at fs

% ADC ZOH still at T256 (sigma-delta oversampling)
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

% Complete digital compensator chain — include switching-rate ZOH
Hdig = minreal(Hadc * Hpid * Hdac * Hblank * Delay * Hzoh_sw);

%% ==================== PLANT & LOOP GAINS ====================
Gvc = minreal(Hdc * Fp * Fh);    % Vout/Vc (PCM control-to-output)

Lloop = Hdig * Gvc * H;          % Digital loop gain (renamed from L)
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

%% ==================== STEP RESPONSE ====================
figure;
subplot(2,1,1)
step(Tloop); grid on; title('Digital Closed-Loop Step Response (Tloop)');
subplot(2,1,2)
step(Tc);    grid on; title('Analog Closed-Loop Step Response (Tc)');

%% ==================== DIAGNOSTIC: PHASE CONTRIBUTION ====================
fprintf('\n===== PHASE BUDGET AT CROSSOVER (%.0f kHz) =====\n', fc/1e3);

blocks = {Hadc, Hpid, Hdac, Hblank, Delay, Gvc*H};
names  = {'ADC', 'PID', 'DAC', 'Blanking', 'CompDelay', 'Plant*H'};

total_phase = 0;
for k = 1:length(blocks)
    [~, ph] = bode(blocks{k}, 2*pi*fc);
    fprintf('  %-12s: %+7.1f deg\n', names{k}, ph);
    total_phase = total_phase + ph;
end
fprintf('  %-12s: %+7.1f deg\n', 'TOTAL', total_phase);
fprintf('  %-12s: %+7.1f deg\n', 'Phase Margin', total_phase + 180);

%% ==================== WHAT-IF: MATCH AMS DELAY ====================
% Adjust this until MATLAB matches AMS behavior
Td_ams = 80e-9;   % <-- Increase until MATLAB also goes unstable
[nAms, dAms] = pade(Td_ams, 3);
Hdelay_ams = tf(nAms, dAms);

Hdig_ams = minreal(Hadc * Hpid * Hdac * Hdelay_ams);
Lloop_ams = Hdig_ams * Gvc * H;
[~, Pm_ams] = margin(Lloop_ams);
fprintf('\nWith %.0f ns total delay: PM = %.1f deg\n', Td_ams*1e9, Pm_ams);

figure; margin(Lloop_ams, w);
title(sprintf('Loop Gain with %.0f ns AMS-matched delay (PM=%.1f°)', Td_ams*1e9, Pm_ams));
grid on;

%% ---------------------------------------------------------- %%


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
