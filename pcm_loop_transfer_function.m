%% Peak Current Mode DC-DC Converter Loop Transfer Function Analysis
% This script analyzes the loop transfer function of a Peak Current Mode
% (PCM) controlled DC-DC converter and compares different compensation
% approaches.
%
% References:
% - Erickson, R. W., & Maksimovic, D. (2001). Fundamentals of Power Electronics
% - Reference paper on PCM control compensation

clear all;
close all;
clc;

%% System Parameters
% Converter Specifications
Vin = 12;           % Input voltage (V)
Vout = 5;           % Output voltage (V)
D = Vout/Vin;       % Duty cycle
fsw = 100e3;        % Switching frequency (Hz)
Tsw = 1/fsw;        % Switching period (s)

% Component Values
L = 10e-6;          % Inductor (H)
C = 100e-6;         % Output capacitor (F)
ESR = 0.05;         % ESR of capacitor (Ohm)
Rload = 5;          % Load resistance (Ohm)
Iout = Vout/Rload;  % Output current (A)

% Current Sense Parameters
Ri = 0.1;           % Current sense resistor (Ohm)
Rs = Ri;            % Slope compensation not included in this basic model

% Modulator Gain
Vm = 2;             % PWM ramp amplitude (V)
Fm = 1/Vm;          % Modulator gain (1/V)

%% Small Signal Transfer Functions
% Frequency range for analysis
f = logspace(1, 6, 1000);
w = 2*pi*f;
s = 1j*w;

% LC Filter Corner Frequency
w0 = 1/sqrt(L*C);
f0 = w0/(2*pi);
Q = Rload*sqrt(C/L);
zeta = 1/(2*Q);

fprintf('System Characteristics:\n');
fprintf('LC corner frequency: %.2f Hz\n', f0);
fprintf('Quality factor Q: %.2f\n', Q);
fprintf('Damping factor zeta: %.4f\n', zeta);
fprintf('ESR zero frequency: %.2f Hz\n', 1/(2*pi*ESR*C));

%% Power Stage Transfer Function (Control to Output)
% Gvd(s) = Vout(s)/D(s) for PCM control
% Simplified model including LC filter and ESR zero

% ESR zero
fz_esr = 1/(2*pi*ESR*C);
wz_esr = 2*pi*fz_esr;

% Transfer function: Gvd(s) = Vout/D * (1 + s*ESR*C) / (1 + s/(w0*Q) + s^2/w0^2)
Gvd = Vout .* (1 + s*ESR*C) ./ (1 + s/(w0*Q) + (s/w0).^2);

% Current Loop Gain (for PCM)
% He = Current sense gain
He = Ri;

%% Compensation Network - Erickson Approach (Type II/III Compensator)
% Type II Compensator: H(s) = K * (1 + s/wz) / (s * (1 + s/wp))
% Designed to provide adequate phase margin and crossover frequency

% Design targets
fc_target = fsw/10;     % Crossover frequency (typically fsw/5 to fsw/10)
PM_target = 60;         % Phase margin target (degrees)

% Compensator zeros and poles (Erickson design)
fz_comp = f0/2;                    % Place zero below LC corner freq
fp_comp = min(fz_esr*2, fsw/2);    % Place pole above ESR zero
wz_comp = 2*pi*fz_comp;
wp_comp = 2*pi*fp_comp;

% Calculate DC gain to achieve desired crossover
% This is simplified; detailed design requires iteration
K_comp = 2e3;  % Compensator DC gain (adjusted for better phase margin)

% Type II Compensator Transfer Function
Hc_typeII = K_comp * (1 + s/wz_comp) ./ (s .* (1 + s/wp_comp));

% Type III Compensator: H(s) = K * (1 + s/wz1)(1 + s/wz2) / (s * (1 + s/wp1)(1 + s/wp2))
% More aggressive compensation with double pole at origin
fz1_comp = f0/2;
fz2_comp = f0;
fp1_comp = fz_esr*2;
fp2_comp = fsw/2;

wz1_comp = 2*pi*fz1_comp;
wz2_comp = 2*pi*fz2_comp;
wp1_comp = 2*pi*fp1_comp;
wp2_comp = 2*pi*fp2_comp;

K_comp_typeIII = 3e4;

% Type III Compensator Transfer Function
Hc_typeIII = K_comp_typeIII * (1 + s/wz1_comp) .* (1 + s/wz2_comp) ./ ...
             (s .* (1 + s/wp1_comp) .* (1 + s/wp2_comp));

fprintf('\nCompensation Network (Erickson Type II):\n');
fprintf('Zero frequency: %.2f Hz\n', fz_comp);
fprintf('Pole frequency: %.2f Hz\n', fp_comp);
fprintf('DC Gain: %.2e\n', K_comp);

fprintf('\nCompensation Network (Erickson Type III):\n');
fprintf('Zero 1 frequency: %.2f Hz\n', fz1_comp);
fprintf('Zero 2 frequency: %.2f Hz\n', fz2_comp);
fprintf('Pole 1 frequency: %.2f Hz\n', fp1_comp);
fprintf('Pole 2 frequency: %.2f Hz\n', fp2_comp);
fprintf('DC Gain: %.2e\n', K_comp_typeIII);

%% Digital Compensation Transfer Function Hdig (ADC -> PID -> DAC)
% Digital implementation introduces additional dynamics:
% - ADC sampling delay
% - Computation delay
% - Zero-Order Hold (ZOH) from DAC

fs = fsw;           % Sampling frequency equals switching frequency
Ts = 1/fs;          % Sampling period

% Digital delay (1.5 sample periods typical for ADC + computation + DAC)
delay_samples = 1.5;
Hdelay = exp(-s * delay_samples * Ts);

% Zero-Order Hold (ZOH) effect
% H_zoh(s) = (1 - exp(-s*Ts))/(s*Ts)
Hzoh = (1 - exp(-s*Ts)) ./ (s*Ts);

% Digital PID implementation (using bilinear transform from Type II)
% Simplified: we approximate the continuous compensator with digital effects
Hdig_typeII = Hc_typeII .* Hdelay .* Hzoh;

% For Type III
Hdig_typeIII = Hc_typeIII .* Hdelay .* Hzoh;

fprintf('\nDigital Compensation:\n');
fprintf('Sampling frequency: %.2f kHz\n', fs/1e3);
fprintf('Delay: %.1f sample periods\n', delay_samples);
fprintf('Total delay: %.2f us\n', delay_samples*Ts*1e6);

%% Loop Transfer Functions
% T(s) = Gvd(s) * Hc(s) * Fm
% Where Fm is the modulator gain

% Continuous compensation (Theoretical Hc)
T_typeII = Gvd .* Hc_typeII * Fm;
T_typeIII = Gvd .* Hc_typeIII * Fm;

% Digital compensation (Hdig)
T_dig_typeII = Gvd .* Hdig_typeII * Fm;
T_dig_typeIII = Gvd .* Hdig_typeIII * Fm;

%% Bode Plot Generation
figure('Position', [100, 100, 1200, 800]);

% Magnitude plot
subplot(2,1,1);
semilogx(f, 20*log10(abs(T_typeII)), 'b-', 'LineWidth', 2);
hold on;
semilogx(f, 20*log10(abs(T_typeIII)), 'r-', 'LineWidth', 2);
semilogx(f, 20*log10(abs(T_dig_typeII)), 'b--', 'LineWidth', 1.5);
semilogx(f, 20*log10(abs(T_dig_typeIII)), 'r--', 'LineWidth', 1.5);
semilogx(f, zeros(size(f)), 'k:', 'LineWidth', 1);
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Loop Transfer Function - Bode Magnitude Plot');
legend('Hc Type II (Continuous)', 'Hc Type III (Continuous)', ...
       'Hdig Type II (Digital)', 'Hdig Type III (Digital)', ...
       '0 dB Line', 'Location', 'southwest');
xlim([f(1) f(end)]);
ylim([-80 80]);

% Phase plot
subplot(2,1,2);
phase_typeII = angle(T_typeII)*180/pi;
phase_typeIII = angle(T_typeIII)*180/pi;
phase_dig_typeII = angle(T_dig_typeII)*180/pi;
phase_dig_typeIII = angle(T_dig_typeIII)*180/pi;

% Unwrap phase for better visualization
phase_typeII = unwrap(phase_typeII*pi/180)*180/pi;
phase_typeIII = unwrap(phase_typeIII*pi/180)*180/pi;
phase_dig_typeII = unwrap(phase_dig_typeII*pi/180)*180/pi;
phase_dig_typeIII = unwrap(phase_dig_typeIII*pi/180)*180/pi;

semilogx(f, phase_typeII, 'b-', 'LineWidth', 2);
hold on;
semilogx(f, phase_typeIII, 'r-', 'LineWidth', 2);
semilogx(f, phase_dig_typeII, 'b--', 'LineWidth', 1.5);
semilogx(f, phase_dig_typeIII, 'r--', 'LineWidth', 1.5);
semilogx(f, -180*ones(size(f)), 'k:', 'LineWidth', 1);
grid on;
xlabel('Frequency (Hz)');
ylabel('Phase (degrees)');
title('Loop Transfer Function - Bode Phase Plot');
legend('Hc Type II (Continuous)', 'Hc Type III (Continuous)', ...
       'Hdig Type II (Digital)', 'Hdig Type III (Digital)', ...
       '-180° Line', 'Location', 'southwest');
xlim([f(1) f(end)]);

% Save the figure
saveas(gcf, 'loop_transfer_function_bode.png');

%% Calculate and Display Stability Margins
% Find crossover frequency and phase margin for each case
[~, idx_cross_typeII] = min(abs(abs(T_typeII) - 1));
fc_typeII = f(idx_cross_typeII);
PM_typeII = 180 + phase_typeII(idx_cross_typeII);

[~, idx_cross_typeIII] = min(abs(abs(T_typeIII) - 1));
fc_typeIII = f(idx_cross_typeIII);
PM_typeIII = 180 + phase_typeIII(idx_cross_typeIII);

[~, idx_cross_dig_typeII] = min(abs(abs(T_dig_typeII) - 1));
fc_dig_typeII = f(idx_cross_dig_typeII);
PM_dig_typeII = 180 + phase_dig_typeII(idx_cross_dig_typeII);

[~, idx_cross_dig_typeIII] = min(abs(abs(T_dig_typeIII) - 1));
fc_dig_typeIII = f(idx_cross_dig_typeIII);
PM_dig_typeIII = 180 + phase_dig_typeIII(idx_cross_dig_typeIII);

fprintf('\n========== Stability Analysis ==========\n');
fprintf('\nType II Compensator (Continuous - Hc):\n');
fprintf('  Crossover Frequency: %.2f kHz\n', fc_typeII/1e3);
fprintf('  Phase Margin: %.2f degrees\n', PM_typeII);

fprintf('\nType III Compensator (Continuous - Hc):\n');
fprintf('  Crossover Frequency: %.2f kHz\n', fc_typeIII/1e3);
fprintf('  Phase Margin: %.2f degrees\n', PM_typeIII);

fprintf('\nType II Compensator (Digital - Hdig):\n');
fprintf('  Crossover Frequency: %.2f kHz\n', fc_dig_typeII/1e3);
fprintf('  Phase Margin: %.2f degrees\n', PM_dig_typeII);
fprintf('  Phase loss due to digital delay: %.2f degrees\n', PM_typeII - PM_dig_typeII);

fprintf('\nType III Compensator (Digital - Hdig):\n');
fprintf('  Crossover Frequency: %.2f kHz\n', fc_dig_typeIII/1e3);
fprintf('  Phase Margin: %.2f degrees\n', PM_dig_typeIII);
fprintf('  Phase loss due to digital delay: %.2f degrees\n', PM_typeIII - PM_dig_typeIII);

%% Comparison Plot: Hc vs Hdig
figure('Position', [150, 150, 1200, 800]);

% Type II Comparison
subplot(2,2,1);
semilogx(f, 20*log10(abs(Hc_typeII)), 'b-', 'LineWidth', 2);
hold on;
semilogx(f, 20*log10(abs(Hdig_typeII)), 'r--', 'LineWidth', 2);
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Type II: Hc (Continuous) vs Hdig (Digital) - Magnitude');
legend('Hc (Theoretical)', 'Hdig (with ADC-PID-DAC)', 'Location', 'northeast');
xlim([f(1) f(end)]);

subplot(2,2,2);
phase_Hc_typeII = unwrap(angle(Hc_typeII))*180/pi;
phase_Hdig_typeII = unwrap(angle(Hdig_typeII))*180/pi;
semilogx(f, phase_Hc_typeII, 'b-', 'LineWidth', 2);
hold on;
semilogx(f, phase_Hdig_typeII, 'r--', 'LineWidth', 2);
grid on;
xlabel('Frequency (Hz)');
ylabel('Phase (degrees)');
title('Type II: Hc vs Hdig - Phase');
legend('Hc (Theoretical)', 'Hdig (with ADC-PID-DAC)', 'Location', 'southwest');
xlim([f(1) f(end)]);

% Type III Comparison
subplot(2,2,3);
semilogx(f, 20*log10(abs(Hc_typeIII)), 'b-', 'LineWidth', 2);
hold on;
semilogx(f, 20*log10(abs(Hdig_typeIII)), 'r--', 'LineWidth', 2);
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Type III: Hc (Continuous) vs Hdig (Digital) - Magnitude');
legend('Hc (Theoretical)', 'Hdig (with ADC-PID-DAC)', 'Location', 'northeast');
xlim([f(1) f(end)]);

subplot(2,2,4);
phase_Hc_typeIII = unwrap(angle(Hc_typeIII))*180/pi;
phase_Hdig_typeIII = unwrap(angle(Hdig_typeIII))*180/pi;
semilogx(f, phase_Hc_typeIII, 'b-', 'LineWidth', 2);
hold on;
semilogx(f, phase_Hdig_typeIII, 'r--', 'LineWidth', 2);
grid on;
xlabel('Frequency (Hz)');
ylabel('Phase (degrees)');
title('Type III: Hc vs Hdig - Phase');
legend('Hc (Theoretical)', 'Hdig (with ADC-PID-DAC)', 'Location', 'southwest');
xlim([f(1) f(end)]);

% Save the figure
saveas(gcf, 'compensation_comparison_hc_vs_hdig.png');

fprintf('\n========== Analysis Complete ==========\n');
fprintf('Plots saved:\n');
fprintf('  - loop_transfer_function_bode.png\n');
fprintf('  - compensation_comparison_hc_vs_hdig.png\n');
