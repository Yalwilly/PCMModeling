%% ------------------------------- %%
clear; close all; clc;
s = tf('s');

% ====== define your blocks ======
% H(s): feedback network 
beta = 0.5; %% Vfb divider 
H = beta;
%% -------------------- Params --------------------
VOUT=0.8;
VIN=3.3 ; 
L=0.47e-6;
Vref= 0.8 ;

Rramp=100e3;
Cramp=1e-12;

fs = 6e6;  % switching frequency (Hz)
Ts=1/fs; 
fc=fs/10; %% Due to theortical tansfer function 
Tf=1/(10*fc);
fdig=256e6; 
Tdig=1/fdig; 

GI=(1/640)*(10/64)*(1/4);%Current Gain (1/3200) in TC
Rsense=8e3; % Rsebse low side 8k Ohm
Ri=GI*Rsense; %Total Current sense gain 
 
D=VOUT/VIN; 
D_tag=1-D;
%Inductor current slopes. 
se=0.7e6; % Artificial ramp from simulation 
sn=(VIN-VOUT)/L ; 
sf=VOUT/L;
seff=Ri*(VIN-VOUT)/L +se; % Effective Vramp slope.

Alpha=(sf-se)/(sn+se) ;

mc=1+se/sn; 

R    = 1e-3; % ohm
C    = 22e-6;      % F
Resr = 10e-3;       % ohm

wp=1/(C*R) + (Ts*(mc*D_tag -0.5)/(L*C));
wn=pi/Ts; 
Qp=1/(pi*(mc*D_tag-0.5)); 
R1 = sqrt( 1 - 4*(fc^2)*(Ts^2) + 16*(fc^4)*(Ts^4) );
R2 = sqrt( 1 + (39.48*(C^2)*(fc^2)*(L^2)*(R^2)) / ( (L + 0.32*R*Ts)^2 ) );
%R1=268e03;
%R2=R1;
wz  = 2*pi*fc/5;          % rad/s
wp2 = 1/(C*Resr);           % rad/s
wp1 = (1.23*fc*Ri*R1*R2*(L + 0.32*R*Ts)) / (L*R);   % *** units per given equation ***

Hdc=(R/Ri)*(1/1+(R*Ts*(mc*D_tag - 0.5)/L));
Fp=(1+s*C*Resr)/(1+s/wp);
Fh=1/(1+s/(wn*Qp)+ (s^2)/(wn^2));
Hc = (wp1/s) * (1 + s/wz) / (1 + s/wp2);


K1=1547;%1547
K2=-1590;%
K3=45; 
% Gc(s): controller Vdac/e  (ADC --> PID --> SIGMA Delta --> Vdac )
Kp = (-K2-2*K3)/64;     % <-- your digital PI gains mapped to cont
Ki = (K1+K2+K3)/64;      % <-- Ki
Kd=K3/64;       % KD

Kp = 50;  %50   % <-- your digital PI gains mapped to cont
Ki = 8e07;%8e07      % <-- Ki
Kd=0;       % KD

K1=Kp + Ki + Kd; 
K2=-Kp -2*Kd; 
K3=Kd;

wp_dac = 2*pi*500e3; % DAC reconstruction pole (example)
Kadc=1;
T256=3.9e-9;
%Tc=3e-6;
%fc = 1/Tc;      % chosen crossover (Hz)
wc=2*pi*fc; 

Td = 4e-9;      % total digital delay (s)
Td_ADC=5e-9;

fd = min([10*fc, 1/Td, fs/10]);
wd = 2*pi*fd;

%%meff=3.55e6; %From AMS. 

Km=VIN*fs/seff ;
kloop=Km*Ri; 
KD=1+R/(Km*Ri); 
wz=1/(C*Resr);



Vap=VIN; 
 



if Alpha < 1
    disp('Alpha is less than 1 - the FB loop is stable'); 
else 
    disp('Alpha is greater than 1 - the FB loop isnt stable'); 
end 

if se>0.5*sf 
    disp('The buck converter stable for all D');
else
     disp('The buck converter isnt stable for all D - increase ma');

end      



%% -------------------------plots-----------------------

Tblank = 32e-9;
[num,den] = pade(Tblank,2);   % 2nd order is usually enough
Hblank = tf(num,den);

[numD, denD] = pade(Td, 2);      % 2nd order Pade for Bode work
Delay = tf(numD, denD);
% ZOH Model (using Pade for exponential delay)
[numZ, denZ] = pade(T256, 1);
Hzoh = (1 - tf(numZ, denZ)) / (T256*s);

% Total ADC Model including Hardware Delay
Hadc = Hzoh * exp(-s*Td_ADC); 
HsigmaDelta = 1/(1+s*T256); %% Sigma Delta transfer function.
%Hadc=(1-Delay)/s; 
%Hpid= pid(Kp,Ki,Kd,Ts);
%
Hpid= Kp + Ki/s + (Kd*s)/(Tf*s+1);
Hdac=(Vref/(1 + s/wp_dac));
%Hpid= pid(Kp,Ki,Kd,Tf);
Hdig = Hadc*Hpid*Hdac*Hblank*Delay; %% Voltage Outer loop transfer function 

Gvc = Hdc*Fp*Fh;% Vout/Vc  

L = Hdig * Gvc * H;
T=feedback(L,1);
Lc=Hc*Gvc*H;
Tc=feedback(Lc,1);
% ====== Plot Bode ======
%figure; bode(T_Vo_Vin); grid on; title('Vo/Vin (closed-loop line susceptibility)');
%figure; bode(T_iL_Vin); grid on; title('iL/Vin (closed-loop inductor current susceptibility)');
w = 2*pi*logspace(log10(0.0000001), log10(1e8), 1000);  % rad/s
% Optional: check margins of the loop

%figure; margin(T_Vo_Vin); grid on; title('Vo/Vin (closed-loop line susceptibility)');
figure; margin(Gvc,w); grid on; title('Gvc');
figure; margin(Hadc,w); grid on; title('Hadc');
figure; margin(Hpid,w); grid on; title('Hpid');
figure; margin(Hdig,w); grid on; title('Hadc -> Hpid -> Hdac');
figure; margin(Hc,w); grid on; title('Desired Feedback Hc');
figure; margin(Lc,w); grid on; title('Desired loop feedback Tc');
figure; margin(L,w); grid on; title('Loop Gain T(s) = Gc*Gp*H');

figure; 
subplot(2,1,1)
step(T);grid on; title('T Closed loop step resonse ');
subplot(2,1,2)
step(Tc) ;grid on; title('Tc Closed loop step resonse ');



%% ---------------------------------------------------------- %% 