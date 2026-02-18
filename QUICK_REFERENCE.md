# Quick Reference Guide - PCM Compensation Design

## Design Flow Chart

```
1. Specify Converter Parameters
   ├─ Vin, Vout, fsw
   ├─ L, C, ESR, Rload
   └─ Target PM and fc

2. Calculate Power Stage Characteristics
   ├─ f₀ = 1/(2π√LC)
   ├─ Q = Rload√(C/L)
   └─ fESR = 1/(2πESR·C)

3. Choose Compensator Type
   ├─ Type II: fc < fsw/15
   └─ Type III: fc up to fsw/5

4. Place Poles and Zeros
   ├─ Type II: fz = f₀/2, fp = 2·fESR
   └─ Type III: fz1 = f₀/2, fz2 = f₀, fp1 = 2·fESR, fp2 = fsw/2

5. Calculate Gain (K)
   └─ Adjust to achieve target fc

6. Simulate and Verify
   ├─ Run MATLAB script
   ├─ Check PM > 45°
   └─ Check fc is appropriate

7. For Digital Implementation
   ├─ Model delays (Hdig)
   ├─ Reduce fc if needed
   └─ Re-verify stability
```

## Key Formulas

### Power Stage (PCM Control)
```
Gvd(s) = (Vout/D) · (1 + s·ESR·C) / (1 + s/(ω₀·Q) + s²/ω₀²)

where:
  ω₀ = 1/√(LC)
  Q = Rload·√(C/L)
  D = Vout/Vin (duty cycle)
```

### Type II Compensator
```
Hc(s) = K · (1 + s/ωz) / (s · (1 + s/ωp))

Design Guidelines:
  fz = f₀/2 to f₀/3    (below LC corner)
  fp = 2·fESR to 5·fESR (above ESR zero, max fsw/2)
  K = adjust for target fc
```

### Type III Compensator
```
Hc(s) = K · (1 + s/ωz1)(1 + s/ωz2) / (s · (1 + s/ωp1)(1 + s/ωp2))

Design Guidelines:
  fz1 = f₀/2           (first zero at half LC corner)
  fz2 = f₀             (second zero at LC corner)
  fp1 = 2·fESR         (first pole above ESR zero)
  fp2 = fsw/2          (second pole for noise filtering)
  K = adjust for target fc
```

### Loop Transfer Function
```
T(s) = Gvd(s) · Hc(s) · Fm

where:
  Fm = 1/Vm (modulator gain, typically Vm = 1-3V)
```

### Digital Compensation
```
Hdig(s) = Hc(s) · Hdelay(s) · Hzoh(s)

where:
  Hdelay(s) = e^(-s·Td)
  Hzoh(s) = (1 - e^(-s·Ts)) / (s·Ts)
  Td = delay time (typically 1-2 sampling periods)
  Ts = sampling period (1/fs)
```

### Phase Loss Due to Digital Delays
```
ΔPhase ≈ -360° · fc · Td - (fc/fs) · 30°

Example: fc = 10 kHz, fs = 100 kHz, Td = 1.5·Ts
  ΔPhase ≈ -360° · 10k · 15μs - (10k/100k) · 30°
         ≈ -54° - 3° ≈ -57°
```

## Stability Criteria

### Phase Margin (PM)
```
PM = 180° + ∠T(jωc)

where ωc is the crossover frequency (|T(jωc)| = 1)
```

**Recommended Values:**
- PM > 45°: Minimum acceptable
- PM = 50°-60°: Good design
- PM > 70°: Conservative (slower response)

### Gain Margin (GM)
```
GM = 1 / |T(jω₁₈₀)|

where ω₁₈₀ is the frequency where ∠T(jω₁₈₀) = -180°
```

**Recommended Values:**
- GM > 6 dB (factor of 2): Minimum
- GM > 10 dB: Good design

## Design Trade-offs

| Parameter | Increase Effect | Decrease Effect |
|-----------|----------------|-----------------|
| **Crossover Frequency (fc)** | Faster response, less PM | Slower response, more PM |
| **Compensator Gain (K)** | Higher fc, more sensitive | Lower fc, more robust |
| **Zero Frequency (fz)** | More phase boost at high freq | More phase boost at low freq |
| **Pole Frequency (fp)** | Less HF attenuation | More HF attenuation |

## Component Selection Guidelines

### Compensator Implementation (Analog)

**Type II - Single Op-Amp Configuration:**
```
R1, R2, C1, C2

Design equations:
  K = R2/R1
  fz = 1/(2π·R2·C1)
  fp = 1/(2π·R2·C2)
```

**Type III - Two Op-Amp Configuration:**
```
R1, R2, R3, C1, C2, C3

Design equations:
  fz1 = 1/(2π·R2·C1)
  fz2 = 1/(2π·R3·C2)
  fp1 = 1/(2π·R2·C3)
  fp2 = 1/(2π·R3·C4)
```

## MATLAB Script Usage

### Running the Analysis
```matlab
% Edit parameters in pcm_loop_transfer_function.m
Vin = 12;           % Input voltage
Vout = 5;           % Output voltage
fsw = 100e3;        % Switching frequency
L = 10e-6;          % Inductance
C = 100e-6;         % Capacitance
ESR = 0.05;         % ESR
Rload = 5;          % Load resistance

% Run the script
pcm_loop_transfer_function

% Output:
% - Console display of parameters and margins
% - loop_transfer_function_bode.png
% - compensation_comparison_hc_vs_hdig.png
```

### Modifying Compensation

**To adjust Type II compensator:**
```matlab
% Find this section in the script
fz_comp = f0/2;                    % Adjust zero placement
fp_comp = min(fz_esr*2, fsw/2);    % Adjust pole placement
K_comp = 2e3;                       % Adjust gain
```

**To adjust Type III compensator:**
```matlab
% Find this section in the script
fz1_comp = f0/2;    % First zero
fz2_comp = f0;      % Second zero
fp1_comp = fz_esr*2;    % First pole
fp2_comp = fsw/2;       % Second pole
K_comp_typeIII = 3e4;   % Gain
```

## Common Issues and Solutions

### Issue 1: Negative or Low Phase Margin
**Symptoms:** PM < 30°, unstable or oscillatory
**Solutions:**
- Reduce compensator gain (K)
- Lower crossover frequency
- Add more phase boost (adjust zero placement)
- Check for modeling errors

### Issue 2: Crossover Too Low
**Symptoms:** Very slow transient response
**Solutions:**
- Increase compensator gain (K)
- Move zeros closer to f₀
- Consider Type III instead of Type II
- Verify power stage model is correct

### Issue 3: Digital Implementation Unstable (Hdig)
**Symptoms:** Good PM with Hc, poor with Hdig
**Solutions:**
- Reduce crossover frequency (fc < fs/6)
- Increase sampling frequency if possible
- Add extra zero for phase boost
- Use predictor-based compensation

### Issue 4: High-Frequency Noise
**Symptoms:** Switching noise in output
**Solutions:**
- Lower pole frequency (fp)
- Add second pole (use Type III)
- Ensure fp < fsw/2
- Check PCB layout and filtering

## Practical Tips

### Design Process
1. **Start conservative**: Low fc, high PM
2. **Iterate gradually**: Increase fc while maintaining PM > 45°
3. **Simulate before build**: Use MATLAB/SPICE
4. **Verify in hardware**: Load transients, line transients
5. **Document**: Save final values and test results

### Digital Implementation
1. **Always model Hdig**: Don't assume Hc = Hdig
2. **Sample fast**: fs ≥ 2·fsw preferred
3. **Minimize delays**: Optimize firmware
4. **Account for quantization**: Use sufficient ADC/DAC resolution
5. **Test worst-case**: Min/max load, input voltage

### Measurement and Validation
1. **Bode plot**: Network analyzer or swept sine
2. **Step response**: Load transient with oscilloscope
3. **Stability**: Tap on inductor, observe ringing
4. **Margins**: Vary load/line, check for oscillation
5. **Temperature**: Test over operating range

## References for Further Study

**Books:**
- Erickson & Maksimovic: "Fundamentals of Power Electronics" (2nd ed.)
  - Chapter 9: Controller Design
  - Chapter 19: Current Mode Control
  
**Application Notes:**
- TI SLVA662: "Designing a TPS40210 SEPIC Power Supply"
- Analog Devices MT-031: "Grounding Data Converters and Solving the Mystery of 'AGND' and 'DGND'"
- Linear Technology AN-149: "Modeling and Control Loop Design for CCM PFC Pre-Regulators"

**Online Tools:**
- Ridley Engineering: Free compensation calculator
- TI WEBENCH Power Designer: Automated design tool
- LTspice: Free SPICE simulator with power components

**Papers:**
- Ridley, R. B. (1990): "A New, Continuous-Time Model for Current-Mode Control"
- Tang et al. (1993): "Small-Signal Modeling of Average Current-Mode Control"

## Quick Reference Table

| Symbol | Description | Typical Value |
|--------|-------------|---------------|
| f₀ | LC corner frequency | 1-10 kHz |
| Q | Quality factor | 0.5-20 |
| fESR | ESR zero frequency | 10-50 kHz |
| fc | Crossover frequency | fsw/10 to fsw/5 |
| PM | Phase margin | 45-65° |
| GM | Gain margin | 6-15 dB |
| fs | Sampling frequency | fsw to 3·fsw |
| Td | Digital delay | 1-2.5 sample periods |

## Version History

- v1.0 (Feb 2026): Initial release with Type II and Type III compensators
- Includes Hc vs Hdig comparison
- MATLAB/Octave compatible
- Automatic stability analysis

---

**For questions or issues, refer to:**
- README.md: Project overview
- COMPENSATION_COMPARISON.md: Detailed design comparison
- PRESENTATION.md: Complete theoretical background
- pcm_loop_transfer_function.m: Source code with comments
