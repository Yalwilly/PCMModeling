# PCMModeling

Peak Current Mode DC-DC Transfer function modeling and compensation network analysis.

## Project Overview

This repository contains comprehensive analysis tools and documentation for designing compensation networks for Peak Current Mode (PCM) controlled DC-DC converters. The project includes:

1. **MATLAB/Octave code** for loop transfer function analysis and Bode plot generation
2. **Detailed comparison** of compensation design approaches (Erickson vs. reference papers)
3. **Analysis of digital implementation effects** (Hc theoretical vs. Hdig digital)
4. **Complete presentation** with theoretical explanations and results

## Contents

- `pcm_loop_transfer_function.m` - Main MATLAB/Octave script for transfer function analysis
- `COMPENSATION_COMPARISON.md` - Detailed comparison of design methodologies
- `PRESENTATION.md` - Complete presentation with problem statement, theory, and results
- `loop_transfer_function_bode.png` - Generated Bode plots of loop transfer functions
- `compensation_comparison_hc_vs_hdig.png` - Comparison of continuous vs. digital compensation

## Features

### MATLAB Code Capabilities

- **Power Stage Modeling**: Configurable Buck/Boost/Buck-Boost converter parameters
- **Type II Compensator**: Single pole-zero pair design based on Erickson's methodology
- **Type III Compensator**: Double pole-zero pair for higher bandwidth applications
- **Digital Compensation Modeling**: Includes ADC, PID, and DAC delays
- **Bode Plot Generation**: Publication-quality magnitude and phase plots
- **Stability Analysis**: Automatic calculation of crossover frequency and phase margin

### Analysis Highlights

1. **Systematic Design Approach**: Based on Erickson & Maksimovic's "Fundamentals of Power Electronics"
2. **Digital Implementation Effects**: Quantifies phase loss due to sampling delays and ZOH
3. **Comparison Framework**: Erickson's analytical method vs. empirical reference paper approaches
4. **Practical Guidelines**: Step-by-step design procedures for both analog and digital implementations

## Quick Start

### Requirements

- MATLAB R2016b or later, OR
- GNU Octave 4.0 or later (free, open-source alternative)

### Running the Analysis

```bash
# Using MATLAB
matlab -batch "pcm_loop_transfer_function"

# Using Octave
octave --no-gui pcm_loop_transfer_function.m
```

### Output

The script generates:
- Console output with system parameters and stability margins
- `loop_transfer_function_bode.png` - Bode plots comparing all compensation types
- `compensation_comparison_hc_vs_hdig.png` - Direct comparison of Hc vs Hdig

## Key Results

### Type II Compensator (Conservative Design)
- **Crossover Frequency**: 0.86 kHz
- **Phase Margin (Hc)**: 109° (Excellent)
- **Phase Margin (Hdig)**: 103° (Excellent)
- **Status**: Stable for both analog and digital implementation

### Type III Compensator (Aggressive Design)
- **Crossover Frequency**: 26.8 kHz
- **Phase Margin (Hc)**: 64° (Good)
- **Phase Margin (Hdig)**: -112° (Unstable)
- **Status**: Requires gain reduction for digital implementation

### Phase Loss Due to Digital Delays
- **Type II**: 6° loss (low crossover frequency)
- **Type III**: 176° loss (high crossover frequency)
- **Conclusion**: Digital delays significantly impact high-bandwidth designs

## Documentation

### COMPENSATION_COMPARISON.md

Comprehensive comparison covering:
- Peak Current Mode control basics
- Erickson's systematic design methodology (Type II and Type III)
- Typical reference paper approaches
- Hc (continuous) vs. Hdig (digital) transfer functions
- Design implications and recommendations

### PRESENTATION.md

22-slide presentation including:
- Problem statement and objectives
- Theoretical foundations
- Design procedures and guidelines
- MATLAB simulation results
- Stability analysis and recommendations

## Design Parameters (Example)

| Parameter | Value | Unit |
|-----------|-------|------|
| Input Voltage | 12 | V |
| Output Voltage | 5 | V |
| Switching Frequency | 100 | kHz |
| Inductance | 10 | μH |
| Capacitance | 100 | μF |
| ESR | 50 | mΩ |
| Load Resistance | 5 | Ω |

## Key Design Equations

### Power Stage Transfer Function
```
         Vout     1 + s·ESR·C
Gvd(s) = ──── · ─────────────────────
          D      1 + s/(ω₀·Q) + s²/ω₀²
```

### Type II Compensator
```
         K · (1 + s/ωz)
Hc(s) = ────────────────
        s · (1 + s/ωp)
```

### Digital Implementation
```
Hdig(s) = Hc(s) · e^(-s·Td) · (1 - e^(-s·Ts))/(s·Ts)
```

## References

1. Erickson, R. W., & Maksimovic, D. (2001). *Fundamentals of Power Electronics* (2nd ed.). Springer.
2. Ridley, R. B. (1990). "A New, Continuous-Time Model for Current-Mode Control." *IEEE Transactions on Power Electronics*.
3. Tang, W., Lee, F. C., & Ridley, R. B. (1993). "Small-Signal Modeling of Average Current-Mode Control." *IEEE Transactions on Power Electronics*.

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests for:
- Additional compensation techniques
- Different converter topologies
- Improved analysis methods
- Documentation enhancements

## License

This project is provided for educational and research purposes.

## Author

PCM Modeling Project - February 2026
 
