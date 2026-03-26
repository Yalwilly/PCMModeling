# NGSTability Project Progress

**Last updated:** March 26, 2026

## 1. Project Goal

Build a MATLAB-based stability analysis model (NGSTability) for the Next-Gen PCM (Peak Current Mode) buck converter digital control loop. The model must correlate with AMS simulation, support RTL-accurate coefficient mapping between lab Position\[P,I,D\], RTL K1/K2/K3, and effective Kp/Ki/Kd, and provide multi-load PID optimization via Nelder-Mead and Particle Swarm methods. The project encompasses small-signal Bode analysis, large-signal nonlinear time-domain prediction, and load-step transient metrics.

---

## 2. File Inventory

| File | Purpose | Status |
|------|---------|--------|
| `NGSTability_V5.m` | RTL-accurate model, lab position framework, large-signal predictor, laplace/z domain | Superseded |
| `NGSTability_V6.m` | Optimization-only PID tuning (Nelder-Mead + PSO), multi-load sweep | Superseded |
| `NGSTability_V7.m` | Merged analysis + optimization with all bug fixes and performance improvements | **Current** |

---

## 3. Version Evolution

### V3 — Baseline Manual Analysis

- **PID gains:** Fixed `Kp = 50`, `Ki = 8e8`, `Kd = 0` (no derivative)
- **Operating point:** `R = 400 mΩ` load resistance
- **Sensing:** `Rsense = 8 kΩ`, `Resr = 10 mΩ`
- **Digital scaling:** `G_digital = 1/64`, K1/K2/K3 derived manually
- **Functions (3):** `k123_to_pid`, `pid_to_k123`, `regcheck`
- **Bug fixes applied:**
  - K2/K3 velocity-form `/Ts` scaling was missing
  - `Hdc` parenthesis bug: `(1/1 + ...)` corrected to `1/(1 + ...)`

### V4 — Gain Compensation + Load Step

- **PID gains scaled for G_digital:** `Kp = 70/G_digital = 4480`, `Ki = 20e6/G_digital`, `Kd = 2e-6/G_digital`
- **Operating point:** `R = 1 mΩ` (extreme load test)
- **New features:**
  - Blanking delay `Tblank = 15.2 ns` modeled with Padé approximation
  - ZOH model (`Hzoh`) added in ADC path
  - DAC includes `Vref` scaling: `Hdac = Vref / (1 + s/wp_dac)`
  - Load step response analysis section
- **Delays:** `Td = 4 ns`, `Td_ADC = 3.9 ns`
- **Functions:** None (pure script)

### V5 — RTL-Accurate Model + Lab Position Framework + Large-Signal Predictor

Major rewrite with the following additions:

- **ADC/DAC = 1** for small-signal; only `>>6` digital scaling affects PID gains
- **Lab position framework:** `P_cmd`/`I_cmd`/`D_cmd` → K1/K2/K3 mapping
- **Controller domain selector:** `'laplace'` or `'z'`
- **Updated sensing:** `Rsense = 4 kΩ` (reduced from 8k), `Resr = 3 mΩ` (reduced from 10m)
- **Updated delays:** `Tblank = 12 ns`, `Td = 8 ns`, `Td_ADC = 5 ns`
- **Large-signal predictor:** RTL-accurate PID with:
  - ADC quantization (4-bit, 0.36–0.44 V window)
  - PID accumulator preclamp (MIN_VALUE / MAX_VALUE)
  - 2048 midscale offset for unsigned 12-bit output
  - DAC output limiting (NMOS Vth cap at 1.6 V)
  - 500 kHz analog DAC filter
- **Target gains:** `Kp = 18.3`, `Ki = 7.02e6`, `Kd = 6.24e-8`
- **Functions (12):** `k123_to_pid`, `pid_to_k123`, `lab_position_to_pid`, `lab_position_to_shifted_pid`, `regcheck`, `force_bode_phase_ylim`, `apply_binary_yaxis`, `signed_code_to_bin`, `run_large_signal_predictor`, `quantize_adc_code`, `k123_to_lab_position`, `lab_position_to_k123`

### V6 — Optimization-Only PID Tuning (Current)

- **Stripped all plotting / Bode / step-response;** purely an optimization engine
- **Optimizers:**
  - Nelder-Mead multi-start (24 random starts + 1 centroid)
  - PSO (swarm = 40, iterations = 80) with optional NM hybrid refinement
- **Multi-load sweep:** 9 operating points from 1 mA to 3 A
- **Coefficient limits widened:** `K1_MAG_BITS = 19`, `K2_MAG_BITS = 19`, `K3_MAG_BITS = 12`
- **Load-step transient metrics** added to cost function (slew, settling, undershoot)
- **Output:** Prints lab programming command: `ChipTC.FullChip.Power.SetPIDCofficients(1, P, I, D)`
- **Functions (10):** `simple_optimize_pid`, `simple_optimize_pid_nm`, `simple_optimize_pid_pso`, `pid_margin_cost_multiload`, `evaluate_pid_candidate_multiload`, `build_pcm_plant_at_iload`, `evaluate_loadstep_response_metric`, `k123_to_pid`, `lab_position_to_pid`, `lab_position_to_k123`, `ternary`

---

## 4. System Parameters (V6 Current Values)

### Converter

| Parameter | Value | Description |
|-----------|-------|-------------|
| `VIN` | 3.3 V | Input voltage |
| `VOUT` | 0.8 V | Output voltage |
| `D` | 0.2424 | Duty cycle (VOUT/VIN) |
| `Lind` | 0.47 µH | Inductor |
| `C` | 22 µF | Output capacitance |
| `Resr` | 3 mΩ | ESR |
| `fs` | 6 MHz | Switching frequency |
| `Ts` | 166.7 ns | Switching period |

### Digital Controller

| Parameter | Value | Description |
|-----------|-------|-------------|
| `fdig` | 256 MHz | Digital clock frequency |
| `Tctrl` | 3.906 ns | Digital clock period |
| `SHIFT_BITS` | 6 | RTL right-shift (`>>6`) |
| `G_digital` | 1/64 = 0.015625 | Digital scaling factor |

### Current Sensing & Slope Compensation

| Parameter | Value | Description |
|-----------|-------|-------------|
| `GI` | (1/640)×(10/64)×(1/4) | Current gain chain |
| `Rsense` | 4 kΩ | Sense resistance |
| `Ri` | GI × Rsense | Effective current gain |
| `se` | 0.7e6 V/s | Slope compensation ramp |
| `sn` | (VIN − VOUT) / L | On-time inductor slope |
| `mc` | 1 + se/sn | Slope compensation factor |

### ADC

| Parameter | Value | Description |
|-----------|-------|-------------|
| `N_ADC` | 4 bits | ADC resolution (16 levels) |
| `VADC_MIN` | 0.36 V | ADC window lower bound |
| `VADC_MAX` | 0.44 V | ADC window upper bound |
| `V_FS_ADC` | 80 mV | ADC full-scale range |
| Resolution | 5 mV/LSB | Per-code voltage step |

### DAC

| Parameter | Value | Description |
|-----------|-------|-------------|
| `N_DAC` | 12 bits | DAC resolution (4096 levels) |
| `V_FS_DAC` | 1.2 V | DAC full-scale voltage |
| NMOS max | 1.6 V | VDD − Vth output limit |
| `wp_dac` | 2π × 500 kHz | DAC analog filter pole |

### Delays & Feedback

| Parameter | Value | Description |
|-----------|-------|-------------|
| `Td` | 8 ns | Computational delay |
| `Td_ADC` | 5 ns | ADC delay |
| `Tblank` | 15 ns | Blanking delay |
| `beta` | 0.5 | Feedback divider ratio |
| `H` | 0.5 | Feedback gain (= beta) |

---

## 5. RTL Coefficient Framework

### Conversion Formulas

**Lab Position → K1/K2/K3:**

```
K1 =  P + I + D
K2 = -P     - 2D
K3 =          D
```

**K1/K2/K3 → PID (pre-shift):**

```
Kp = -K2 - 2·K3
Ki = (K1 + K2 + K3) / Tctrl
Kd =  K3 · Tctrl
```

**Effective PID gains (post >>6 shift):**

```
Kp_eff = Kp × G_digital
Ki_eff = Ki × G_digital
Kd_eff = Kd × G_digital
```

### RTL PID Recursion

```
u[n] = u[n-1] + K1·e[n] + K2·e[n-1] + K3·e[n-2]
output = (u >> 6) + 2048
```

- **Preclamp limits:** `MIN_VALUE = (-2048) << 6 = -131072`, `MAX_VALUE = (2047) << 6 = 131008`
- **Midscale offset:** 2048 (unsigned 12-bit output, half-scale = no correction)

### Example Conversions

Using `Tctrl = 1/128 MHz = 7.8125 ns`, `G_digital = 1/64`:

| | K1 | K2 | K3 | Kp_eff | Ki_eff | Kd_eff |
|---|---|---|---|---|---|---|
| **Set 1** | 3008 | −3940 | 936 | 32.3125 | 8.0e6 | 1.14e-7 |
| **Set 2** | 4261 | −5279 | 1023 | 50.516 | 1.0e7 | 1.25e-7 |

> **Note:** V6 code uses `fdig = 256 MHz`; the latest AMS correlation used `fdig = 128 MHz`. Verify which value matches the current RTL before interpreting coefficients.

---

## 6. Optimization Framework

### Search Space (log10-scaled)

| Parameter | Lower Bound | Upper Bound |
|-----------|-------------|-------------|
| `Kp` | 0.015625 | 10,000 |
| `Ki` | 4e6 | 5e8 |
| `Kd` | Tctrl × G_digital | 4095 × Tctrl × G_digital |

### Stability Constraints

- Phase margin ≥ 35°
- Gain margin ≥ 6 dB
- K1, K2, K3 must fit within RTL bit-width limits (200-point penalty per overflow)
- Closed-loop must be stable at all load points

### Load Points

```
OPT_I_LOAD_VEC = [0.001, 0.10, 0.30, 0.50, 0.70, 0.90, 1.10, 2, 3] A
```

### Cost Function

```
cost = -(margin_score) + overflow_penalty
```

**Margin score:**

```
score = w_PM_min × PM_min  +  w_GM_min × GM_min
      + w_PM_avg × PM_avg  +  w_GM_avg × GM_avg
      - w_FC    × mean(|log10(fc / fc_target)|)
```

| Weight | Value |
|--------|-------|
| `OPT_W_PM_MIN` | 2 |
| `OPT_W_GM_MIN` | 1 |
| `OPT_W_PM_AVG` | 0.15 |
| `OPT_W_GM_AVG` | 0.10 |
| `OPT_W_FC` | 1 |
| `OPT_W_SPREAD` | 0 (disabled) |

**Load-step terms (when enabled, weight = 12 each):**

- Slew bonus: `min(slew / ref_slew, 10)` — ref = 0.002 V/µs
- Settling penalty: `settle / ref_settle` — ref = 5 µs
- Undershoot penalty: `undershoot / ref_undershoot` — ref = 30 mV

### Optimizer Methods

| Method | Configuration |
|--------|--------------|
| **Nelder-Mead** | 24 random starts + 1 centroid start, `fminsearch` |
| **PSO** | Swarm size = 40, max iterations = 80, optional hybrid NM refinement |

---

## 7. Key Technical Decisions

| Decision | Rationale |
|----------|-----------|
| **Tctrl normalization** | K1/K2/K3 are interpreted with `Tctrl` (digital clock period), NOT `Ts` (switching period). This resolved the small-signal vs. large-signal model mismatch. |
| **ADC/DAC = 1 for small-signal** | Only `G_digital` (>>6) affects PID gains in the linear model. ADC and DAC are separate blocks whose gains cancel in the small-signal loop. |
| **DAC NMOS Vth limit** | Max DAC output capped at `VDD − Vth = 1.6 V`, not 1.8 V, matching silicon behavior. |
| **PID midscale offset** | Unsigned 12-bit output with 2048 midscale: `output = (u >> 6) + 2048`. Zero error → mid-range DAC code. |
| **Laplace/Z domain switch** | `CONTROLLER_DOMAIN` flag enables comparison of continuous-time approximation vs. exact discrete-time PID model. |
| **Preclamp values from RTL** | `MIN_VALUE = (-2048) << 6 = -131072`, `MAX_VALUE = (2047) << 6 = 131008` — matches exact RTL accumulator limits. |

---

## 8. Known Issues

| # | Issue | Impact | Severity |
|---|-------|--------|----------|
| 1 | `pt_cfg` typo in V6 — should be `opt_cfg` for `OPT_ENABLE_LOADSTEP_METRIC` | Load-step metrics silently disabled | **High** |
| 2 | Load-step metrics return `NaN` in some runs | Config fields not fully passed to evaluation function | High |
| 3 | Ki resolution limited to ~4e6 LSB | Coarse integral tuning granularity; proposed `COEF_FRAC_BITS` | Medium |
| 4 | K3 may hit ceiling (4095 at 12 bits) | Limits achievable derivative gain | Medium |
| 5 | 1 mA load point (`R = 800 Ω`) may cause near-singular plant model | Potential numerical instability in `Hdc` denominator | Medium |
| 6 | `OPT_W_SPREAD = 0` disables margin uniformity penalty | Optimizer may find solutions with wildly varying margins across loads | Low |
| 7 | Padé approximation degrades near `fs/2` | Margin calculations may be inaccurate at high frequencies | Low |
| 8 | `NGSOptimizePID.m` planned as standalone file but not yet created | Analysis and optimization not cleanly separated | Low |
| 9 | `fdig` mismatch: V6 uses 256 MHz, latest AMS correlation used 128 MHz | Coefficient interpretation depends on correct clock value | **High** |

---

## 9. Next Steps (Prioritized)

1. **Fix `pt_cfg` → `opt_cfg` typo** in V6 to re-enable load-step metrics
2. **Resolve `NaN` load-step metrics** — ensure all config fields are passed correctly
3. **Verify `Tctrl` value** (128 MHz vs 256 MHz) against current RTL to ensure coefficient interpretation is correct
4. **Add overshoot penalty** (`OPT_W_LOADSTEP_OVERSHOOT`) to cost function
5. **Implement `COEF_FRAC_BITS`** for improved Ki resolution when RTL supports fractional bits
6. **Evaluate widening K3** beyond 12 bits if derivative gain is still ceiling-limited
7. **Create standalone `NGSOptimizePID.m`** — clean separation of analysis vs. optimization code
8. **Re-enable `OPT_W_SPREAD`** to enforce margin uniformity across load points
9. **Update `README.md`** with Phase 2 pointer to this document

---

## 10. Milestone Timeline

| Date | Milestone |
|------|-----------|
| Feb 2026 | Phase 1 complete: `pcm_loop_transfer_function.m`, Type-II/III Bode, Hc vs Hdig comparison, documentation |
| Feb–Mar 2026 | V3–V5: Iterative model refinement, RTL-accurate PID, lab position mapping, large-signal predictor |
| Mar 2026 | V6: Multi-load PID optimization with PSO/NM, load-step metrics, coefficient widening |
| Mar 26, 2026 | This progress document created |
