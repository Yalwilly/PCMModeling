%% generate_presentation.m
% Generates NGSTability_Presentation.pptx using PowerPoint COM automation
% Run in MATLAB: >> generate_presentation

fprintf('Creating NGSTability presentation...\n');

%% ---- Configuration ----
outFile = fullfile(pwd, 'NGSTability_Presentation.pptx');

% Colors (RGB)
DARK_BLUE = [0 51 102];
WHITE     = [255 255 255];
GRAY      = [64 64 64];
TBL_HDR   = [0 80 144];
TBL_ROW1  = [232 240 250];
TBL_ROW2  = [255 255 255];

%% ---- Launch PowerPoint ----
ppt = actxserver('PowerPoint.Application');
ppt.Visible = 1;
pres = ppt.Presentations.Add;

% Set widescreen 13.333 x 7.5
pres.PageSetup.SlideWidth = 13.333 * 72;  % points
pres.PageSetup.SlideHeight = 7.5 * 72;

slideW = pres.PageSetup.SlideWidth;
slideH = pres.PageSetup.SlideHeight;

slideIdx = 0;

% Get the blank layout from the first slide master
blankLayout = pres.SlideMaster.CustomLayouts.Item(pres.SlideMaster.CustomLayouts.Count);
% Find a truly blank layout (fewest placeholders)
for iLay = 1:pres.SlideMaster.CustomLayouts.Count
    lay = pres.SlideMaster.CustomLayouts.Item(iLay);
    if lay.Name == "Blank" || contains(char(lay.Name), 'Blank', 'IgnoreCase', true)
        blankLayout = lay;
        break;
    end
end

%% ============================================================
%% HELPER FUNCTIONS (nested at bottom, used via function handles)
%% ============================================================

%% ---- TITLE SLIDE ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
set_slide_bg(slide, DARK_BLUE);

add_textbox(slide, 0.8, 2.0, 11.7, 2.0, ...
    {'NGSTability — Next Gen PCM Buck Converter', 'Stability Analysis & PID Optimization'}, ...
    36, WHITE, true);

add_textbox(slide, 0.8, 4.2, 11.7, 1.5, ...
    {'Digital Control Loop Modeling  |  RTL-Accurate Coefficients  |  Global Optimization  |  AMS Correlation'}, ...
    18, [187 204 221], false);

%% ---- MOTIVATION SLIDE ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
set_slide_bg(slide, WHITE);
add_title_bar(slide, slideW, 'Why NGSTability?  —  Motivation', DARK_BLUE, WHITE);

bullets = {
    'New architecture: moved from voltage-mode analog compensation to PCM buck with digital PID'
    '   → Entirely different control loop — no legacy model or design methodology to reuse'
    ''
    'The gaps we faced:'
    '   • No accurate model to predict which K1/K2/K3 PID sets produce a stable loop'
    '   • PID tuning was done manually by trial-and-error — both in lab and in AMS simulation'
    '   • Even after finding a stable set, we had no visibility into actual PM/GM margins'
    '   • Pre-silicon: each AMS trial takes hours — searching for a stable set is painfully slow'
    ''
    'NGSTability approach:'
    '   1.  Model the full closed-loop system in MATLAB (Laplace domain)'
    '        → Bode plot → extract GM, PM, crossover frequency in milliseconds'
    '   2.  Validate: if MATLAB predicts stable, it should correlate with lab and AMS measurements'
    '   3.  Once the model is trusted, run global optimization over all possible PID sets'
    '        → Find the best K1/K2/K3 under stability and hardware constraints automatically'
    '   4.  Replaces days of manual AMS iteration with minutes of automated search'
};
add_bullet_box(slide, 0.8, 1.4, 11.5, 5.8, bullets, 15, GRAY);

%% ---- SLIDE 1.1: PCM Feedback Loop Architecture ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, '1. Small-Signal Modeling — PCM Feedback Loop Architecture', DARK_BLUE, WHITE);

headers = {'Block', 'Transfer Function', 'Key Parameters'};
rows = {
    'PCM Plant',            'Gvc = Hdc · Fp · Fh',           'L=0.47µH, C=22µF, Resr=3mΩ';
    'Digital PID',          'Hpid = Kp + Ki/s + Kd·s/(Tf·s+1)', 'After >>6 shift';
    'ADC delay',            'Padé(5ns, order 2)',              '4-bit, 80mV window';
    'DAC filter',           '1 / (1 + s/ωdac)',                '500 kHz pole, 12-bit';
    'Blanking + Comp delay','Padé(15ns + 8ns)',                '';
    'Feedback',             'H = β = 0.5',                     'Resistive divider';
};
add_table(slide, 1.5, 0.9, 11.5, headers, rows, TBL_HDR, TBL_ROW1, TBL_ROW2, WHITE);

%% ---- SLIDE 1.2: PCM Plant Model ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, '1. Small-Signal Modeling — PCM Plant: Control-to-Output', DARK_BLUE, WHITE);

bullets = {
    'Gvc(s) = Hdc · Fp(s) · Fh(s)'
    'Hdc = (R/Ri) · 1/(1 + R·Ts·(mc·D''−0.5)/L)  →  DC gain, load-dependent'
    'Fp(s) = (1 + s·C·Resr) / (1 + s/ωp)  →  output filter pole + ESR zero'
    'Fh(s) = 1 / (1 + s/(ωn·Qp) + s²/ωn²)  →  sampling double pole at fs/2'
    'Slope compensation: mc = 1 + se/sn,  se = 0.7 MV/s'
    'Plant gain varies significantly with load current (1 mA – 3 A)'
    '→ Must evaluate stability across the full load range, not just one operating point'
};
add_bullet_box(slide, 0.8, 1.4, 11.5, 5.5, bullets, 16, GRAY);

%% ---- SLIDE 2.1: Closed-Form Components ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, '2. Closed-Form Loop Components — Transfer Functions', DARK_BLUE, WHITE);

bullets = {
    'Analog reference compensator (Type-II):'
    '   Hc(s) = (ωp1/s) · (1 + s/ωz,comp) / (1 + s/ωp2)'
    ''
    'Digital PID compensator (Laplace domain):'
    '   Hpid(s) = Kp + Ki/s + Kd·s / (Tf·s + 1)'
    ''
    'Complete digital compensator chain:'
    '   Hdig(s) = G_ADC · Hadc(s) · Hpid(s) · G_digital · G_DAC · Hdac(s) · Hblank(s) · e^(-sTd)'
    ''
    'Loop gains:'
    '   Lc(s)    = Hc(s) · Gvc(s) · H                (analog reference)'
    '   Lloop(s) = Hdig(s) · Gvc(s) · H              (digital loop gain)'
    ''
    'Closed-loop transfer function:'
    '   Tloop(s) = Lloop(s) / (1 + Lloop(s))         (feedback(Lloop, 1))'
    ''
    'Delays → 2nd-order Padé   |   G_digital = 1/64   |   ADC/DAC gains = 1'
};
add_bullet_box(slide, 0.8, 1.4, 11.5, 5.5, bullets, 15, GRAY);

%% ---- SLIDE 2.2: Phase Budget ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, '2. Phase Budget at Crossover (fc = 600 kHz)', DARK_BLUE, WHITE);

headers = {'Block', 'Phase Contribution', 'Notes'};
rows = {
    'Plant × H',              '~ −XX°',       'Load-dependent';
    'ADC delay (5 ns)',        '~ −X°',        'Padé model';
    'Digital PID',             '~ +XX°',       'Phase boost from zeros';
    'DAC filter (500 kHz)',    '~ −X°',        'Single pole';
    'Blanking delay (15 ns)',  '~ −X°',        'Padé model';
    'Comp delay (8 ns)',       '~ −X°',        'Padé model';
    'TOTAL',                   '~ −XXX°',      '';
    'Phase Margin',            'PM = Total + 180°', 'Target ≥ 35°';
};
add_table(slide, 1.5, 0.9, 11.5, headers, rows, TBL_HDR, TBL_ROW1, TBL_ROW2, WHITE);

%% ---- SLIDE 3.1: RTL Coefficient Framework ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, '3. RTL-Accurate Coefficient Mapping', DARK_BLUE, WHITE);

bullets = {
    'Three equivalent representations of the same controller:'
    '   Lab Position [P, I, D]  ↔  RTL [K1, K2, K3]  ↔  Effective [Kp, Ki, Kd]'
    ''
    'Lab → RTL:   K1 = P+I+D,   K2 = −P−2D,   K3 = D'
    'RTL → PID:   Kp = −K2−2K3,  Ki = (K1+K2+K3)/Tctrl,  Kd = K3·Tctrl'
    'Effective:    Kp_eff = Kp · G_digital,  Ki_eff = Ki · G_digital,  Kd_eff = Kd · G_digital'
    ''
    'RTL PID recursion (velocity form):'
    '   u[n] = u[n−1] + K1·e[n] + K2·e[n−1] + K3·e[n−2]'
    '   output = (u >> 6) + 2048'
    ''
    'All conversions are invertible — any representation maps to any other'
};
add_bullet_box(slide, 0.8, 1.4, 11.5, 5.5, bullets, 16, GRAY);

%% ---- SLIDE 3.2: Quantization Constraints ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, '3. Hardware Constraints on PID Design Space', DARK_BLUE, WHITE);

headers = {'Coefficient', 'Bit Width', 'Range', 'Constraint'};
rows = {
    'K1',              '20-bit signed',   '±524,287',               '|K1| ≤ 2^19 − 1';
    'K2',              '20-bit signed',   '±524,287',               '|K2| ≤ 2^19 − 1';
    'K3',              '11-bit signed',   '±1,023',                 '|K3| ≤ 2^10 − 1';
    'PID accumulator', 'Pre-clamp',       '−131,072 to +131,008',   'RTL MIN/MAX_VALUE';
    'PID output',      '12-bit unsigned', '0 – 4,095',              'After >>6 + 2048 offset';
    'Ki resolution',   '~4×10⁶ LSB',     'Coarse steps',            'Integer I_cmd quantization';
};
add_table(slide, 1.5, 0.9, 11.5, headers, rows, TBL_HDR, TBL_ROW1, TBL_ROW2, WHITE);

%% ---- SLIDE 4.1: Optimization Architecture ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, '4. Global PID Optimization — Architecture', DARK_BLUE, WHITE);

bullets = {
    'Goal: Find [Kp, Ki, Kd] maximizing stability + transient performance across 9 loads'
    ''
    'Search space (log10 scaled):'
    '   Kp ∈ [0.016, 10000]    Ki ∈ [4×10⁶, 5×10⁸]    Kd ∈ [Kd_min, Kd_max]'
    ''
    'Load sweep:  Iload = [1mA, 100mA, 300mA, 500mA, 700mA, 900mA, 1.1A, 2A, 3A]'
    ''
    'Two-stage optimizer:'
    '   1. PSO (Particle Swarm) — 40 particles × 80 iterations → global exploration'
    '   2. Nelder-Mead hybrid refinement → local polishing of best PSO result'
    ''
    'Hard constraints:'
    '   • Closed-loop stable at ALL load points'
    '   • PM ≥ 35°,  GM ≥ 6 dB at every load'
    '   • K1/K2/K3 within RTL bit-width (200-point penalty per overflow)'
};
add_bullet_box(slide, 0.8, 1.4, 11.5, 5.5, bullets, 16, GRAY);

%% ---- SLIDE 4.2: Cost Function ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, '4. Multi-Metric Cost Function', DARK_BLUE, WHITE);

bullets = {
    'cost = −score + overflow_penalty'
    ''
    'Score = stability margins + load-step transient (all in one sum):'
    ''
    '   score = 2·PM_min + 1·GM_min + 0.15·PM_avg + 0.1·GM_avg − 1·|log₁₀(fc/fc_target)|'
    '         + 12 · min(slew / 0.002, 10)             ← reward fast recovery'
    '         − 12 · t_settle / 5µs                     ← penalize slow settling'
    '         − 12 · ΔV_undershoot / 20mV               ← penalize deep voltage dip'
    ''
    'All terms contribute to a single score — optimizer maximizes the total'
    ''
    'Margin caps:  PM capped at 85°,  GM capped at 40 dB'
    'Overflow penalty:  +200 per K1/K2/K3 exceeding RTL bit-width'
    ''
    'Convergence visualization: cost vs iteration with PSO → fminsearch transition marker'
};
add_bullet_box(slide, 0.8, 1.4, 11.5, 5.5, bullets, 15, GRAY);

%% ---- SLIDE 5.1: Digital vs Analog Comparison ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, '5. Optimized Digital Loop vs Ideal Analog Compensator', DARK_BLUE, WHITE);

headers = {'Metric', 'Analog (Ideal Hc)', 'Digital (Optimized)', 'Notes'};
rows = {
    'Compensator type',  'Type-II continuous',    'Velocity PID (K1/K2/K3)', '';
    'Phase Margin',      'XX°',                   'XX°',                     'Fill from simulation';
    'Gain Margin',       'XX dB',                 'XX dB',                   'Fill from simulation';
    'Crossover freq',    '600 kHz',               '~XXX kHz',                'Target = fs/10';
    'Phase loss sources','None',                   'ADC/DAC/blanking delays', '10–20° typical loss';
    'Quantization',      'Infinite resolution',    'Integer K1/K2/K3',        'Finite bit-width';
    'Load robustness',   'Single operating pt',    '9 loads simultaneously',  '1mA to 3A';
};
add_table(slide, 1.5, 0.9, 11.5, headers, rows, TBL_HDR, TBL_ROW1, TBL_ROW2, WHITE);

%% ---- SLIDE 5.2: 3D Stability Map ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, '5. 3D Stability Design Space Visualization', DARK_BLUE, WHITE);

bullets = {
    'Axes: Kd (X),  Ki (Y),  Kp (Z)  — log-scaled'
    ''
    'GREEN = stable at ALL 9 load points (PM ≥ 35°, GM ≥ 6 dB)'
    'RED = unstable or insufficient margins at any load point'
    'BLUE STAR = optimizer''s selected solution'
    ''
    'Key insights:'
    '   • The stable region is a bounded volume — not all PID combinations work'
    '   • Optimizer solution sits inside the stable region, near the best-margin boundary'
    '   • Boundaries shift with load current — multi-load check is essential'
    ''
    '2D projections (Kd-Ki, Kd-Kp, Ki-Kp) show the stable envelope for each pair'
    '   → Green if stable for ANY value in the collapsed axis'
    ''
    'Grid: 25 × 25 × 15 = 9,375 points evaluated'
};
add_bullet_box(slide, 0.8, 1.4, 11.5, 5.5, bullets, 16, GRAY);

%% ---- SLIDE 6.1: Nonlinear Predictor ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, '6. RTL-Accurate Cycle-by-Cycle Nonlinear Predictor', DARK_BLUE, WHITE);

headers = {'Feature', 'Implementation', 'Details'};
rows = {
    'ADC',              '4-bit quantization',        '0.36–0.44V window, 16 levels, 5mV/LSB';
    'PID accumulator',  'Velocity form + pre-clamp', 'K1·e[n] + K2·e[n−1] + K3·e[n−2]';
    'Shift + offset',   '>>6 then +2048',            'Unsigned 12-bit output';
    'DAC',              '12-bit code → analog',       'NMOS Vth cap at 1.6V (not 1.8V)';
    'DAC filter',       '500 kHz analog pole',        'Discrete IIR at Tctrl';
    'Plant model',      'State-space @ Tctrl',        'Control path + load disturbance (parallel)';
    'Load disturbance', 'Gload = −Zout_OL',          'Passive output impedance only';
    'Load step',        'Configurable profile',       'Delay, rise time, amplitude, width';
};
add_table(slide, 1.5, 0.9, 11.5, headers, rows, TBL_HDR, TBL_ROW1, TBL_ROW2, WHITE);

%% ---- SLIDE 6.2: Validation Pipeline ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, '6. Validation Pipeline: Predictor → AMS → Silicon', DARK_BLUE, WHITE);

bullets = {
    'Tiered validation flow:'
    '   Optimizer  →  Nonlinear Predictor  →  AMS Simulation  →  Silicon'
    ''
    '   (fast, 1000s        (catches quant/          (full circuit       (final'
    '    of evals)           saturation effects)       validation)         proof)'
    ''
    'Metrics compared at each stage:'
    '   • Vout undershoot:   target < 20 mV'
    '   • Recovery slew:     target > 0.002 V/µs'
    '   • Settling time:     target < 5 µs'
    '   • Steady-state Vout: 0.800 V'
    ''
    'Why this tiered approach?'
    '   → Linear model: fast optimization (thousands of evaluations in minutes)'
    '   → Nonlinear predictor: catches effects the linear model misses'
    '   → AMS: full circuit-level validation before silicon commitment'
    '   → Saves weeks of AMS iteration time'
};
add_bullet_box(slide, 0.8, 1.4, 11.5, 5.5, bullets, 16, GRAY);

%% ---- SUMMARY SLIDE ----
slideIdx = slideIdx + 1;
slide = pres.Slides.AddSlide(slideIdx, blankLayout);
add_title_bar(slide, slideW, 'Summary — NGSTability Methodology', DARK_BLUE, WHITE);

bullets = {
    '1.  Small-signal Laplace-domain model of the PCM feedback loop'
    '     → Closed-form plant, digital blocks, delays — all analytically derived'
    ''
    '2.  RTL-accurate coefficient mapping (K1/K2/K3 ↔ Lab P/I/D ↔ Kp/Ki/Kd)'
    '     → Exact quantization and bit-width constraints from RTL'
    ''
    '3.  Global optimization (PSO + Nelder-Mead) across full load range'
    '     → Cost function: PM + GM + crossover + load-step transient metrics'
    ''
    '4.  3D stability map visualization of the feasible PID design space'
    ''
    '5.  Cycle-by-cycle nonlinear predictor for AMS correlation'
    '     → RTL-accurate ADC/DAC quantization, PID clamping, load disturbance'
    ''
    '6.  Validated pipeline: Linear model → Predictor → AMS → Silicon'
    '     → Each tier catches progressively finer effects while managing compute cost'
};
add_bullet_box(slide, 0.8, 1.4, 11.5, 5.5, bullets, 16, GRAY);

%% ---- Save & Close ----
% Use temp path to avoid OneDrive/COM path issues, then copy back
tempOut = fullfile(tempdir, 'NGSTability_Presentation.pptx');
if exist(tempOut, 'file'), delete(tempOut); end
pres.SaveAs(tempOut, 24);  % 24 = ppSaveAsOpenXMLPresentation (.pptx)
copyfile(tempOut, outFile, 'f');
delete(tempOut);
fprintf('Saved: %s\n', outFile);
fprintf('Total slides: %d\n', pres.Slides.Count);

% Leave PowerPoint open so user can see/edit
% pres.Close;
% ppt.Quit;

%% ============================================================
%% LOCAL HELPER FUNCTIONS
%% ============================================================

function set_slide_bg(slide, rgb)
    slide.FollowMasterBackground = 0;
    slide.Background.Fill.Solid;
    slide.Background.Fill.ForeColor.RGB = rgb(1) + rgb(2)*256 + rgb(3)*65536;
end

function add_textbox(slide, leftIn, topIn, widthIn, heightIn, lines, fontSize, rgb, bold)
    left   = leftIn * 72;
    top    = topIn * 72;
    width  = widthIn * 72;
    height = heightIn * 72;
    tb = slide.Shapes.AddTextbox(1, left, top, width, height);
    tf = tb.TextFrame;
    tf.WordWrap = 1;
    for k = 1:numel(lines)
        if k == 1
            para = tf.TextRange;
        else
            para = tf.TextRange.InsertAfter(sprintf('\r'));
        end
        r = tf.TextRange.InsertAfter(lines{k});
        r.Font.Size = fontSize;
        r.Font.Color.RGB = rgb(1) + rgb(2)*256 + rgb(3)*65536;
        r.Font.Bold = bold;
    end
end

function add_title_bar(slide, slideW, titleText, bgRGB, textRGB)
    % Background rectangle
    barH = 1.1 * 72;
    shp = slide.Shapes.AddShape(1, 0, 0, slideW, barH);  % msoShapeRectangle
    shp.Fill.Solid;
    shp.Fill.ForeColor.RGB = bgRGB(1) + bgRGB(2)*256 + bgRGB(3)*65536;
    shp.Line.Visible = 0;

    % Title text
    tb = slide.Shapes.AddTextbox(1, 0.6*72, 0.15*72, 12*72, 0.9*72);
    tf = tb.TextFrame;
    tf.WordWrap = 1;
    tr = tf.TextRange;
    tr.Text = titleText;
    tr.Font.Size = 26;
    tr.Font.Bold = 1;
    tr.Font.Color.RGB = textRGB(1) + textRGB(2)*256 + textRGB(3)*65536;
end

function add_bullet_box(slide, leftIn, topIn, widthIn, heightIn, bullets, fontSize, rgb)
    left   = leftIn * 72;
    top    = topIn * 72;
    width  = widthIn * 72;
    height = heightIn * 72;

    tb = slide.Shapes.AddTextbox(1, left, top, width, height);
    tf = tb.TextFrame;
    tf.WordWrap = 1;

    allText = strjoin(bullets, '\r');
    tf.TextRange.Text = allText;
    tf.TextRange.Font.Size = fontSize;
    tf.TextRange.Font.Color.RGB = rgb(1) + rgb(2)*256 + rgb(3)*65536;
    tf.TextRange.Font.Name = 'Calibri';
end

function add_table(slide, topIn, leftIn, widthIn, headers, rows, hdrRGB, row1RGB, row2RGB, hdrTextRGB)
    nRows = size(rows, 1) + 1;
    nCols = numel(headers);

    left   = leftIn * 72;
    top    = topIn * 72;
    width  = widthIn * 72;
    rowH   = 0.4 * 72;
    height = nRows * rowH;

    tblShape = slide.Shapes.AddTable(nRows, nCols, left, top, width, height);
    tbl = tblShape.Table;

    % Header row
    for c = 1:nCols
        cell = tbl.Cell(1, c);
        cell.Shape.TextFrame.TextRange.Text = headers{c};
        cell.Shape.TextFrame.TextRange.Font.Size = 13;
        cell.Shape.TextFrame.TextRange.Font.Bold = 1;
        cell.Shape.TextFrame.TextRange.Font.Color.RGB = hdrTextRGB(1) + hdrTextRGB(2)*256 + hdrTextRGB(3)*65536;
        cell.Shape.Fill.Solid;
        cell.Shape.Fill.ForeColor.RGB = hdrRGB(1) + hdrRGB(2)*256 + hdrRGB(3)*65536;
    end

    % Data rows
    for r = 1:size(rows, 1)
        if mod(r, 2) == 1
            rowRGB = row1RGB;
        else
            rowRGB = row2RGB;
        end
        for c = 1:nCols
            cell = tbl.Cell(r+1, c);
            cell.Shape.TextFrame.TextRange.Text = rows{r, c};
            cell.Shape.TextFrame.TextRange.Font.Size = 12;
            cell.Shape.TextFrame.TextRange.Font.Color.RGB = 0; % black
            cell.Shape.Fill.Solid;
            cell.Shape.Fill.ForeColor.RGB = rowRGB(1) + rowRGB(2)*256 + rowRGB(3)*65536;
        end
    end
end
