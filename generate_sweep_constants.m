function generate_sweep_constants(N_ITER, WORD_LEN, Kp, Ki)
% Usage: generate_sweep_constants(16, 16, 1.9999, 0.05)
% Prints ready-to-paste Verilog constants for the given sweep point.

FRAC_BITS = WORD_LEN - 2;
SCALE = 2^FRAC_BITS;

fprintf('\n=== Sweep point: N_ITER=%d, WORD_LEN=%d (FRAC_BITS=%d, SCALE=%d) ===\n\n', ...
    N_ITER, WORD_LEN, FRAC_BITS, SCALE);

%% Circular gain and angle table
angle_c = zeros(N_ITER,1);
K_c = 1;
for k = 0:N_ITER-1
    angle_c(k+1) = atan(2^-k);
    K_c = K_c * sqrt(1 + 2^(-2*k));
end
AN_INV = 1/K_c;

fprintf('// --- CIRCULAR ANGLE ROM (paste into cordic_core.v, mode 2''b01 case) ---\n');
for k = 0:N_ITER-1
    val = round(angle_c(k+1)*SCALE);
    if val > 32767 || val < -32768
        fprintf('                    %2d: angle_val = %d''sd%d; // WARNING: overflow, WORD_LEN too small\n', k, WORD_LEN, val);
    else
        fprintf('                    %2d: angle_val = %d''d%d;\n', k, WORD_LEN, val);
    end
end
fprintf('\n// AN_INV_Q (circular gain compensation):\n');
fprintf('parameter AN_INV_Q = %d''d%d;  // 1/K_circular = %.6f\n\n', WORD_LEN, round(AN_INV*SCALE), AN_INV);

%% Hyperbolic gain and angle table (with repeats at standard indices 4, 13, 40...)
repeat_idx = [4, 13, 40];
K_h = 1;
angle_h = [];
idx_list = [];
k = 1;
rep_count = zeros(1, max(repeat_idx)+1);
while k <= N_ITER
    angle_h(end+1) = atanh(2^-k);
    idx_list(end+1) = k;
    K_h = K_h * sqrt(1 - 2^(-2*k));
    if ismember(k, repeat_idx) && rep_count(k) == 0
        rep_count(k) = 1;
        % repeat this same k on the next iteration (don't increment k)
        angle_h(end+1) = atanh(2^-k);
        idx_list(end+1) = k;
        K_h = K_h * sqrt(1 - 2^(-2*k));
        k = k + 1;
    else
        k = k + 1;
    end
end
AH_INV = 1/K_h;

fprintf('// --- HYPERBOLIC ANGLE ROM (paste into cordic_core.v, mode 2''b11 case) ---\n');
fprintf('// NOTE: repeat-iteration control logic in cordic_core.v must match repeat_idx used here\n');
printed_idx = containers.Map('KeyType','double','ValueType','double');
for n = 1:length(idx_list)
    k = idx_list(n);
    val = round(atanh(2^-k)*SCALE);
    if ~isKey(printed_idx, k)
        fprintf('                    %2d: angle_val = %d''d%d;\n', k, WORD_LEN, val);
        printed_idx(k) = 1;
    end
end
fprintf('// AH_INV (hyperbolic gain, 1/K_hyperbolic = %.6f) -- not currently wired into any module,\n', AH_INV);
fprintf('// add as a parameter if you use hyperbolic mode inside a gain-corrected pipeline\n\n');

%% Mahony gains, rescaled to this WORD_LEN
KP_Q = round(Kp * SCALE);
KI_Q = round(Ki * SCALE);
fprintf('// --- MAHONY GAINS (paste into mahony_filter.v parameter list) ---\n');
if KP_Q > 32767 || KP_Q < -32768
    fprintf('parameter KP_Q = ... // WARNING: Kp=%.4f overflows WORD_LEN=%d, max representable is %.4f\n', Kp, WORD_LEN, (2^(WORD_LEN-1)-1)/SCALE);
else
    fprintf('parameter KP_Q = %d''d%d;  // Kp = %.4f\n', WORD_LEN, KP_Q, Kp);
end
fprintf('parameter KI_Q = %d''d%d;  // Ki = %.4f\n\n', WORD_LEN, KI_Q, Ki);

fprintf('// --- ONE_Q (paste into vector_normalize.v instantiations'' parameter, e.g. reciprocal step) ---\n');
fprintf('parameter ONE_Q = %d''d%d;  // 1.0 in this format\n\n', WORD_LEN, SCALE);

fprintf('// --- FRAC_BITS override (only needed if not using default WORD_LEN-2) ---\n');
fprintf('parameter FRAC_BITS = %d;\n\n', FRAC_BITS);

fprintf('=== End of sweep point %d/%d ===\n\n', N_ITER, WORD_LEN);
end