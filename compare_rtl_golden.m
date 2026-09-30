%% compare_rtl_golden.m -- sample-by-sample RTL vs golden comparison

rtl_data = csvread('rtl_quat_output.csv');       % [sample, q0, q1, q2, q3]
golden_data = csvread('golden_quat_output_scaled.csv');  % [sample, q0, q1, q2, q3] (already Q2.14 scaled)

N = min(size(rtl_data,1), size(golden_data,1));

rtl_q1    = rtl_data(1:N, 3);
golden_q1 = golden_data(1:N, 3);
diff_q1   = rtl_q1 - golden_q1;

fprintf('%6s %10s %10s %10s\n', 'sample', 'RTL_q1', 'golden_q1', 'diff');
for k = 1:10:N   % print every 10th sample to keep it readable
    fprintf('%6d %10d %10d %10d\n', k-1, rtl_q1(k), golden_q1(k), diff_q1(k));
end

figure;
plot(0:N-1, rtl_q1, 'b-', 'LineWidth', 1.5); hold on;
plot(0:N-1, golden_q1, 'r--', 'LineWidth', 1.5);
xlabel('Sample'); ylabel('q1 (Q2.14 scaled)');
legend('RTL', 'Golden'); title('q1 trajectory: RTL vs Golden');
grid on;

figure;
plot(0:N-1, diff_q1, 'k-');
xlabel('Sample'); ylabel('RTL - Golden (q1)');
title('q1 divergence over time');
grid on;

fprintf('\nMax abs diff: %d at sample %d\n', max(abs(diff_q1)), find(abs(diff_q1)==max(abs(diff_q1)),1)-1);
fprintf('Diff at sample 0: %d, sample %d (last): %d\n', diff_q1(1), N-1, diff_q1(end));