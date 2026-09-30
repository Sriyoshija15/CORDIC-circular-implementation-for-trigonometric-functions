%% angular_error_check.m -- proper quaternion comparison via angular distance

rtl_data = csvread('rtl_quat_output.csv');
golden_data = csvread('golden_quat_output_scaled.csv');

N = min(size(rtl_data,1), size(golden_data,1));

ang_error_deg = zeros(N,1);

for k = 1:N
    q_rtl = rtl_data(k, 2:5) / 16384;      % unscale back to unit quaternion
    q_gold = golden_data(k, 2:5) / 16384;

    q_rtl = q_rtl / norm(q_rtl);            % renormalize (scaling/rounding may have drifted norm slightly)
    q_gold = q_gold / norm(q_gold);

    dotp = abs(sum(q_rtl .* q_gold));       % abs() handles the q vs -q ambiguity (same rotation)
    dotp = min(dotp, 1.0);                  % guard against acos domain error from rounding
    ang_error_deg(k) = 2 * acosd(dotp);     % angular difference between the two orientations
end

figure;
plot(0:N-1, ang_error_deg, 'b-', 'LineWidth', 1.5);
xlabel('Sample'); ylabel('Angular error (degrees)');
title('RTL vs Golden: orientation angular error');
grid on;

fprintf('Angular error: mean=%.3f deg, max=%.3f deg at sample %d, final=%.3f deg\n', ...
    mean(ang_error_deg), max(ang_error_deg), find(ang_error_deg==max(ang_error_deg),1)-1, ang_error_deg(end));