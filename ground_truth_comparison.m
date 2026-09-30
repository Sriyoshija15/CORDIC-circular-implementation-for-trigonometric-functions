%% ground_truth_comparison.m -- RTL-vs-golden angle error (headline metric) across the N_ITER sweep,
%% plus RTL/golden-vs-ground-truth RMSE kept for reference (see caveat below)

load('golden_data.mat');   % q_gold: N x 4, float, from mahony_golden.m

N_list = [8 10 12 14 16];

fprintf('\n=== RTL vs Golden angle error (degrees) -- headline fidelity metric ===\n');
fprintf('%6s %8s %8s %8s\n', 'N_ITER', 'mean', 'max', 'final');
for n = N_list
    f = sprintf('rtl_quat_N%d.csv', n);
    if ~exist(f, 'file')
        fprintf('%6d   missing file: %s\n', n, f);
        continue;
    end
    r = csvread(f);
    if size(r,1) ~= 1000
        fprintf('%6d   wrong row count: %d (expected 1000) -- re-simulate this N_ITER\n', n, size(r,1));
        continue;
    end
    q_rtl = r(:,2:5) / 16384;
    q_rtl = q_rtl ./ sqrt(sum(q_rtl.^2, 2));

    ang = 2*acosd(min(abs(sum(q_rtl .* q_gold, 2)), 1));
    fprintf('%6d %8.2f %8.2f %8.2f\n', n, mean(ang), max(ang), ang(end));
end

fprintf('\nNOTE: ground-truth RMSE below is NOT a reliable ranking metric for this sweep.\n');
fprintf('A frozen/broken filter stays near its seed orientation and can score BETTER\n');
fprintf('than a correctly-tracking filter, since the golden model itself has known yaw\n');
fprintf('drift (no magnetometer). Use the RTL-vs-golden table above as the headline result;\n');
fprintf('the table below is kept only for full transparency / appendix reporting.\n');

%% ---- secondary: RTL vs ground truth RMSE, per N_ITER (for reference only) ----
gt_data = csvread('ground_truth_euler.csv');
gt_euler_deg = rad2deg(gt_data(1:1000, 2:4));

fprintf('\n=== [reference only] RTL vs Ground Truth RMSE (degrees) ===\n');
fprintf('%6s %8s %8s %8s %8s\n', 'N_ITER', 'Roll', 'Pitch', 'Yaw', 'Overall');
for n = N_list
    f = sprintf('rtl_quat_N%d.csv', n);
    if ~exist(f, 'file'), continue; end
    r = csvread(f);
    if size(r,1) ~= 1000, continue; end

    q_rtl = r(:,2:5) / 16384;
    q_rtl = q_rtl ./ sqrt(sum(q_rtl.^2, 2));

    rtl_euler_deg = zeros(1000,3);
    for k = 1:1000
        rtl_euler_deg(k,:) = rad2deg(quat2euler_manual(q_rtl(k,:)));
    end

    rmse_axis = zeros(1,3);
    for a = 1:3
        d = wrap_angle_deg(rtl_euler_deg(:,a) - gt_euler_deg(:,a));
        rmse_axis(a) = sqrt(mean(d.^2));
    end
    fprintf('%6d %8.3f %8.3f %8.3f %8.3f\n', n, rmse_axis(1), rmse_axis(2), rmse_axis(3), norm(rmse_axis));
end


%% ---- local functions ----

function eul = quat2euler_manual(q)
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));
    sinp  = 2*(qw*qy - qz*qx);
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi/2;
    else
        pitch = asin(sinp);
    end
    yaw   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
    eul = [roll, pitch, yaw];
end

function wrapped = wrap_angle_deg(diff_deg)
    wrapped = mod(diff_deg + 180, 360) - 180;
end