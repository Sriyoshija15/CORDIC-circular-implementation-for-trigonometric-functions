function run_sweep_rmse(N_ITER, WORD_LEN)
    rtl_data    = csvread('rtl_quat_output.csv');
    golden_data = csvread('golden_quat_output_scaled.csv');
    gt_data     = csvread('ground_truth_euler.csv');

    N = min([size(rtl_data,1), size(golden_data,1), size(gt_data,1)]);

    rtl_euler_deg = zeros(N,3);
    gt_euler_deg  = rad2deg(gt_data(1:N, 2:4));

    for k = 1:N
        q_rtl = rtl_data(k, 2:5) / 16384;
        q_rtl = q_rtl / norm(q_rtl);
        rtl_euler_deg(k,:) = rad2deg(quat2euler_manual(q_rtl));
    end

    rmse_axis = zeros(1,3);
    for a = 1:3
        d = wrap_angle_deg(rtl_euler_deg(:,a) - gt_euler_deg(:,a));
        rmse_axis(a) = sqrt(mean(d.^2));
    end
    overall = norm(rmse_axis);

    fprintf('N_ITER=%d WORD_LEN=%d -> roll=%.3f pitch=%.3f yaw=%.3f overall=%.3f\n', ...
        N_ITER, WORD_LEN, rmse_axis(1), rmse_axis(2), rmse_axis(3), overall);

    row = [N_ITER, WORD_LEN, rmse_axis, overall];
    if exist('sweep_results.csv', 'file')
        old = csvread('sweep_results.csv');
        csvwrite('sweep_results.csv', [old; row]);
    else
        csvwrite('sweep_results.csv', row);
    end
end

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