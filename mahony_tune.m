%% mahony_tune.m -- grid search Kp/Ki against EuRoC ground truth RMSE

load('data_window.mat');   % gyro_win, accel_win, dt_win, gt_euler_win, imu_t_win

NUM_SAMPLES = 1000;

Kp_list = [1.0, 1.2, 1.4, 1.6, 1.8, 1.9, 1.9999]; 
Ki_list = [0, 0.001, 0.005, 0.01, 0.02, 0.05];

best_rmse = inf;
best_Kp = 0; best_Ki = 0;

results = zeros(length(Kp_list)*length(Ki_list), 3);   % [Kp, Ki, overall_rmse]
row = 1;

for Kp = Kp_list
    for Ki = Ki_list
        euler_out = run_mahony(gyro_win, accel_win, dt_win, gt_euler_win, NUM_SAMPLES, Kp, Ki);

        gt_deg = rad2deg(gt_euler_win(1:NUM_SAMPLES,:));
        out_deg = rad2deg(euler_out);

        rmse_axis = zeros(1,3);
        for a = 1:3
            d = wrap_angle_deg(out_deg(:,a) - gt_deg(:,a));
            rmse_axis(a) = sqrt(mean(d.^2));
        end
        overall = norm(rmse_axis);

        results(row,:) = [Kp, Ki, overall];
        row = row + 1;

        fprintf('Kp=%.4f Ki=%.5f -> roll=%.2f pitch=%.2f yaw=%.2f overall=%.2f\n', ...
            Kp, Ki, rmse_axis(1), rmse_axis(2), rmse_axis(3), overall);

        if overall < best_rmse
            best_rmse = overall;
            best_Kp = Kp; best_Ki = Ki;
        end
    end
end

fprintf('\n=== BEST: Kp=%.4f Ki=%.5f, overall RMSE=%.3f deg ===\n', best_Kp, best_Ki, best_rmse);
fprintf('Corresponding KP_Q (Q2.14, for RTL parameter) = %.0f\n', round(best_Kp*16384));
fprintf('Corresponding KI_Q (Q2.14, for RTL parameter) = %.0f\n', round(best_Ki*16384));


%% ---- local functions ----

function euler_out = run_mahony(gyro_win, accel_win, dt_win, gt_euler_win, NUM_SAMPLES, Kp, Ki)
    ax0 = accel_win(1,1); ay0 = accel_win(1,2); az0 = accel_win(1,3);
    roll0  = atan2(ay0, az0);
    pitch0 = atan2(-ax0, sqrt(ay0^2 + az0^2));
    yaw0   = gt_euler_win(1,3);
    q = euler2quat_manual(roll0, pitch0, yaw0);

    eInt = [0 0 0];
    euler_out = zeros(NUM_SAMPLES, 3);

    for k = 1:NUM_SAMPLES
        gx = gyro_win(k,1); gy = gyro_win(k,2); gz = gyro_win(k,3);
        ax = accel_win(k,1); ay = accel_win(k,2); az = accel_win(k,3);
        dt = dt_win(k);

        a_norm = norm([ax ay az]);
        if a_norm > 0
            axn = ax/a_norm; ayn = ay/a_norm; azn = az/a_norm;
        else
            axn = 0; ayn = 0; azn = 0;
        end

        q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0^2 - q1^2 - q2^2 + q3^2;

        ex = ayn*vz - azn*vy;
        ey = azn*vx - axn*vz;
        ez = axn*vy - ayn*vx;

        eInt = eInt + [ex ey ez]*Ki*dt;
        wx = gx + Kp*ex + eInt(1);
        wy = gy + Kp*ey + eInt(2);
        wz = gz + Kp*ez + eInt(3);

        qd0 = -0.5*(q1*wx + q2*wy + q3*wz);
        qd1 =  0.5*(q0*wx + q2*wz - q3*wy);
        qd2 =  0.5*(q0*wy - q1*wz + q3*wx);
        qd3 =  0.5*(q0*wz + q1*wy - q2*wx);

        q = q + [qd0 qd1 qd2 qd3]*dt;
        q = q / norm(q);

        euler_out(k,:) = quat2euler_manual(q);
    end
end

function q = euler2quat_manual(roll, pitch, yaw)
    cr = cos(roll/2);  sr = sin(roll/2);
    cp = cos(pitch/2); sp = sin(pitch/2);
    cy = cos(yaw/2);   sy = sin(yaw/2);
    qw = cr*cp*cy + sr*sp*sy;
    qx = sr*cp*cy - cr*sp*sy;
    qy = cr*sp*cy + sr*cp*sy;
    qz = cr*cp*sy - sr*sp*cy;
    q = [qw qx qy qz];
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