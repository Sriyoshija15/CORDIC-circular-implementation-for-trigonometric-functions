%% mahony_golden.m -- floating-point Mahony filter, identical equations to mahony_filter.v

load('data_window.mat');   % gyro_win, accel_win, dt_win, gt_euler_win, imu_t_win

NUM_SAMPLES = 1000;

Kp = 1.9999;   % tuned via mahony_tune.m grid search (Q2.14 ceiling)
Ki = 0.05;     % tuned via mahony_tune.m grid search

%% Initialize orientation from first accel reading (roll/pitch) + ground truth (yaw)
ax0 = accel_win(1,1); ay0 = accel_win(1,2); az0 = accel_win(1,3);
roll0  = atan2(ay0, az0);
pitch0 = atan2(-ax0, sqrt(ay0^2 + az0^2));
yaw0   = gt_euler_win(1,3);   % yaw is unobservable without magnetometer -- seed from ground truth

q = euler2quat_manual(roll0, pitch0, yaw0);

fprintf('Initial orientation: roll=%.2f pitch=%.2f yaw=%.2f (deg)\n', ...
    rad2deg(roll0), rad2deg(pitch0), rad2deg(yaw0));
fprintf('Initial quaternion: q0=%.4f q1=%.4f q2=%.4f q3=%.4f\n', q(1), q(2), q(3), q(4));
fprintf('Initial quaternion (Q2.14 scaled, for RTL q0_in/q1_in/q2_in/q3_in): %.0f %.0f %.0f %.0f\n', ...
    q(1)*16384, q(2)*16384, q(3)*16384, q(4)*16384);

eInt = [0 0 0];

golden_quat = zeros(NUM_SAMPLES, 4);
golden_euler = zeros(NUM_SAMPLES, 3);

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

    golden_quat(k,:) = q;
    golden_euler(k,:) = quat2euler_manual(q);
end

writematrix_safe([imu_t_win(1:NUM_SAMPLES)-imu_t_win(1), golden_euler], 'golden_euler_output.csv');
writematrix_safe([(0:NUM_SAMPLES-1)', golden_quat*16384], 'golden_quat_output_scaled.csv');

% --- NEW: save per-sample float quaternions for RTL-vs-golden angle comparison ---
q_gold = golden_quat;   % N x 4, float, unit quaternions
save('golden_data.mat', 'q_gold');

fprintf('Golden filter run complete: %d samples.\n', NUM_SAMPLES);
fprintf('Final quaternion: q0=%.4f q1=%.4f q2=%.4f q3=%.4f\n', q(1), q(2), q(3), q(4));
fprintf('Final quaternion (Q2.14 scaled, for direct RTL comparison): q0=%.0f q1=%.0f q2=%.0f q3=%.0f\n', ...
    q(1)*16384, q(2)*16384, q(3)*16384, q(4)*16384);


%% ---- local functions at the end ----

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

function writematrix_safe(data, filename)
    if exist('writematrix', 'file')
        writematrix(data, filename);
    else
        csvwrite(filename, data);
    end
end