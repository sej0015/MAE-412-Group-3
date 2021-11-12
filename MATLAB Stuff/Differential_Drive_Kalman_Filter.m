clear
clc
close all

%% Using measurement data
Accel_measure_body = load('dynamic_sim_data.mat', 'Accel_measure_body');
Accel_measure_body = Accel_measure_body.Accel_measure_body;
Accel_measure_body(3, :) = Accel_measure_body(3, :);
yaw_rate_measure = load('dynamic_sim_data.mat', 'yaw_rate_measure');
yaw_rate_measure = yaw_rate_measure.yaw_rate_measure;
left_wheel_vel_measure = load('dynamic_sim_data.mat', 'left_wheel_vel_measure');
left_wheel_vel_measure = left_wheel_vel_measure.left_wheel_vel_measure;
right_wheel_vel_measure = load('dynamic_sim_data.mat', 'right_wheel_vel_measure');
right_wheel_vel_measure = right_wheel_vel_measure.right_wheel_vel_measure;

%% Using truth data
% Accel_measure_body = load('dynamic_sim_data.mat', 'Accel_true_body');
% Accel_measure_body = Accel_measure_body.Accel_true_body;
% Accel_measure_body(3, :) = Accel_measure_body(3, :);
% yaw_rate_measure = load('dynamic_sim_data.mat', 'yaw_rate');
% yaw_rate_measure = yaw_rate_measure.yaw_rate;
% left_wheel_vel_measure = load('dynamic_sim_data.mat', 'left_wheel_velocity');
% left_wheel_vel_measure = left_wheel_vel_measure.left_wheel_velocity;
% right_wheel_vel_measure = load('dynamic_sim_data.mat', 'right_wheel_velocity');
% right_wheel_vel_measure = right_wheel_vel_measure.right_wheel_velocity;


time = load('dynamic_sim_data.mat', 'time');
time = time.time;
Ts =  load('dynamic_sim_data.mat', 'Ts');
Ts = Ts.Ts;
g = 9.81;
radius = 0.1; %m (needs to be same as simulator)(half distance between wheels)

%Initialize state vector and covariance matrix
x(:, 1) = [0; 0; 0; 0; 0; 0; 0; 0; 0]; %x, y, z, Vx, Vy, Vz, phi, theta, psi
P = eye(9);

%TODO: need to tune the noise values
Q = eye(9) * 1.5;
R = eye(3) * 1;



for k = 2:length(Accel_measure_body)
    
    %% Setup state estimate by breaking out variables and measurements that will be in the input
    %Earth frame velocities
    Vx = x(4, k-1);
    Vy = x(5, k-1);
    Vz = x(6, k-1);
    %Body frame acceleration measurements
    %Origionally used +z up and +y left, but model uses down and right and includes gravity
    Ax = Accel_measure_body(1, k-1);
    Ay = -Accel_measure_body(2, k-1);
    Az = -Accel_measure_body(3, k-1) + g;
    %Euler angles
    
    phi = x(7, k-1);
    theta = x(8, k-1);
    psi = x(9, k-1);
    %Rate measurements
    p = 0;
    q = 0;
    r = -yaw_rate_measure(k-1);
    
    %% State Estimate
    x_apriori = x(:,k-1) + [Vx;...
                            Vy;...
                            Vz;...
                            cos(psi)*cos(theta)*Ax + (cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi))*Ay + (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*Az;...
                            cos(theta)*sin(psi)*Ax + (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*Ay + (cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi))*Az;...
                            -sin(theta)*Ax + cos(theta)*sin(phi)*Ay + cos(phi)*cos(theta)*Az - g;...
                            p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);...
                            q*cos(phi) - r*sin(phi);...
                            q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta)] * Ts;
    
    %% Calculate the jacobian of state transition using predicted state and prior measurement
    %x, y, z, Vx, Vy, Vz, phi, theta, psi
    phi = x_apriori(7);
    theta = x_apriori(8);
    psi = x_apriori(9);
    F = [1, 0, 0, Ts, 0, 0, 0, 0, 0;...
         0, 1, 0, 0, Ts, 0, 0, 0, 0;...
         0, 0, 1, 0, 0, Ts, 0, 0, 0;...
         0, 0, 0, 1, 0, 0, Ts * ((cos(psi)*cos(phi)*sin(theta) + sin(phi)*sin(psi))*Ay + (cos(phi)*sin(psi) - sin(phi)*cos(psi)*sin(theta))*Az), Ts * (cos(psi)*-sin(theta)*Ax + (cos(psi)*sin(phi)*cos(theta))*Ay + (cos(phi)*cos(psi)*cos(theta))*Az), Ts*(-sin(psi)*cos(theta)*Ax + (-sin(psi)*sin(phi)*sin(theta) - cos(phi)*cos(psi))*Ay + (sin(phi)*cos(psi) + cos(phi)*-sin(psi)*sin(theta))*Az);...
         0, 0, 0, 0, 1, 0, Ts*((-sin(phi)*cos(psi) + cos(phi)*sin(psi)*sin(theta))*Ay + (-sin(phi)*sin(psi)*sin(theta) - cos(psi)*cos(phi))*Az), Ts*(-sin(theta)*sin(psi)*Ax + (sin(phi)*sin(psi)*cos(theta))*Ay + (cos(phi)*sin(psi)*cos(theta))*Az), Ts*(cos(theta)*cos(psi)*Ax + (cos(phi)*-sin(psi) + sin(phi)*cos(psi)*sin(theta))*Ay + (cos(phi)*cos(psi)*sin(theta) + sin(psi)*sin(phi))*Az);...
         0, 0, 0, 0, 0, 1, Ts*(cos(theta)*cos(phi)*Ay - sin(phi)*cos(theta)*Az), Ts*(-cos(theta)*Ax - sin(theta)*sin(phi)*Ay + cos(phi)*-sin(theta)*Az), 0;...
         0, 0, 0, 0, 0, 0, Ts*(q*cos(phi)*tan(theta) + r*-sin(phi)*tan(theta)), Ts*(q*sin(phi)*(sec(theta))^2 + r*cos(phi)*(sec(theta))^2), 0;...
         0, 0, 0, 0, 0, 0, Ts*(q*-sin(phi) - r*cos(phi)), 0, 0;...
         0, 0, 0, 0, 0, 0, Ts*(q*cos(phi)*sec(theta) + r*-sin(phi)*sec(theta)), Ts*(q*sin(phi)*sec(theta)*tan(theta) + r*cos(phi)*sec(theta)*tan(theta)), 0];
         
    %% Predict Error Covariance
    P_apriori = F * P * F' + Q;
    
    %% Calculate Residual
    %The only measurement for now is the wheel encoder speeds, but this
    %could be expanded to include the lidar or fiducal marker locations
    wheel_vel_left = left_wheel_vel_measure(k);
    wheel_vel_right = right_wheel_vel_measure(k);
    Speed_measure = (wheel_vel_left + wheel_vel_right)/2;
    wheel_vel_yaw_rate(k) = (wheel_vel_left - wheel_vel_right)/(2*radius);
    yaw_measure = x(9, k-1) + wheel_vel_yaw_rate(k) * Ts;
    Vx_measure = Speed_measure * cos(yaw_measure);
    Vy_measure = Speed_measure * sin(yaw_measure);
    try
        test_yaw(k) = test_yaw(k-1) + wheel_vel_yaw_rate(k) * Ts;
    catch
        test_yaw(1:k) = 0;
    end
    C = [0, 0, 0, 1, 0, 0, 0, 0, 0; 0, 0, 0, 0, 1, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 1];
    residual = [Vx_measure; Vy_measure; yaw_measure] - C * x_apriori;
    
    %% Finding reidual covariance
    S = C * P_apriori * C' + R;
    
    %% Finding Kalman Gain
    K = P_apriori * C' / S;
    
    %% Finding Posteriori state estimate and covariance
    
    x(:, k) = x_apriori + K * residual;
    P = (eye(9) - K*C) * P_apriori;
    
    
end

plot(x(1, :), x(2, :));
axis equal

























