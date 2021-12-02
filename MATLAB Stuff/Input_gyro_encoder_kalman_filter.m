clear
clear global
close all
clc


rosshutdown
rosinit

%% Setup Subscribers and Publishers
global yaw_rate
yaw_rate_sub = rossubscriber('/smart2_02/imu/data', @imu_callback, 'DataFormat','struct');
yaw_rate = 0;

global wheel_speed wheel_yaw_rate
odom_sub = rossubscriber('/smart2_02/odom', @odom_callback, 'DataFormat', 'struct');
wheel_speed = 0;
wheel_yaw_rate = 0;

cmd_vel_pub = rospublisher('/smart2_02/cmd_vel', 'geometry_msgs/Twist');
cmd_vel_message = rosmessage(cmd_vel_pub);
cmd_vel_message.Linear.X = 0.0;
cmd_vel_message.Angular.Z = 0.0;
send(cmd_vel_pub, cmd_vel_message)

%% Initializing some variables

Ts = .05; %sec, sampling time

dest_points = [3,3,0,0;...
               0,3,3,0]; %m
SV_post = [0;0;0]; %initial guesses
P_post = eye(3); %initial guesses
Q = [1,0,0;...
     0,1,0;...
     0,0,1];  %cov matrix
R = [1,0,0;...
     0,1,0;...
     0,0,1];  %cov matrix
mag_delP = 3;
j = 1;
cmd_r = 0;
cmd_vel = 0;
max_r = 1;

%% loop should start here

tic()
counter = 0;
while toc() < 5
    counter = counter + 1;
    yaw_vector(counter) = yaw_rate;
end

yaw_bias = mean(yaw_vector);

while true
    %kalman filter here
    %cmd_vel: need this for the kalman filter
    %cmd_r: need this for the kalman filter
    % Recieve signals from robot

    enc_vel = wheel_speed;%velocity from encoders
    gyro_r = yaw_rate - yaw_bias;%yawrate from gyro

%     SV_post = [SV_post(1) + gyro_r*Ts;... %rad
%                 SV_post(2) + enc_vel*Ts*cos(SV_post(1));... %m
%                 SV_post(3) + enc_vel*Ts*sin(SV_post(1))]; %m
%     
    
    SV_prior = [SV_post(1) + cmd_r*Ts;... %rad
                SV_post(2) + cmd_vel*Ts*cos(SV_post(1));... %m
                SV_post(3) + cmd_vel*Ts*sin(SV_post(1))]; %m

    state_meas = [SV_post(1) + gyro_r*Ts;...
                  SV_post(2) + enc_vel*Ts*cos(SV_post(1));...
                  SV_post(3) + enc_vel*Ts*sin(SV_post(1))];

    F_k = [1 , 0 , 0;...
           -cmd_vel*Ts*sin(SV_prior(1)) , 1 , 0;...
           cmd_vel*Ts*cos(SV_prior(1)) , 0 , 1];

    H_k = [1 , 0 , 0;...
           -enc_vel*Ts*sin(SV_prior(1)) , 1 , 0;...
           enc_vel*Ts*cos(SV_prior(1)) , 0 , 1];

    P_prior = F_k*P_post*F_k' + Q;

    r_k = state_meas - SV_prior;
    S_k = H_k*P_prior*H_k' + R;
    K_k = P_prior*H_k'*inv(S_k);
    SV_post = SV_prior + K_k*r_k;
    P_post = (eye(3) - K_k*H_k)*P_prior;

    % controller here
    delP = dest_points(:,j) - SV_post(2:3,1);
    mag_delP = sqrt(delP(1)^2 + delP(2)^2); %m
    delpsi = atan2(delP(2),delP(1)) - SV_post(1,1);

    fprintf('delpsi: %f\n', delpsi);
    disp(delP);
    
    kp_yaw = 0.1;
    
    cmd_r = kp_yaw * delpsi/Ts;
    if cmd_r >= max_r
        cmd_r = max_r;
    elseif cmd_r <= -max_r
        cmd_r = -max_r;
    end
    
    kp_speed = 1;
    max_speed = 0.25;
    
    if delpsi < .1745
        if mag_delP >= .05 %m drive forward if not at point
            cmd_vel = kp_speed * mag_delP ; %m/s
            if cmd_vel > max_speed
                cmd_vel = max_speed;
            end
        else
            cmd_vel = 0; %m/s
            j = j + 1;
        end
    end
    
    %still need to send signals to robot
    
    cmd_vel_message.Linear.X = cmd_vel;
    cmd_vel_message.Angular.Z = cmd_r;
    send(cmd_vel_pub, cmd_vel_message)
    
    i = i + 1;
    Time = (i-1)*Ts;
    pause(Ts)
    
end

rosshutdown

%% Callback functions
function imu_callback(~, msg)
    global yaw_rate
    yaw_rate = double(msg.AngularVelocity.Z);
end

function odom_callback(~, msg)
    global wheel_speed wheel_yaw_rate
    wheel_speed = msg.Twist.Twist.Linear.X;
    wheel_yaw_rate = msg.Twist.Twist.Angular.Z;
end
