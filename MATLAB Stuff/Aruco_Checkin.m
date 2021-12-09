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

num_markers = 9;
global marker_measurements initial_marker_estimates marker_0_global_history
marker_sub = rossubscriber('/marker_points', @marker_callback, "DataFormat", "struct");
marker_measurements = [];
initial_marker_estimates = zeros(2, num_markers);
marker_0_global_history = [0;0];


cmd_vel_pub = rospublisher('/smart2_02/cmd_vel', 'geometry_msgs/Twist');
cmd_vel_message = rosmessage(cmd_vel_pub);
cmd_vel_message.Linear.X = 0.0;
cmd_vel_message.Angular.Z = 0.0;
send(cmd_vel_pub, cmd_vel_message)

%% Initializing some variables
global SV_post

Ts = .05; %sec, sampling time

dest_points = [3,3,0,0;...
               0,3,3,0]; %m
SV_post = [0;0;0]; %initial guesses
P_post = eye(3); %initial guesses
Q = eye(3);  %cov matrix
R = eye(2) * 20;  %cov matrix
mag_delP = 3;
j = 1;
loop_counter = 0;
cmd_r = 0;
cmd_vel = 0;
max_r = 1;

pose_history = [0; 0];

%% loop should start here

tic()
counter = 0;
while toc() < 5
    counter = counter + 1;
    yaw_vector(counter) = yaw_rate;
end

yaw_bias = mean(yaw_vector);
currentTime = toc();
total_time = 0;
while true
    Ts = toc() - currentTime;
    currentTime = toc();
    %disp(Ts);
    %k----------------------State Estimate---------------------------------
    %cmd_vel: need this for the kalman filter
    %cmd_r: need this for the kalman filter
    % Recieve signals from robot

    enc_vel = wheel_speed;%velocity from encoders
    gyro_r = yaw_rate - yaw_bias;%yawrate from gyro

    %Every loop, run the PREDICTION step using gyro rate and encoder
    %velocities as input
    SV_prior = [SV_post(1) + gyro_r*Ts;... %rad
                SV_post(2) + enc_vel*Ts*cos(SV_post(1));... %m
                SV_post(3) + enc_vel*Ts*sin(SV_post(1))]; %m
   
    F_k = [1 , 0 , 0;...
          -cmd_vel*Ts*sin(SV_prior(1)) , 1 , 0;...
          cmd_vel*Ts*cos(SV_prior(1)) , 0 , 1];
        
    P_prior = F_k*P_post*F_k' + Q;
    
    %This gets overwritten if there is a measurement
    SV_post = SV_prior;
    SV_post(1) = mod(SV_post(1, 1), 2*pi);
    P_post = P_prior;
    
%    Need to do a MEASUREMENT UPDATE for each marker measured
    for i = 1:size(marker_measurements, 2)       
        offset = [-0.08; -0.12];
        psi = SV_post(1)-pi/2; %Might need to be -pi/2
        xy_from_marker = initial_marker_estimates(:, marker_measurements(1, i)+1) - [cos(psi), -sin(psi); sin(psi), cos(psi)]*(marker_measurements(2:3, i) + offset);
        
        C = [0 1 0; 0 0 1];
        
        residual = xy_from_marker - C * SV_prior;
        S_k = C*P_prior*C'+ R;
        K_k = P_prior*C'/(S_k);
        SV_post = SV_prior + K_k * residual;
        SV_post(1) = mod(SV_post(1, 1), 2*pi);
        P_post = (eye(3) - K_k*C)*P_prior;
        
        %disp('measurement update');
        marker_4_seen = false;
    end
    marker_measurements = [];
            
             
    %----------------------Controller------------------------------------
    delP = dest_points(:,j) - SV_post(2:3);
    mag_delP = sqrt(delP(1)^2 + delP(2)^2); %m
    delpsi = atan2(delP(2),delP(1)) - SV_post(1);
    
    if delpsi > pi
        delpsi = delpsi - 2*pi;
        %disp('Too high');
    elseif delpsi < -pi
        delpsi = delpsi + 2*pi;
        %disp('Too low');
    end

    %fprintf('delpsi: %f\n', delpsi);
    %disp(delP);
    
    kp_yaw = 0.1;
    
    cmd_r = kp_yaw * delpsi/Ts;
    if cmd_r >= max_r
        cmd_r = max_r;
    elseif cmd_r <= -max_r
        cmd_r = -max_r;
    end
    
    kp_speed = 10;
    max_speed = 0.25;
    
    if abs(delpsi) < .15 %A small allowable radian error
        if mag_delP >= .1 %m drive forward if not at point
            cmd_vel = kp_speed * mag_delP ; %m/s
            if cmd_vel > max_speed
                cmd_vel = max_speed;
            end
        else
            cmd_vel = 0; %m/s
            j = j + 1;
            if j == 5
                j = 1;
                loop_counter = loop_counter + 1;
                if loop_counter >= 10
                    break
                end
            end
        end
    else
        cmd_vel = 0;
    end
    
    
    cmd_vel_message.Linear.X = cmd_vel;
    cmd_vel_message.Angular.Z = cmd_r;
    send(cmd_vel_pub, cmd_vel_message)
    fprintf('x: %f\ny: %f\n\n', SV_post(2), SV_post(3));
    
    i = i + 1;
    %Time = (i-1)*Ts;
    pause(0.05)
    
    pose_history(:, end+1) = SV_post(2:3);
    total_time = total_time + Ts;
%     if total_time > 30
%         break
%     end
end

rosshutdown
figure(1)
plot(pose_history(1, :), pose_history(2, :));
figure(2)
plot(marker_0_global_history(1, :), marker_0_global_history(2, :));
axis equal


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

function marker_callback(~, msg)
    global SV_post marker_measurements initial_marker_estimates marker_0_global_history
    marker_measurements = [];
    for i = 1:length(msg.Poses) %For each marker seen this image
        %Set a new marker measurement to be [id; xpos; ypos]
        marker_measurements(1, i) = msg.Poses(i).Orientation.W;
        marker_measurements(2, i) = msg.Poses(i).Position.X;
        marker_measurements(3, i) = msg.Poses(i).Position.Y;
        %If this is the first time seeing the marker define its initial estimate
        if norm(initial_marker_estimates(:, msg.Poses(i).Orientation.W + 1)) == 0
            offset = [-0.08; -0.12];
            psi = SV_post(1)-pi/2; %Might need to be -pi/2
            initial_marker_estimates(:, msg.Poses(i).Orientation.W + 1) = ...
                ([cos(psi), -sin(psi);
                sin(psi), cos(psi)] * (marker_measurements(2:3, i) + offset)) + SV_post(2:3);
        elseif msg.Poses(i).Orientation.W == 0
            offset = [-0.08; -0.12];
            psi = SV_post(1)-pi/2;
            marker_0_global_history(:, end+1) = ...
            ([cos(psi), -sin(psi);
            sin(psi), cos(psi)] * (marker_measurements(2:3, i) + offset)) + SV_post(2:3);
            %fprintf('x:%f \ny:%f\npsi: %f\n\n', marker_0_global_history(1, end), marker_0_global_history(2, end), SV_post(1))
        end
    end
end









