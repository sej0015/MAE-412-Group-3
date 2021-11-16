clear global
clear
clc
close all
%% Loading Data

global imu_data yaw_rate yaw_bias
imu_data = readtable('../ros_bags/driving_with_cam3/imu.csv');
yaw_rate_for_bias_remove = table2array(imu_data(:, 'field_angular_velocity_z'));
yaw_bias = mean(yaw_rate_for_bias_remove(1:310));
yaw_rate = 0;

global odom_data wheel_speed wheel_yaw_rate
odom_data = readtable('../ros_bags/driving_with_cam3/odom.csv');
%odom_time = table2array(odom_data(:, 'x_time'));
wheel_speed = 0;
wheel_yaw_rate = 0;

global marker_0 marker_0_pose
marker_0 = readtable('../ros_bags/driving_with_cam3/marker_0.csv');
marker_0_pose(1, 1) = 0;
marker_0_pose(2, 1) = 0;
marker_0_pose(3, 1) = 0;

global marker_1 marker_1_pose
marker_1 = readtable('../ros_bags/driving_with_cam3/marker_1.csv');
marker_1_pose(1, 1) = 0;
marker_1_pose(2, 1) = 0;
marker_1_pose(3, 1) = 0;

global marker_3 marker_3_pose
marker_3 = readtable('../ros_bags/driving_with_cam3/marker_3.csv');
marker_3_pose(1, 1) = 0;
marker_3_pose(2, 1) = 0;
marker_3_pose(3, 1) = 0;

%% Processing Data

Ts = 0.01;
start_time = table2array(imu_data(1, 'x_time')); %nano sec
total_time = table2array(imu_data(end, 'x_time')); %sec

global x
x(:, 1) = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; %x pos, y pos, z pos in global frame
k = 2;

for time = start_time:Ts*10^9:total_time
    
    x(:, k) = [x(1, k-1) + wheel_speed * cos(x(3, k-1)) * Ts;
               x(2, k-1) + wheel_speed * sin(x(3, k-1)) * Ts;
               x(3, k-1) + yaw_rate * Ts;
               marker_0_pose; 
               marker_1_pose;
               marker_3_pose];
    
    spin(time, time - Ts*10^9);
    k = k + 1;
end

%% Plotting

figure(1)
plot(x(1, :), x(2, :));
axis equal
title('vehicle pose');

figure(2)
marker_pose_0_cam = [table2array(marker_0(:, 'field_transforms0_transform_translation_x')), table2array(marker_0(:, 'field_transforms0_transform_translation_y')), table2array(marker_0(:, 'field_transforms0_transform_translation_z'))];
marker_pose_1_cam = [table2array(marker_1(:, 'field_transforms0_transform_translation_x')), table2array(marker_1(:, 'field_transforms0_transform_translation_y')), table2array(marker_1(:, 'field_transforms0_transform_translation_z'))];
marker_pose_3_cam = [table2array(marker_3(:, 'field_transforms0_transform_translation_x')), table2array(marker_3(:, 'field_transforms0_transform_translation_y')), table2array(marker_3(:, 'field_transforms0_transform_translation_z'))];
plot3(marker_pose_0_cam(:, 1), marker_pose_0_cam(:, 2), marker_pose_0_cam(:, 3), '.');
hold on
plot3(marker_pose_1_cam(:, 1), marker_pose_1_cam(:, 2), marker_pose_1_cam(:, 3), '.');
plot3(marker_pose_3_cam(:, 1), marker_pose_3_cam(:, 2), marker_pose_3_cam(:, 3), '.');
hold off
axis equal
title('marker pose according to camera');

figure(3)
plot3(x(4, :), x(5, :), x(6, :), '.');
hold on
plot3(x(7, :), x(8, :), x(9, :), '.');
plot3(x(10, :), x(11, :), x(12, :), '.');
hold off
axis equal
title('marker pose after transformation');
xlabel('x'); ylabel('y'), zlabel('z');

figure(4)
plot(x(3, :));
title('yaw angle');



%% Functions
function spin(time, prev_time)
    global odom_data imu_data marker_0 marker_1 marker_3
    odom_msgs = odom_data(logical((table2array(odom_data(:, 'x_time')) < time) .* (table2array(odom_data(:, 'x_time')) > prev_time)), :);
    if ~isempty(odom_msgs)
        odom_callback(odom_msgs(end, :));
    end
    
    imu_msgs = imu_data(logical((table2array(imu_data(:, 'x_time')) < time) .* (table2array(imu_data(:, 'x_time')) > prev_time)), :);
    if ~isempty(imu_msgs)
        imu_callback(imu_msgs(end, :));
    end
    
    marker_0_msgs = marker_0(logical((table2array(marker_0(:, 'x_time')) < time) .* (table2array(marker_0(:, 'x_time')) > prev_time)), :);
    if ~isempty(marker_0_msgs)
        marker_0_callback(marker_0_msgs(end, :));
    end
    
    marker_1_msgs = marker_1(logical((table2array(marker_1(:, 'x_time')) < time) .* (table2array(marker_1(:, 'x_time')) > prev_time)), :);
    if ~isempty(marker_1_msgs)
        marker_1_callback(marker_1_msgs(end, :));
    end
    
    marker_3_msgs = marker_3(logical((table2array(marker_3(:, 'x_time')) < time) .* (table2array(marker_3(:, 'x_time')) > prev_time)), :);
    if ~isempty(marker_3_msgs)
        marker_3_callback(marker_3_msgs(end, :));
    end
end

function odom_callback(msg)
    global wheel_speed wheel_yaw_rate
    wheel_speed = table2array(msg(:, 'field_twist_twist_linear_x'));
    wheel_yaw_rate = table2array(msg(:, 'field_twist_twist_angular_z'));
end

function imu_callback(msg)
    global yaw_rate yaw_bias
    yaw_rate = table2array(msg(:, 'field_angular_velocity_z')) - yaw_bias;
end

function marker_0_callback(msg)
    global marker_0_pose x
    marker_0_pose_camera = [table2array(msg(:, 'field_transforms0_transform_translation_x'));
                            table2array(msg(:, 'field_transforms0_transform_translation_y'));
                            table2array(msg(:, 'field_transforms0_transform_translation_z'))];
    psi = x(3, end)+1*pi/2;
    %psi = 0;
    marker_0_pose = ([cos(psi), -sin(psi), 0;
                    sin(psi), cos(psi), 0;
                    0, 0, 1] * (marker_0_pose_camera + [-0.06; +0.11; 0])) + [x(1:2, end); 0];
end

function marker_1_callback(msg)
    global marker_1_pose x
    marker_1_pose_camera = [table2array(msg(:, 'field_transforms0_transform_translation_x'));
                            table2array(msg(:, 'field_transforms0_transform_translation_y'));
                            table2array(msg(:, 'field_transforms0_transform_translation_z'))];
    psi = x(3, end)+1*pi/2;
    %psi = 0;
    marker_1_pose = ([cos(psi), -sin(psi), 0;
                    sin(psi), cos(psi), 0;
                    0, 0, 1] * (marker_1_pose_camera + [-0.06; +0.11; 0])) + [x(1:2, end); 0];
end

function marker_3_callback(msg)
    global marker_3_pose x
    marker_3_pose_camera = [table2array(msg(:, 'field_transforms0_transform_translation_x'));
                            table2array(msg(:, 'field_transforms0_transform_translation_y'));
                            table2array(msg(:, 'field_transforms0_transform_translation_z'))];
    psi = x(3, end)+1*pi/2;
    %psi = 0;
    marker_3_pose = ([cos(psi), -sin(psi), 0;
                    +sin(psi), cos(psi), 0;
                    0, 0, 1] * (marker_3_pose_camera + [-0.06; +0.11; 0])) + [x(1:2, end); 0];
end