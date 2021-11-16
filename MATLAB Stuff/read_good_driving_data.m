clear global
clear
clc
close all

global imu_data yaw_rate yaw_bias
imu_data = readtable('../ros_bags/driving_with_cam/imu.csv');
yaw_rate_for_bias_remove = table2array(imu_data(:, 'field_angular_velocity_z'));
yaw_bias = mean(yaw_rate_for_bias_remove(1:660));
yaw_rate = 0;

global odom_data wheel_speed wheel_yaw_rate
odom_data = readtable('../ros_bags/driving_with_cam/odom.csv');
%odom_time = table2array(odom_data(:, 'x_time'));
wheel_speed = 0;
wheel_yaw_rate = 0;

global marker_1 marker_1_pose
marker_1 = readtable('../ros_bags/driving_with_cam/marker_1.csv');
marker_1_pose(1, 1) = 0;
marker_1_pose(2, 1) = 0;
marker_1_pose(3, 1) = 0;

% marker_1 = readtable('../ros_bags/driving_with_cam/marker_1.csv');
% marker_1_pose(:, 1) = table2array(marker_1(:, 'field_transforms0_transform_translation_x'));
% marker_1_pose(:, 2) = table2array(marker_1(:, 'field_transforms0_transform_translation_y'));
% marker_1_pose(:, 3) = table2array(marker_1(:, 'field_transforms0_transform_translation_z'));
% 
% % marker_2 = readtable('../ros_bags/driving_with_cam/marker_2.csv');
% % marker_2_pose(:, 1) = table2array(marker_2(:, 'field_transforms0_transform_translation_x'));
% % marker_2_pose(:, 2) = table2array(marker_2(:, 'field_transforms0_transform_translation_y'));
% % marker_2_pose(:, 3) = table2array(marker_2(:, 'field_transforms0_transform_translation_z'));
% 
% marker_3 = readtable('../ros_bags/driving_with_cam/marker_3.csv');
% marker_3_pose(:, 1) = table2array(marker_3(:, 'field_transforms0_transform_translation_x'));
% marker_3_pose(:, 2) = table2array(marker_3(:, 'field_transforms0_transform_translation_y'));
% marker_3_pose(:, 3) = table2array(marker_3(:, 'field_transforms0_transform_translation_z'));
% 
% marker_4 = readtable('../ros_bags/driving_with_cam/marker_4.csv');
% marker_4_pose(:, 1) = table2array(marker_4(:, 'field_transforms0_transform_translation_x'));
% marker_4_pose(:, 2) = table2array(marker_4(:, 'field_transforms0_transform_translation_y'));
% marker_4_pose(:, 3) = table2array(marker_4(:, 'field_transforms0_transform_translation_z'));
% 
% marker_5 = readtable('../ros_bags/driving_with_cam/marker_5.csv');
% marker_5_pose(:, 1) = table2array(marker_5(:, 'field_transforms0_transform_translation_x'));
% marker_5_pose(:, 2) = table2array(marker_5(:, 'field_transforms0_transform_translation_y'));
% marker_5_pose(:, 3) = table2array(marker_5(:, 'field_transforms0_transform_translation_z'));
% 
% marker_6 = readtable('../ros_bags/driving_with_cam/marker_6.csv');
% marker_6_pose(:, 1) = table2array(marker_6(:, 'field_transforms0_transform_translation_x'));
% marker_6_pose(:, 2) = table2array(marker_6(:, 'field_transforms0_transform_translation_y'));
% marker_6_pose(:, 3) = table2array(marker_6(:, 'field_transforms0_transform_translation_z'));


% figure(1)
% plot(yaw_rate);
% figure(2)
% plot(wheel_speed);
% figure(3)
% plot(wheel_yaw_rate);
% figure(4)
% plot3(marker_3_pose(:,1), marker_3_pose(:,2), marker_3_pose(:,3), '.');
% axis equal

Ts = 0.01;
start_time = table2array(imu_data(1, 'x_time')); %nano sec
total_time = table2array(imu_data(end, 'x_time')); %sec

global x
x(:, 1) = [0, 0, 0, 0, 0, 0]; %x pos, y pos, z pos in global frame
k = 2;

for time = start_time:Ts*10^9:total_time
    
    x(:, k) = [x(1, k-1) + wheel_speed * cos(x(3, k-1)) * Ts;
               x(2, k-1) + wheel_speed * sin(x(3, k-1)) * Ts;
               x(3, k-1) + yaw_rate * Ts;
               marker_1_pose];
    
    spin(time, time - Ts*10^9);
    k = k + 1;
end

figure(1)
plot(x(1, :), x(2, :));
figure(2)
marker_pose = [table2array(marker_1(:, 'field_transforms0_transform_translation_x')), table2array(marker_1(:, 'field_transforms0_transform_translation_y')), table2array(marker_1(:, 'field_transforms0_transform_translation_z'))];
plot3(marker_pose(:, 1), marker_pose(:, 2), marker_pose(:, 3), '.');
figure(3)
plot3(x(4, :), x(5, :), x(6, :), '.');
figure(4)
plot(x(3, :));
hold on
plot(x(4, :));
plot(x(1, :));
hold off
marker_time = table2array(marker_1(:, 'x_time'));
figure(5)
plot(marker_time, marker_pose(:, 1));

function spin(time, prev_time)
    global odom_data  imu_data marker_1
    odom_msgs = odom_data(logical((table2array(odom_data(:, 'x_time')) < time) .* (table2array(odom_data(:, 'x_time')) > prev_time)), :);
    if ~isempty(odom_msgs)
        odom_callback(odom_msgs(end, :));
    end
    
    imu_msgs = imu_data(logical((table2array(imu_data(:, 'x_time')) < time) .* (table2array(imu_data(:, 'x_time')) > prev_time)), :);
    if ~isempty(imu_msgs)
        imu_callback(imu_msgs(end, :));
    end
    
    marker_1_msgs = marker_1(logical((table2array(marker_1(:, 'x_time')) < time) .* (table2array(marker_1(:, 'x_time')) > prev_time)), :);
    if ~isempty(marker_1_msgs)
        marker_1_callback(marker_1_msgs(end, :));
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















