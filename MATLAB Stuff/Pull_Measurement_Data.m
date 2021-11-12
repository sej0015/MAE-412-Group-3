clear
clc
close all

wheel_radius = 0.0325; %m

imu_data = readtable('imu_data.csv');
wheel_data = readtable('wheel_data.csv');
Accel_measure_body(1, :) = table2array(imu_data(:,'field_linear_acceleration_x'));
Accel_measure_body(2, :) = table2array(imu_data(:,'field_linear_acceleration_y'));
Accel_measure_body(3, :) = table2array(imu_data(:,'field_linear_acceleration_z'));
roll_rate_measure = table2array(imu_data(:, 'field_angular_velocity_x'));
pitch_rate_measure = table2array(imu_data(:, 'field_angular_velocity_y'));
yaw_rate_measure = table2array(imu_data(:, 'field_angular_velocity_z'));
time_imu = table2array(imu_data(:, 'x_time'));
time_imu = (time_imu - time_imu(1))/10^9;

wheel_pos_left = table2array(wheel_data(:, 'field_position0'));
wheel_pos_right = table2array(wheel_data(:, 'field_position1'));
wheel_vel_left = table2array(wheel_data(:, 'field_velocity0')); %rad/s
wheel_vel_right = table2array(wheel_data(:, 'field_velocity1')); %rad/s
left_wheel_vel_measure = wheel_vel_left * wheel_radius;
right_wheel_vel_measure = wheel_vel_right * wheel_radius;

time_wheel = table2array(wheel_data(:, 'x_time'));
time_wheel = (time_wheel - time_wheel(1))/10^9;

temp_pos_left = wheel_pos_left;
temp_pos_left(1) =[];
temp_vel_left = wheel_pos_left(1:end-1) - temp_pos_left;

figure(1)
plot(time_imu, Accel_measure_body(1, :), time_imu, Accel_measure_body(2, :), time_imu, Accel_measure_body(3, :) + 9.81);
figure(2)
plot(time_wheel, left_wheel_vel_measure, time_wheel, right_wheel_vel_measure);
figure(3)
plot(time_imu, roll_rate_measure, time_imu, pitch_rate_measure, time_imu, yaw_rate_measure);

save('Real_robot_data');


