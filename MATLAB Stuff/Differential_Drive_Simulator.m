clear
clc
close all

%This script simulates a differential drive robot driving in a square using
%a proportional controller and calculates the true position, velocity, and
%acceleration, as well as attitude, and angular rates. Measurement values
%for acceleration, gyro rates, and wheel encoder velocities are also found 
%using noise models.

%The differential model uses forces on the two wheels and clamps them so
%that there isn't crazy accelerations

m = 1; %mass in kg
I = 0.05; %mass moment of inertia kg m^2 (not measured, tuned)
radius = 0.1; %m
Ts = 0.01; %Time step
Total_time = 60; %How long to run the sim
time = 0:Ts:Total_time;
x = zeros(size(time));
y = zeros(size(time));
z = zeros(size(time));
Vx = zeros(size(time));
Vy = zeros(size(time));
Vz = zeros(size(time));
Ax = zeros(size(time));
Ay = zeros(size(time));
Az = zeros(size(time));
yaw = zeros(size(time));
yaw_rate = zeros(size(time));
goal_points = [10, 0; 10, 10; 0, 10; 0, 0];
%goal_points = randi(20, [10,2])-5;
goal_counter = 1;
distance_error = 0;
heading_error = 0;
heading_error_sum = 0;

%Main Loop
for i = 1:length(time)-1
    goal_location = goal_points(goal_counter, :);
    if norm([x(i), y(i)] - goal_location) < 0.25
        goal_counter = mod(goal_counter, length(goal_points)) + 1;
        goal_location = goal_points(goal_counter, :);
    end
    [force_left, force_right, heading_error_sum] = ForceController([x(i), y(i)], goal_location, yaw(i), heading_error_sum, Ts);
    [Vx_b, Vy_b, Vz_b] = ReverseFrameConversion(Vx(i), Vy(i), Vz(i), yaw(i));
    [Ax_b, Ay_b, Az_b, alpha_yaw] = DifferentialDynamics(force_left, force_right, Vx_b, Vy_b, yaw_rate(i), m, I, radius, Ts, 2);
    [Ax(i+1), Ay(i+1), Az(i+1)] = FrameConversion(Ax_b, Ay_b, Az_b, yaw(i));
    [x(i+1), y(i+1), z(i+1), Vx(i+1), Vy(i+1), Vz(i+1), yaw(i+1), yaw_rate(i+1)] = Kinematics(x(i), y(i), z(i), Vx(i), Vy(i), Vz(i), yaw(i), yaw_rate(i), alpha_yaw, Ax(i+1), Ay(i+1), Az(i+1), Ts);
    
    
    plot(x(1:i), y(1:i), 'b');
    hold on
    plot(goal_points(:,1), goal_points(:,2), 'ok');
    hold off
    xlim([-5, 15]);
    ylim([-5, 15]);
    pause(0.001);
end

    plot(x(1:i), y(1:i), 'b');
    hold on
    plot(goal_points(:,1), goal_points(:,2), 'ok');
    hold off
    xlim([-5, 15]);
    ylim([-5, 15]);
    pause(0.001);

%Adding noise for measurement data and calculating 
%The measurements we care about are accelerometer, gyro rates, and wheel
%encoder velocity for left a right wheels

Accel_measure = [Ax; Ay; Az] + normrnd(0, 0.5, 3, length(Ax));
[Accel_measure_body(1,:), Accel_measure_body(2,:), Accel_measure_body(3, :)] = ReverseFrameConversion(Accel_measure(1, :), Accel_measure(2, :), Accel_measure(3, :), yaw);
[Accel_true_body(1,:), Accel_true_body(2,:), Accel_true_body(3, :)] = ReverseFrameConversion(Ax, Ay, Az, yaw);

figure(2)

plot(time, Accel_measure(1,:), time, Accel_measure(2,:), time, Accel_measure(3,:));
title('Acceleration Measurement in global frame');
figure(3)
plot(time, Accel_measure_body(1,:), time, Accel_measure_body(2,:), time, Accel_measure_body(3,:))
title('Acceleration Measurement in body frame');

yaw_rate_measure = yaw_rate + normrnd(0, 0.1, 1, length(yaw_rate));
figure(4)
plot(time, yaw_rate_measure(1,:))
title('Yaw rate measurement');

[Vx_b, Vy_b, Vz_b] = ReverseFrameConversion(Vx, Vy, Vz, yaw);
left_wheel_velocity = Vx_b - radius * yaw_rate;
right_wheel_velocity = Vx_b + radius * yaw_rate;
left_wheel_vel_measure = left_wheel_velocity + normrnd(0, 0.05, 1, length(left_wheel_velocity));
right_wheel_vel_measure = right_wheel_velocity + normrnd(0, 0.05, 1, length(right_wheel_velocity));
figure(5)
plot(time, left_wheel_vel_measure)
hold on
plot(time, right_wheel_vel_measure)
hold off
title('Wheel encoder velocity measurement');


%This function finds the body frame acceleration of a differential drive
%robot given the forces applied to the left and right wheels and the mass
%and moment of inertial of the vehicle
function [Ax_b, Ay_b, Az_b, alpha_yaw] = DifferentialDynamics(force_left, force_right, Vx_b, Vy_b, yaw_rate, m, I, radius, Ts, mu_d)
Fx = (force_left + force_right) - Vx_b * mu_d;
Fy = -(Vy_b*m) / Ts; %This is an attempt to stop skidding
Fz = 0;
A_b = 1/m * [Fx; Fy; Fz] - [yaw_rate * Vy_b; yaw_rate * Vx_b; 0];
Ax_b = A_b(1);
Ay_b = A_b(2);
Az_b = A_b(3);

torque_sum = force_right * radius - force_left * radius - yaw_rate * mu_d/10; %Sum of torques including a friction to slow rotation (friction is tuned)
alpha_yaw = torque_sum/I;
end

%This function transforms the body frame acceleration to inertial frame.
function [Ax, Ay, Az] = FrameConversion(Ax_b, Ay_b, Az_b, yaw)
A = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1] * [Ax_b, Ay_b, Az_b]';
Ax = A(1);
Ay = A(2);
Az = A(3);
end

function [Vx_b, Vy_b, Vz_b] = ReverseFrameConversion(Vx, Vy, Vz, yaw)
Vx_b = zeros(size(Vx));
Vy_b = zeros(size(Vx));
Vz_b = zeros(size(Vx));
for i = 1:length(Vx)
    V_b = [cos(yaw(i)), sin(yaw(i)), 0; -sin(yaw(i)), cos(yaw(i)), 0; 0, 0, 1] * [Vx(i), Vy(i), Vz(i)]';
    Vx_b(i) = V_b(1);
    Vy_b(i) = V_b(2);
    Vz_b(i) = V_b(3);
end
end

%Basic 3D kinematics
function [x, y, z, Vx, Vy, Vz, yaw, yaw_rate] = Kinematics(x, y, z, Vx, Vy, Vz, yaw, yaw_rate, alpha_yaw, Ax, Ay, Az, Ts)
Vx = Vx + Ax * Ts;
Vy = Vy + Ay * Ts;
Vz = Vz + Az * Ts;
x = x + Vx * Ts;
y = y + Vy * Ts;
z = z + Vz * Ts;

yaw_rate = yaw_rate + alpha_yaw * Ts;
yaw = yaw + yaw_rate * Ts;
end


%Proportional Derivative controller on force to steer to goal using truth
%data just to collect data
function [force_left, force_right, head_error_sum] = ForceController(current_location, goal_location, yaw, head_error_sum, Ts)

kp_dist = 2;
kp_heading = 100;
ki_heading = 0;
max_force = 1;

goal_heading = mod(atan2(goal_location(2) - current_location(2), goal_location(1) - current_location(1)), 2*pi);
heading_error = mod(yaw, 2*pi) - goal_heading;
if heading_error > pi
    heading_error = heading_error - 2*pi;
elseif heading_error < -pi
    heading_error = heading_error + 2*pi;
end

if abs(heading_error) < pi/4
    distance_error = norm(goal_location - current_location);
else
    distance_error = 0;
end

head_int = head_error_sum + heading_error * Ts;

force_left = distance_error * kp_dist/2 + heading_error * kp_heading;
force_right = distance_error * kp_dist/2 - heading_error * kp_heading + head_int * ki_heading;
if abs(force_left) > max_force
    force_left = max_force * sign(force_left);
end
if abs(force_right) > max_force
    force_right = max_force * sign(force_right);
end
end






