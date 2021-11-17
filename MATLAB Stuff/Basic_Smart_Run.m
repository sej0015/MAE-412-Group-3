clear
clear global
close all
clc

rosshutdown
rosinit

%% Setup Subscribers and Publishers
global yaw_rate
rossubscriber("/smart2_02/imu/data", @imu_callback, "DataFormat","struct");

global wheel_speed wheel_yaw_rate
rossubscriber("/smart2_02/odom", @odom_callback, "DataFormat", "struct");

global marker_0_measure marker_1_measure marker_3_measure marker_0_pose marker_1_pose marker_3_pose
rossubscriber('/marker_points', @marker_callback, "DataFormat", "struct");
marker_0_pose(1, 1) = 0;
marker_0_pose(2, 1) = 0;
marker_0_pose(3, 1) = 0;
marker_1_pose(1, 1) = 0;
marker_1_pose(2, 1) = 0;
marker_1_pose(3, 1) = 0;
marker_3_pose(1, 1) = 0;
marker_3_pose(2, 1) = 0;
marker_3_pose(3, 1) = 0;

cmd_vel = rospublisher(robot_name+'/cmd_vel', 'geometry_msgs/Twist');
cmd_vel_message = rosmessage(cmd_vel);
cmd_vel_message.Linear.X = 0.0;
cmd_vel_message.Angular.Z = 0.0;
send(cmd_vel, cmd_vel_message)

%% Main section

Ts = 0.01;
start_time = 0; %nano sec
total_time = 60; %sec

global x
x(:, 1) = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; %x pos, y pos, z pos in global frame
k = 2;

for time = start_time:Ts:total_time
    
    x(:, k) = [x(1, k-1) + wheel_speed * cos(x(3, k-1)) * Ts;
               x(2, k-1) + wheel_speed * sin(x(3, k-1)) * Ts;
               x(3, k-1) + yaw_rate * Ts;
               marker_0_pose(:, end); 
               marker_1_pose(:, end);
               marker_3_pose(:, end)];
    
    k = k + 1;
end

%% Plotting

figure(1)
plot(x(1, :), x(2, :));
axis equal
title('vehicle pose');

figure(2)
plot3(marker_0_measure(:, 1), marker_0_measure(:, 2), marker_0_measure(:, 3), '.');
hold on
plot3(marker_1_measure(:, 1), marker_1_measure(:, 2), marker_1_measure(:, 3), '.');
plot3(marker_3_measure(:, 1), marker_3_measure(:, 2), marker_3_measure(:, 3), '.');
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

%% Callback functions
function imu_callback(~, msg)
    global yaw_rate
    yaw_rate = msg.angular_velocity.z;
end

function odom_callback(~, msg)
    global wheel_speed wheel_yaw_rate
    wheel_speed = msg.twist.twist.linear.x;
    wheel_yaw_rate = msg.twist.twist.angular.z;
end

function marker_callback(~, msg)
    global x marker_0_measure marker_1_measure marker_3_measure marker_0_pose marker_1_pose marker_3_pose
    for i = 1:length(msg.poses)
        if msg.poses(i).orientation.w == 0
            marker_0_measure(1, end + 1) = msg.poses(i).position.x;
            marker_0_measure(2, end + 1) = msg.poses(i).position.y;
            marker_0_measure(3, end + 1) = msg.poses(i).position.z;
            psi = x(3, end)+1*pi/2;
            marker_0_pose(:, end+1) = ([cos(psi), -sin(psi), 0;
                            sin(psi), cos(psi), 0;
                            0, 0, 1] * (marker_0_measure(:, end) + [-0.06; +0.11; 0])) + [x(1:2, end); 0];
        end
        if msg.poses(i).orientation.w == 1
            marker_1_measure(1, end + 1) = msg.poses(i).position.x;
            marker_1_measure(2, end + 1) = msg.poses(i).position.y;
            marker_1_measure(3, end + 1) = msg.poses(i).position.z;
            psi = x(3, end)+1*pi/2;
            marker_1_pose(:, end+1) = ([cos(psi), -sin(psi), 0;
                            sin(psi), cos(psi), 0;
                            0, 0, 1] * (marker_1_measure(:, end) + [-0.06; +0.11; 0])) + [x(1:2, end); 0];
        end
        if msg.poses(i).orientation.w == 3
            marker_3_measure(1, end + 1) = msg.poses(i).position.x;
            marker_3_measure(2, end + 1) = msg.poses(i).position.y;
            marker_3_measure(3, end + 1) = msg.poses(i).position.z;
            psi = x(3, end)+1*pi/2;
            marker_3_pose(:, end+1) = ([cos(psi), -sin(psi), 0;
                            sin(psi), cos(psi), 0;
                            0, 0, 1] * (marker_3_measure(:, end) + [-0.06; +0.11; 0])) + [x(1:2, end); 0];
        end
    end
end








