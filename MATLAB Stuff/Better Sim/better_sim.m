clear 
close all
clc

% This simulator is designed to mimic the callback structure of the ros
% code. There is also sensor noise and an attempt at modeling the aruco
% marker detection accurately so that everything is easier to port over
% later

%% Initialization
pose = [0; 0; 0]; %x, y, theta position of robot (truth)
prev_pose = [0; 0; 0]; %previous pose
vel = [0; 0]; %Vx, Vy of robot (truth)

dest_points = [3,3,0,0;...
               0,3,3,0]; %m (Points of square path)
current_point = 1; %Start going to point 1

Ts = 0.1; %Time step (s)

cmd_vel = [0, 0]; %Initialize command velocity as 0, (0 linear x and 0 angular z)

x_dead = [0; 0; 0]; %Dead reconing state estimate
x = [0; 0; 0]; %State Estimate
Q = eye(3);
R = eye(2)*20;
P = eye(3) * 0.05;

max_speed = 0.25; %max linear speed m/s
max_yaw_rate = 1; %max angular rate rad/s

num_markers = 20;
markers = [rand([2, num_markers]) * 5 - 1; rand([1, num_markers]) * 2*pi];;
marker_estimates = zeros([2, num_markers, 1]); %Will be big array of all estimates
initial_marker_estimates = zeros([3, num_markers]);
sensing_radius = 1.5;
camera_counter = 21;

%% Main Loop
while true
    %Get measurements from sensors
    [encoder_speed, gyro_rate] = get_measurement(pose(:, end), prev_pose, Ts);
    camera_counter = camera_counter + 1;
    if camera_counter > 3 %Only do camera update every so many tics
        camera_counter = 0;
        marker_measurements = camera_measurement(pose(:, end), markers, sensing_radius);
        %------------------------Marker Coordinate Change------------------------
        %Try to estimate global position of markers based on state estimate
        marker_estimates(:, :, end + 1) = zeros(2, num_markers);
        for i = 1:size(marker_measurements,2)
            estimated_pose = [cos(x(3, end)), -sin(x(3, end)); sin(x(3, end)), cos(x(3, end))] * marker_measurements(2:3, i) + x(1:2, end);
            marker_estimates(:, marker_measurements(1, i), end) = estimated_pose;
            if norm(initial_marker_estimates(1:2, marker_measurements(1, i))) == 0
                initial_marker_estimates(1:2, marker_measurements(1, i)) = estimated_pose;
            end
        end
    end
    
    %------------------------State Estimate------------------------
    %Use encoder speed and yaw gyro as input to state estimate
    x_apriori = x(:, end) + [encoder_speed * cos(x(3, end)) * Ts;...
                            encoder_speed * sin(x(3, end)) * Ts;...
                            gyro_rate * Ts];
    
    F = [1, 0, encoder_speed * -sin(x_apriori(3, end)) * Ts;...
         0, 1, encoder_speed * cos(x_apriori(3, end)) * Ts;...
         0, 0, 1];
                        
    P_apriori = F * P * F' + Q;
    
    x_dead_apriori = x_dead(:, end) + [encoder_speed * cos(x_dead(3, end)) * Ts;...
                            encoder_speed * sin(x_dead(3, end)) * Ts;...
                            gyro_rate * Ts];
    
    x_dead(:, end + 1) = x_dead_apriori;
    x_dead(3, end) = mod(x_dead(3, end), 2*pi);
    
    %Need to do a measurement update for each marker that is being seen
    %(Could possibly average all the estimates for efficiency?)
    
    if ~isempty(marker_measurements)
        x(:, end + 1) =  x_apriori;
        P = P_apriori;
        for i = 1:size(marker_measurements, 2)
            C = [1 0 0; 0 1 0];
            y = initial_marker_estimates(1:2, marker_measurements(1, i)) - [cos(x(3, end)), -sin(x(3, end)); sin(x(3, end)), cos(x(3, end))] * marker_measurements(2:3, i);
            r = y - C*x(:, end);
            S = C*P*C' + R;
            K = P * C' / S;
            x(:, end) = x(:, end) + K * r;
            P = (eye(3) - K*C)*P;
        end
    else
        x(:, end + 1) = x_apriori;
        x(3, end) = mod(x(3, end), 2*pi);
        P = P_apriori;
    end
        

    %------------------------Controller------------------------
    vect_to_goal = dest_points(:, current_point) - x(1:2, end);
    dist_error = norm(vect_to_goal);
    heading_error = atan2(vect_to_goal(2), vect_to_goal(1)) - x(3, end);
    if heading_error > pi
        heading_error = heading_error - 2*pi;
    elseif heading_error < -pi
        heading_error = heading_error + 2*pi;
    end
    
    kp_dist = 1;
    kp_heading = 1;
    
    if abs(heading_error) < 0.1
        cmd_vel(1) = min(dist_error * kp_heading, max_speed);
    else
        cmd_vel(1) = 0;
    end
    
    cmd_vel(2) = min(heading_error * kp_heading, max_yaw_rate);
    
    if dist_error < 0.2
        current_point = current_point + 1;
        if current_point > 4
            current_point = 1;
        end
    end
    
    %------------------------Run the Simulation------------------------
    prev_pose = pose(:, end);
    pose(:, end + 1) = update_kinematics(pose(:, end), cmd_vel, Ts);
    plot_world(pose, x, x_dead, markers, marker_measurements, marker_estimates);
    %cmd_vel(1) = cmd_vel(1) + 0.005;
end

%% Functions
function pose = update_kinematics(pose, cmd_vel, Ts)
    %pose is x, y, psi in global frame
    %cmd_vel is speed and angular rate
    %Ts is time step in seconds
    
    pose = pose + [cmd_vel(1) * cos(pose(3)) * Ts;...
                   cmd_vel(1) * sin(pose(3)) * Ts;...
                   cmd_vel(2) * Ts];
    pose(3) = mod(pose(3), 2*pi);
end

function [encoder_speed, gyro_rate] = get_measurement(pose, prev_pose, Ts)
%Generate measurements with noise added
encoder_speed = norm(pose(1:2) - prev_pose(1:2))/Ts + normrnd(0, 0.1);
gyro_rate = (pose(3) - prev_pose(3))/Ts + normrnd(0, 0.1);
end

function marker_measurements = camera_measurement(pose, markers, sensing_radius)
marker_measurements = [];
for i = 1:size(markers, 2)
    vect_to_marker = markers(1:2, i) - pose(1:2); %Vector to marker in global coordinates (truth)
    dist_to_marker = norm(vect_to_marker); %Distance from robot to marker (truth)
    if dist_to_marker < sensing_radius %If the marker is close enough to be seen add it to the measurement array
        marker_measurements(1, end+1) = i;
        marker_measurements(2:3, end) = [cos(pose(3)), sin(pose(3)); -sin(pose(3)), cos(pose(3))]*vect_to_marker;
    end
end
end

function plot_world(pose, x, x_dead, markers, marker_measurements, marker_estimates)

    figure(1)
    %Plor true position as circle
    plot(pose(1, end), pose(2, end), 'bo', 'MarkerSize', 20);
    hold on
    %Plot true position history
    plot(pose(1, :), pose(2, :), 'b');
    %Plot heading as straight line
    plot([pose(1, end), pose(1, end) + 0.2*cos(pose(3, end))], [pose(2, end), pose(2, end) + 0.2*sin(pose(3, end))], 'b');
    %Plot viewable area as green circle
    plot(pose(1, end) + 1.5 * cos(0:0.1:2*pi), pose(2, end) + 1.5 * sin(0:0.1:2*pi), 'g--');
    %Plot believ position as circle
    plot(x(1, end), x(2, end), 'ro', 'MarkerSize', 20);
    %Plot heading as straight line
    plot([x(1, end), x(1, end) + 0.2*cos(x(3, end))], [x(2, end), x(2, end) + 0.2*sin(x(3, end))], 'r');
    %Plot belief position history
    plot(x(1, :), x(2, :), 'r');
    %Plot dead reconing belief position history
    plot(x_dead(1, :), x_dead(2, :), 'g');
    for i = 1:length(markers)
        plot(markers(1, i), markers(2, i), '*');
    end
    hold off
    xlim([-1, 4]);
    ylim([-1, 4]);
    
%     figure(2)
%     for i = 1:size(marker_measurements, 2)
%         plot(marker_measurements(2, i), marker_measurements(3, i), '*');
%         hold on
%     end
%     hold off
%     xlim([-1.5, 1.5]);
%     ylim([-1.5, 1.5]);
%     
%     figure(3)
%     flattened_estimates = reshape(marker_estimates, [2, size(marker_estimates, 2) * size(marker_estimates, 3)]);
%     plot(flattened_estimates(1, :), flattened_estimates(2, :), 'b*');
%     %     for i = 1:size(marker_estimates, 3)
% %         plot(marker_estimates(1, :, i), marker_estimates(2, :, i), 'b*');
% %         hold on
% %     end
%     xlim([-1, 4]);
%     ylim([-1, 4]);
end







