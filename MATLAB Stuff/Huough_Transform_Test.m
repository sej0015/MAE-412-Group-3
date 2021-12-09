%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is the main code for runing the SMART2 Robot on Matlab        %
% Author: Chris Tatsch, Conner Castle,Alejandro Mejia -              %
% based on Smart Run code by Yu Gu                                   %
%                                                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
rosshutdown % shutdown any previous node
rosinit % initialize a ros node

%Define Global Variables for Callback functions
% global imu_data;
% global mag_data;
global camera_data;


robot_name = "smart2_02"; %Name of the robot group to subscribe to




cmd_vel = rospublisher(robot_name+'/cmd_vel', 'geometry_msgs/Twist');
cmd_vel_message = rosmessage(cmd_vel);
cmd_vel_message.Linear.X = 0.0;%% Position in radians
cmd_vel_message.Angular.Z = 0.0;%%
send(cmd_vel, cmd_vel_message)

%2 Options to subscribe to data:


%Subscribe a single message

%Subscribing to Lidar
% lidar = rossubscriber(robot_name+'/scan');
% lidar_data = receive(lidar,10);
%Subscribing to Battery_Charge
battery_charge = rossubscriber(robot_name+'/battery/charge_ratio');
%battery_charge_data = receive(lidar,10);



% Subscribing to IMU using callback functions:
% imu = rossubscriber(robot_name+'/imu/data',@imu_Callback);
% % Subscribing to Magnetometer using callback functions:
% mag = rossubscriber(robot_name+'/mag/data',@mag_Callback);
% Subscribing to Camera using callback functions:
camera = rossubscriber('/usb_cam/image_raw',@camera_Callback);

while isempty(camera_data)
    pause(0.1)
end

%Current absolute time
time = rostime("now");

bigAngle=[];
while(true)
    %Plot Scan Data
    %figure(1)
    %plot(lidar_data,'MaximumRange',7)
    %figure(2)
    bw=rgb2gray(camera_data);
    BW=edge(bw, "Canny");
    [H,theta,rho] = hough(BW);
    P = houghpeaks(H,5,'threshold',ceil(0.2*max(H(:))));
    lines = houghlines(BW,theta,rho,P,'FillGap',5,'MinLength',7);
    angles=[lines.theta];
    angVals=abs(angles);
    bigAngle = [];
    for i=1:length(angVals)
        if angVals(i)>45
            bigAngle=[bigAngle,angles(i)];
        end
    end
    error = pi/2 -(mean(bigAngle)*(pi/180));
    %cmd_vel_message.Linear.X = 0.01;%% Position in radians
    %cmd_vel_message.Angular.Z = 0.2*error;%%
    disp(mean(bigAngle));
    disp(error);
    %send(cmd_vel, cmd_vel_message)
    
    imshow(BW)
    hold on
    for k = 1:length(lines)
        if lines(k).theta>0
        xy = [lines(k).point1; lines(k).point2];
        plot(xy(:,1), xy(:,2), 'LineWidth', 2, 'Color', 'green');
        end
    end
    
   %pause(0.1);
end

%Magnetometer

%Calibrate Data



%Show Images:
% while true
%     imshow(camera_data)
% end

%Drive Square




%% Callback functions:

% %IMU:
% function imu_Callback(src, message)
% global imu_data
% imu_data = message;
% end
% 
% %MAGNETOMETER:
% function mag_Callback(src, message)
% global mag_data
% mag_data = message;
% end


%CAMERA:
function camera_Callback(src, message)
global camera_data
img = readImage(message);
camera_data = flipdim((img),1); %Flip camera

end
