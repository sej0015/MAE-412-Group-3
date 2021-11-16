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
global imu_data;
global mag_data;
global camera_data;


robot_name = "smart2_01" %Name of the robot group to subscribe to




% cmd_vel = rospublisher(robot_name+'/cmd_vel', 'geometry_msgs/Twist')
% cmd_vel_message = rosmessage(cmd_vel);
% cmd_vel_message.Linear.X = 0.5%% Position in radians
% cmd_vel_message.Angular.Z = 0.0%%
% send(cmd_vel, cmd_vel_message)

%2 Options to subscribe to data:


%Subscribe a single message 

%Subscribing to Lidar
lidar = rossubscriber(robot_name+'/scan');
lidar_data = receive(lidar,10);
%Subscribing to Battery_Charge
battery_charge = rossubscriber(robot_name+'/battery/charge_ratio');
battery_charge_data = receive(lidar,10);


%Plot Scan Data
figure
plot(lidar_data,'MaximumRange',7)



% Subscribing to IMU using callback functions:
imu = rossubscriber(robot_name+'/imu/data',@imu_Callback)
% Subscribing to Magnetometer using callback functions:
mag = rossubscriber(robot_name+'/mag/data',@mag_Callback)
% Subscribing to Camera using callback functions:
camera = rossubscriber(robot_name+'/usb_cam/image_raw',@camera_Callback)





%Current absolute time
time = rostime("now")


%Magnetometer

%Calibrate Data



%Show Images:
% while true
%     imshow(camera_data)
% end

%Drive Square




%% Callback functions:

%IMU:
function imu_Callback(src, message)
   global imu_data 
   imu_data = message;
end

%MAGNETOMETER:
function mag_Callback(src, message)
   global mag_data 
   mag_data = message;
end


%CAMERA:
function camera_Callback(src, message)
    global camera_data
    img = readImage(message);
    camera_data = flipdim((img),1); %Flip camera
    
end
