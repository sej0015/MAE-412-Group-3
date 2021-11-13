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

robot_name = "smart2_02" %Name of the robot group to subscribe to

cmd_vel = rospublisher(robot_name+'/cmd_vel', 'geometry_msgs/Twist')
cmd_vel_message = rosmessage(cmd_vel);

% ==== Potential Field  ====
% Coefficients / constants
Kr = 40;                       % Coefficient of repulsion
Ka = 25;                       % Coefficient of attraction
Ko = 5;                        % Force of object. Controls the shape of the object
dh = 0.4;

rad = 0.4;
% th = linspace(0,2,11);
th = (0:0.1*dh:2) * pi;
xo = rad.*cos(th);
yo = rad.*sin(th);

% Creating coordinate system
X = -2:dh:2;
Y = -2:dh:2;
[X,Y] = meshgrid(X,Y);

x0 = [0 ; 0]; % Initial Position
xT = [0 ; 5]; % Goal position

% Repelling forces from start position
Vr = Kr./sqrt((X - x0(1)).^2 + (Y - x0(2).^2));

[fxr, fyr] = gradient(Vr,dh,dh);

% Attraction forces from start position
Va = 0.5*Ka.*sqrt((X - xT(1)).^2 + (Y - xT(2)).^2);
[fxa, fya] = gradient(Va,dh,dh);

% cmd_vel_message.Linear.X = -0.5 %% Position in radians
% cmd_vel_message.Angular.Z = 0.0 %%

for n=1:10000
    pause(.001);
    % send(cmd_vel, cmd_vel_message)
    lidar = rossubscriber(robot_name+'/scan');
    lidar_data = receive(lidar,10);
    
    figure(1);
    plot(lidar_data,'MaximumRange',2);
    
    % Plot the obstacles
    for i = 1:length(lidar_data.Ranges)
        x(i) = lidar_data.Ranges(i) * cosd(i);
        y(i) = lidar_data.Ranges(i) * sind(i);
    end
    
    Vo = zeros(size(Vr));
    % Generate a force for each obstacle
    for i=1:length(lidar_data.Ranges)
        xo1 = xo - x(i);
        yo1 = yo - y(i);
        for j=1:length(th)
            Voi = Ko./sqrt((X - xo1(j)).^2 + (Y - yo1(j)).^2); 
        end
        Vo = Vo + Voi;
    end
    figure(2);
    surf(Vo);

    
    % figure(2);
    % surf(Voi);
    % figure(2);
    % plot(x,y, '.');
    
end
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
%camera = rossubscriber(robot_name+'/usb_cam/image_raw',@camera_Callback)





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

