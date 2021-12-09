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
dh = 0.05;
X = -2:dh:2;
Y = -2:dh:2;
[X,Y] = meshgrid(X,Y);

x0 = [0;0];                                        % Initial position
xT = [0;1];                                        % Goal

Kr = 45;                                            % Coefficient of repulsion
Vr = Kr./sqrt((X - x0(1)).^2+(Y+x0(2)).^2);         % Vector field of repulsion
[fxr, fyr] = gradient(Vr,dh,dh);

Ka = 50;                                            % Coefficient of attraction
Va=0.5.*Ka.*((X-xT(1)).^2 + (Y-xT(2)).^2);          % Vector field of attraction
[fxa, fya] = gradient(Va,dh,dh);                    % Gradient field of attraction forces

rad = 0.000001;
th = (0:0.1*dh:2)* pi;
xo=rad.*cos(th);
yo=rad.*sin(th);

% cmd_vel_message.Linear.X = -0.5 %% Position in radians
% cmd_vel_message.Angular.Z = 0.0 %%

for n=1:10000
    pause(.001);
    % send(cmd_vel, cmd_vel_message)
    lidar = rossubscriber(robot_name+'/scan');
    lidar_data = receive(lidar,10);
    
    % Plot the obstacles
    for i = 1:length(lidar_data.Ranges)
        x(i) = lidar_data.Ranges(i) * cosd(i);
        y(i) = lidar_data.Ranges(i) * sind(i);
    end
    
    figure(1);
    plot(lidar_data,'MaximumRange',2);
    
    % Filter out unwanted obstacles
    for j=1:length(y)
        for i=1:length(x)
           if (x(i) == 0 && y(i) == 0)
               x(i) = [];
               y(i) = [];
               break;
           end
        end
    end
    
    Vo=zeros(size(Vr));                                 % Repulsion field
    Ko=1;                                               % Coefficient of repulsion
    for j=1:length(x)                                   % For ever obstacle
        xoi = xo - x(j);
        yoi = yo - y(j);

        xop(j, :) = -xoi;
        yop(j, :) = -yoi;
        for i=1:length(th)                              % Calculate the obstacle field
            Voi = Ko./sqrt((X-xoi(i)).^2 + (Y - yoi(i)).^2);
            Vo = Vo + Voi;
        end
    end

    [fxo,fyo]=gradient(Vo,dh,dh);
    fX= -fxr - fxa - fxo - fyo;
    fY= -fyr - fya - fyo + fxo;
    figure(2);
    quiver(X,Y,fX,fY,'k')
    title('Potential Field of Area')
    xlabel('X-Axis')
    ylabel('Y-Axis')
    axis equal;
    xlim([-2 2]);
    ylim([-2 2]);
    scatter(-yop,xop,'b');
    xlim([-2 2]);
    ylim([-2 2]);
    
end


%2 Options to subscribe to data:


%Subscribe a single message 

%Subscribing to Lidar
lidar = rossubscriber(robot_name+'/scan');
lidar_data = receive(lidar,10);
%Subscribing to Battery_Charge
battery_charge = rossubscriber(robot_name+'/battery/charge_ratio');
battery_charge_data = receive(lidar,10);


% === Plot LIDAR Scan Data ===
figure(1);
plot(lidar_data,'MaximumRange',7);

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

