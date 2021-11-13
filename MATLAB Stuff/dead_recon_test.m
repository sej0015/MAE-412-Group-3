clear
clc
close all

rosshutdown % shutdown any previous node
rosinit % initialize a ros node

robot_name = "smart2_02"; %Name of the robot group to subscribe to

%Initialize Publisher
cmd_vel = rospublisher(robot_name+'/cmd_vel', 'geometry_msgs/Twist');
cmd_vel_message = rosmessage(cmd_vel);


%Initialize Odometry Subscriber
odom = rossubscriber(robot_name+'/odom',"DataFormat","struct");
odom_data = receive(odom, 2);

for turns = 1:4
    cmd_vel_message.Linear.X = 0.1; % Position in radians
    cmd_vel_message.Angular.Z = 0.0; %%
    
    for n=1:1000
        pause(.01);
        send(cmd_vel, cmd_vel_message)
        %odom_data(length(odom_data) + 1) = receive(odom, 2);
    end
    
    cmd_vel_message.Linear.X = 0; % Position in radians
    cmd_vel_message.Angular.Z = pi/2; %%
    
    for n=1:84
        pause(.01);
        send(cmd_vel, cmd_vel_message)
        %odom_data(length(odom_data) + 1) = receive(odom, 2);
    end
end

% for i = 1:length(odom_data)
%     position_x(i) = odom_data(i).Pose.Pose.Position.X;
%     position_y(i) = odom_data(i).Pose.Pose.Position.Y;
% end
% 
% figure(1)
% hold on
% axis square
% plot(position_x, position_y)
% hold off


