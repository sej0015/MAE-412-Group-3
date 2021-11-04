clear
clc
close all

load('basic_data');

for i = 1:length(odom_data)
    position_x(i) = odom_data(i).Pose.Pose.Position.X;
    position_y(i) = odom_data(i).Pose.Pose.Position.Y;
end

plot(position_x, position_y);