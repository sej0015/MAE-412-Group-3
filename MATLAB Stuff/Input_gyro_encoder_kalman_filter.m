
Ts = .2; %sec, sampling time

dest_points = [3,3,0,0;...
               0,3,3,0]; %m
SV_post = [0;0;0]; %initial guesses
P_post = eye(3); %initial guesses
Q = [1,0,0;...
     0,1,0;...
     0,0,1];  %cov matrix
R = [1,0,0;...
     0,1,0;...
     0,0,1];  %cov matrix
mag_delP = 3;
j = 1;
%% loop should start here

while mag_delP >= .1
    %kalman filter here
    %cmd_vel: need this for the kalman filter
    %cmd_r: need this for the kalman filter
    % Recieve signals from robot

    enc_vel = %velocity from encoders
    gyro_r = %yawrate from gyro

    SV_prior = [SV_post(1) + cmd_r*Ts;... %rad
                SV_post(2) + cmd_vel*Ts*cos(SV_post(1));... %m
                SV_post(3) + cmd_vel*Ts*sin(SV_post(2))]; %m

    state_meas = [SV_post(1) + gyro_r*Ts;...
                  SV_post(2) + enc_vel*Ts*cos(SV_post(1));...
                  SV_post(3) + enc_vel*Ts*sin(SV_post(2))];

    F_k = [1 , 0 , 0;...
           -cmd_vel*Ts*sin(SV_prior(1)) , 1 , 0;...
           cmd_vel*Ts*cos(SV_prior(1)) , 0 , 1];

    H_k = [1 , 0 , 0;...
           -enc_vel*Ts*sin(SV_prior(1)) , 1 , 0;...
           enc_vel*Ts*cos(SV_prior(1)) , 0 , 1];

    P_prior = F_k*P_post*F_k' + Q;

    r_k = state_meas - SV_prior;
    S_k = H_k*P_prior*H_k' + R;
    K_k = P_prior*H_k'*inv(S_k);
    SV_post = x_prior + K_k*r_k;
    P_post = (eye(2) - K_k*H_k)*P_prior;

    % controller here
    delP = dest_points(:,j) - SV_post(1:2,1);
    mag_delP = sqrt(delP(1)^2 + delP(2)^2); %m
    delpsi = atan(delP(2)/delP(1)) - SV_post(3,1);

    cmd_r = delpsi/Ts;
    if cmd_r >= max_r
        cmd_r = max_r;
    elseif cmd_r <= -max_r
        cmd_r = -max_r;
    end
    
    if delpsi < .1745
        if mag_delP >= .1 %m drive forward if not at point
            cmd_vel = .2; %m/s
        else
            cmd_vel = 0; %m/s
            j = j + 1;
        end
    end
    
    %still need to send signals to robot
    
    
    i = i + 1;
    Time = (i-1)*Ts;
    pause(Ts)
    
end


