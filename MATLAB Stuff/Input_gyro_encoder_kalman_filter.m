
Ts = %sampling time
SV_post = %initial guesses
P_post = %initial guesses
Q = [1,0,0;...
     0,1,0;...
     0,0,1];  %cov matrix
R = [1,0,0;...
     0,1,0;...
     0,0,1];  %cov matrix

%% loop should start here

%kalman filter here
cmd_vel = %velocity command from last step
cmd_r = %yawrate commanded on last step

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

r_k = state_meas - 
S_k = H_k*P_prior*H_k' + R;
K_k = P_prior*H_k'*inv(S_k);
SV_post = x_prior + K_k*r_k;
P_post = (eye(2) - K_k*H_k)*P_prior;

%controller here




i = i + 1;



