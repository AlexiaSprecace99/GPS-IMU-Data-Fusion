%In this script there is the initialization of varibles used for GPS/IMU
%data fusion with Kalman filter 

%Initialization
clc
clear all

syms T
%Filter State Vector: [Pn Pe Pd Vn Ve Vd An Ae Ad] where n stands for Nord,
%e for East and d for Down

%Initial Condition for the Filter State Vector
Pn_0 = -0.8041;
Pe_0 = -0.3792;
Pd_0 = 0.1570;
Vn_0 = 0.0993;
Ve_0 = -0.0591;
Vd_0 = 0.6949;
An_0 = -0.0654;
Ae_0 = -0.0512;
Ad_0 = 0.0543;
X_start = [Pn_0 Pe_0 Pd_0 Vn_0 Ve_0 Vd_0 An_0 Ae_0 Ad_0]';

%Standard Deviation on the Initial State 
std_dev_init = [0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1];

%Initial Estimate
X_hat = X_start + (std_dev_init)*randn(size(X_start,1),1);

%Uncertainty matrix on the initial state
P = diag(std_dev_init)^2; 

%Dynamic Matrix
dt = 1/50; %Sample Time of the filter
F = [eye(3) T*eye(3) (T^2)*eye(3)/2; zeros(3) eye(3) T*eye(3); zeros(3) zeros(3) eye(3)];

%Sensor Parameter
%Sensor Available: GPS (observation of position) and IMU (observation of
%acceleration)
dt_gps = 0.1; %Sampling Time for the Gps sensor
dt_imu = 1/50; %Sampling Time for the IMU
std_dev_gps = 0.05; %Standard Deviation for the GPS
std_dev_imu = 0.01; %Standard Deviation for the IMU
R_gps = blkdiag(std_dev_gps,std_dev_gps,std_dev_gps)^2; %Gps variance matrix 
R_imu = blkdiag(std_dev_imu,std_dev_imu,std_dev_imu)^2; %Imu variance matrix

%Covariance Matrix of the process noise
Q = [1*eye(3) zeros(3,3) zeros(3,3); zeros(3,3) 0.1*eye(3) zeros(3,3);zeros(3,3) zeros(3,3) 0.01*eye(3)];
%Q = 0.001*eye(9);
%Measure Matrix for GPS
H_gps = [eye(3) zeros(3) zeros(3)];

%Measure matrix for IMU
H_imu = [zeros(3) zeros(3) eye(3)];

%Simulation time
t_max = 250.12;

%Measure matrix 
H = [eye(3) zeros(3) zeros(3);zeros(3) zeros(3) eye(3)];

%Covariance matrix for sensor noise
R = blkdiag(R_gps,R_imu);

load('LOG00054_parsed_seg3.mat');
load('tgps.mat');

%Start simulation: 250s
%End simulation: 500s
acceleration(1,:) = DATA(12290:24577,31);
acceleration(2,:) = DATA(12290:24577,34);
acceleration(3,:) = DATA(12290:24577,37);
position_complete(1,:) = DATA(12290:24577,29);
position_complete(2,:) = DATA(12290:24577,32);
position_complete(3,:) = DATA(12290:24577,35);
velocity_complete(1,:) = DATA(12290:24577,30);
velocity_complete(2,:) = DATA(12290:24577,33);
velocity_complete(3,:) = DATA(12290:24577,36);

k = 1;
for i = 1 : size(tgps)
    if(tgps(i) >= 249.96 && tgps(i) <= 500.086)
        tgps2(k) = tgps(i);
        for j = 1:size(DATA,1)

             if tgps(i) == DATA(j,1)
                 position(1,k) = DATA(j,29);
                 position(2,k) = DATA(j,32);
                 position(3,k) = DATA(j,35);
                 velocity(1,k) = DATA(j,30);
                 velocity(2,k) = DATA(j,33);
                 velocity(3,k) = DATA(j,36);
                 k = k+1;
             end
        end
    end
end


Fs = 1/mean(diff(tgps2));

% Determina il vettore del tempo
X = position(1,:);
Y = position(2,:);
Z = position(3,:);

t = (0:length(X)-1)/Fs;
tgps2 = tgps2 - 249.96;
% Interpola i dati ad una nuova frequenza di 1000 Hz
t_new = 0:1/10:max(t);
X_interp = interp1(tgps2, X, t_new,'spline');
Y_interp = interp1(tgps2, Y, t_new,'spline');
Z_interp = interp1(tgps2, Z, t_new,'spline');

GPS_interp(1,:) = X_interp;
GPS_interp(2,:) = Y_interp;
GPS_interp(3,:) = Z_interp;


j = 1;
for i = 1:1:size(X_interp,2)
Gps(:,j) = GPS_interp(:,i);
j = j+1;
end

t_gps = 0:dt_gps:250.1;
ta = timeseries(Gps,t_gps); %timeseries per il gps

s = 1;
for i = 1 : size(DATA(:,1))
    if DATA(i,1) >= 249.96 && DATA(i,1) <= 500.086
        timu(s,1) = DATA(i,1);
        s = s+1;
    end
end

Fs_a = 1/mean(diff(timu));


t_a = (0:length(acceleration(1,:))-1)/Fs_a;
timu = timu - 249.96;
% Interpola i dati ad una nuova frequenza di 1000 Hz
t_new_a = 0:1/50:max(t_a);
AX_interp = interp1(timu, acceleration(1,:), t_new_a,'spline');
AY_interp = interp1(timu, acceleration(2,:), t_new_a,'spline');
AZ_interp = interp1(timu, acceleration(3,:), t_new_a,'spline');

A_interp(1,:) = AX_interp;
A_interp(2,:) = AY_interp;
A_interp(3,:) = AZ_interp;


j = 1;
for i = 1:1:size(A_interp,2)
Imu(:,j) = A_interp(:,i);
j = j+1;
end

t_imu = 0:0.02:250.12;
ts = timeseries(Imu,t_imu);


VX = interp1(tgps2, velocity(1,:), t_new,'spline');
VY = interp1(tgps2, velocity(2,:), t_new,'spline');
VZ = interp1(tgps2, velocity(3,:), t_new,'spline');


% t_gps = 0:0.1:(size(position,2)*0.1)-0.1;
% ta = timeseries(position,t_gps); %timeseries per il gps
% 
% t_imu = 0:0.02:(size(acceleration,2)*0.02)-0.02;
% ts = timeseries(acceleration,t_imu);
