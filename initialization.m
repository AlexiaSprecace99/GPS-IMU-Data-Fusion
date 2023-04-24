%In this script there is the initialization of varibles used for GPS/IMU
%data fusion with Kalman filter 

%Initialization
clc
clear all

syms T
%Filter State Vector: [Pn Pe Pd Vn Ve Vd An Ae Ad] where n stands for Nord,
%e for East and d for Down

%Initial Condition for the Filter State Vector
Pn_0 = 1;
Pe_0 = 1;
Pd_0 = 1;
Vn_0 = 0;
Ve_0 = 0;
Vd_0 = 0;
An_0 = 0;
Ae_0 = 0;
Ad_0 = 0;
X_start = [Pn_0 Pe_0 Pd_0 Vn_0 Ve_0 Vd_0 An_0 Ae_0 Ad_0]';

%Standard Deviation on the Initial State 
std_dev_init = [0.01 0.01 0.01 0.1 0.1 0.1 0.1 0.1 0.1];

%Initial Estimate
X_hat = X_start + (std_dev_init)*randn(size(X_start,1),1);

%Uncertainty matrix on the initial state
P = diag(std_dev_init)^2; 

%Dynamic Matrix
dt = 1/100; %Sample Time of the filter
F = [eye(3) T*eye(3) (T^2)*eye(3)/2; zeros(3) eye(3) T*eye(3); zeros(3) zeros(3) eye(3)];

%Sensor Parameter
%Sensor Available: GPS (observation of position) and IMU (observation of
%acceleration)
dt_gps = 0.2; %Sampling Time for the Gps sensor
dt_imu = 1/75; %Sampling Time for the IMU
std_dev_gps = 0.01; %Standard Deviation for the GPS
std_dev_imu = 0.05; %Standard Deviation for the IMU
R_gps = blkdiag(std_dev_gps,std_dev_gps,std_dev_gps)^2;
R_imu = blkdiag(std_dev_imu,std_dev_imu,std_dev_imu)^2;

%Covariance Matrix of the process noise
Q = eye(9);

%Measure Matrix for GPS
H_gps = [eye(3) zeros(3) zeros(3)];

%Measure matrix for IMU
H_imu = [zeros(3) zeros(3) eye(3)];

%Simulation
t_max = 50;

%Measure matrix 
H = [eye(3) zeros(3) zeros(3);zeros(3) zeros(3) eye(3)];

%Covariance matrix for sensor noise
R = blkdiag(R_gps,R_imu);


