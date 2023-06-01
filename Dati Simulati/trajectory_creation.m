%In this script is created the simulated trajectory to use in Kalman filter

clc
initialization
%% Reference Generation
trajectory_rif=[]; %Matrix for position reference
velocity_rif=[]; %Matrix for velocity reference
acceleration_rif=[]; %Matrix for acceleration reference 
log_vars = [];

Ts=1/75;
te=(t_max/dt)/75; %Final time

x_start=[1 1 1]'; %Starting position
v_start=[0 0 0]'; %Starting velocity
a_start=[0 0 0]'; %Starting acceleration

%x_end=[100 45 20]'; %Final position
%v_end=[1 5 0]'; %Final velocity
x_end=[350 100 50]'; %Final position
v_end=[10 20 -5]'; %Final velocity
a_end=[0 0 0]'; %Final acceleration

A=[te^3 te^4 te^5;
   3*te^2 4*te^3 5*te^4;
   6*te 12*te^2 20*te^3];

PR=zeros(6,3); %Each column is an [a0 a1 a2 a3 a4 a5]' vector

%The first three row of P comes from boundary condition (a0,a1,a2)
PR(1,:)=[x_start(1),x_start(2),x_start(3)];
PR(2,:)=[v_start(1),v_start(2),v_start(3)];
PR(3,:)=[a_start(1)/2,a_start(2)/2,a_start(3)/2];

for i=1:3
%when i = 1 we only consider x coordinate, when i = 2 we consider y
%coordinate
x_tend = x_end(i);
v_tend = v_end(i);
a_tend = a_end(i);
M=zeros(3,1);
M(1) = x_tend - PR(1,i) - PR(2,i)*te - PR(3,i)*(te^2);
M(2) = v_tend - PR(2,i) - 2*PR(3,i)*te;
M(3) = a_tend - 2*PR(3,i);

PR(4:6,i) = inv(A)*M; %the first three row of P have already been computed 

end
i = 1;
for t= 0:Ts:te
x = x_start(1) + PR(2,1)*t + PR(3,1)*t^2 + PR(4,1)*t^3 + PR(5,1)*t^4 + PR(6,1)*t^5;
y = x_start(2) + PR(2,2)*t + PR(3,2)*t^2 + PR(4,2)*t^3 + PR(5,2)*t^4 + PR(6,2)*t^5;
z = x_start(3) + PR(2,3)*t + PR(3,3)*t^2 + PR(4,3)*t^3 + PR(5,3)*t^4 + PR(6,3)*t^5;

vx = PR(2,1) + 2*PR(3,1)*t+3*PR(4,1)*t^2 + 4*PR(5,1)*t^3 + 5*PR(6,1)*t^4;
vy = PR(2,2) + 2*PR(3,2)*t+3*PR(4,2)*t^2 + 4*PR(5,2)*t^3 + 5*PR(6,2)*t^4;
vz = PR(2,3) + 2*PR(3,3)*t+3*PR(4,3)*t^2 + 4*PR(5,3)*t^3 + 5*PR(6,3)*t^4;

ax = 2*PR(3,1) + 6*PR(4,1)*t + 12*PR(5,1)*t^2 + 20*PR(6,1)*t^3;
ay = 2*PR(3,2) + 6*PR(4,2)*t + 12*PR(5,2)*t^2 + 20*PR(6,2)*t^3;
az = 2*PR(3,3) + 6*PR(4,3)*t + 12*PR(5,3)*t^2 + 20*PR(6,3)*t^3;

p =[x y z]';
v = [vx vy vz]';
a = [ax ay az]';
trajectory_gen(:,i) = [trajectory_rif,p];
velocity_gen(:,i) = [velocity_rif,v];
acceleration_gen(:,i) = [acceleration_rif,a];
i = i+1;
end

log_vars.trajectory_gen = trajectory_gen;
log_vars.velocity_gen = velocity_gen;
log_vars.acceleration_gen = acceleration_gen;
plot3(trajectory_gen(1,:),trajectory_gen(2,:),trajectory_gen(3,:));


%Creation of timeseries for acceleration data
t_imu = 0:Ts:te;
ts = timeseries(acceleration_gen,t_imu);

%Creation of timeseries for position data
j = 1;
for i = 1:15:size(trajectory_gen,2)
pos_gps(:,j) = trajectory_gen(:,i);
j = j+1;
end
t_gps = 0:dt_gps:te;
ta = timeseries(pos_gps,t_gps);

grid on