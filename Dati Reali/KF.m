%In this Script there is an implementation of a Kalman Filter for GPS/IMU data fusion
%in the standard form prediction/correction without the addition of contextual aspect

%Measure
pos = log_vars.trajectory_gen;
vel = log_vars.velocity_gen;
acc = log_vars.acceleration_gen;
meas = [pos;vel;acc];
rand_pos = 0.01*randn(3,1);
rand_acc = 0.05*randn(3,1);
Tc = 0:1/75:t_max;

%Initialization of Matlab Function
f = matlabFunction(F);
k = 1;
log_EKF = [];

%Variable initialization for correction when there are different time sampling

selection_vector = [false false]';  % measurement selection at current iteration
flag = [0 0]';  % keeps track of the index of the most recent measurements already used for each sensor
actual_meas = [0 0 0 0 0 0]';   %contains measures at current time

log_EKF.x_hat(:,1) = X_hat;
for t = 0:dt:t_max
    %prediction step
    [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,log_vars,k);
    log_EKF.x_hat(:,k) = X_hat;

    %[actual_meas, selection_vector, flag] = getActualMeas(x,y,z,vx,vy,vz,ax,ay,az,flag,selection_vector, k, dt);
                                                       
 %getActualMeas returns sensor measures at each step. If the measure has already been used for correction, it is not taken
 %If the measurement does not arrive, then it is not corrected with that sensor.
    
    [actual_meas, selection_vector, flag] = getActualMeas(ts,ta, flag, selection_vector, t,rand_pos,rand_acc);
    
    % correction step
    [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t);


    error_x(1,k) = trajectory_gen(1,k)-log_EKF.x_hat(1,k);
    error_y(1,k) = trajectory_gen(2,k)-log_EKF.x_hat(2,k);
    error_z(1,k) = trajectory_gen(3,k)-log_EKF.x_hat(3,k);
    error_vx(1,k) = velocity_gen(1,k)-log_EKF.x_hat(4,k);
    error_vy(1,k) = velocity_gen(2,k)-log_EKF.x_hat(5,k);
    error_vz(1,k) = velocity_gen(3,k)-log_EKF.x_hat(6,k);
    error_ax(1,k) = acceleration_gen(1,k)-log_EKF.x_hat(7,k);
    error_ay(1,k) = acceleration_gen(2,k)-log_EKF.x_hat(8,k);
    error_az(1,k) = acceleration_gen(3,k)-log_EKF.x_hat(9,k);

    k = k + 1;
end

grid on;
% figure(1);
% plot(Tc,trajectory_gen(1,:),'r');hold on; grid on;
% plot(Tc,log_EKF.x_hat(1,:),'b'); 
% legend('gps North position','estimated North position');
% xlabel('T[s]');
% ylabel('North position[m]');
% 
% figure(2);
% plot(Tc,trajectory_gen(2,:),'g'); hold on; grid on;
% plot(Tc,log_EKF.x_hat(2,:),'y');
% legend('gps East position','estimated East position');
% xlabel('T[s]');
% ylabel('East position[m]');
% 
% figure(3);
% plot(Tc,trajectory_gen(3,:),'k'); hold on; grid on;
% plot(Tc,log_EKF.x_hat(3,:),'m'); hold on;
% legend('gps Down position','estimated Down position');
% xlabel('T[s]');
% ylabel('Down position[m]');
% 
% figure(4);
% plot(Tc,acceleration_gen(1,:),'k'); hold on; grid on;
% plot(Tc,log_EKF.x_hat(7,:),'m'); hold on;
% legend('Imu North acceleration','estimated North acceleration');
% xlabel('T[s]');
% ylabel('North acceleration[m/s^2]');
% 
% figure(5);
% plot(Tc,acceleration_gen(2,:),'k'); hold on; grid on;
% plot(Tc,log_EKF.x_hat(8,:),'m'); hold on;
% legend('Imu East acceleration','estimated East acceleration');
% xlabel('T[s]');
% ylabel('East acceleration[m/s^2]');
% 
% figure(6);
% plot(Tc,acceleration_gen(3,:),'k'); hold on; grid on;
% plot(Tc,log_EKF.x_hat(9,:),'m'); hold on;
% legend('Imu Down acceleration','estimated Down acceleration');
% xlabel('T[s]');
% ylabel('Down acceleration[m/s^2]');
% 
% figure(7);
% plot(Tc,velocity_gen(1,:),'k'); hold on; grid on;
% plot(Tc,log_EKF.x_hat(4,:),'m'); hold on;
% legend('North velocity','estimated North velocity');
% xlabel('T[s]');
% ylabel('North velocity[m/s]');
% 
% figure(8);
% plot(Tc,velocity_gen(2,:),'k'); hold on; grid on;
% plot(Tc,log_EKF.x_hat(5,:),'m'); hold on;
% legend('East velocity','estimated East velocity');
% xlabel('T[s]');
% ylabel('East velocity[m/s]');
% 
% figure(9);
% plot(Tc,velocity_gen(3,:),'k'); hold on; grid on;
% plot(Tc,log_EKF.x_hat(6,:),'m'); hold on;
% legend('Down velocity','estimated Down velocity');
% xlabel('T[s]');
% ylabel('Down velocity[m/s]');
grid on;
figure(1);
plot3(trajectory_gen(1,:),trajectory_gen(2,:),trajectory_gen(3,:),'r');
hold on; 
plot3(log_EKF.x_hat(1,:),log_EKF.x_hat(2,:),log_EKF.x_hat(3,:),'b');
xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');

grid on;
figure(2); 
plot(Tc,error_x);
legend('North position error');
xlabel('T[s]');
ylabel('North position error[m]');

grid on;
figure(3); 
plot(Tc,error_y);
legend('East position error');
xlabel('T[s]');
ylabel('East position error[m]');

grid on;
figure(4); grid on;
plot(Tc,error_z); grid on;
legend('Down position error');
xlabel('T[s]');
ylabel('Down position error[m]');

grid on;
figure(5); grid on;
plot(Tc,error_vx); grid on;
legend('North velocity error');
xlabel('T[s]');
ylabel('North velocity error[m/s]');

grid on;
figure(6); grid on;
plot(Tc,error_vy); grid on;
legend('East velocity error');
xlabel('T[s]');
ylabel('East velocity error[m/s]');

grid on;
figure(7); grid on;
plot(Tc,error_vz); grid on;
legend('Down velocity error');
xlabel('T[s]');
ylabel('Down velocity error[m/s]');

grid on;
figure(8); grid on;
plot(Tc,error_ax); grid on;
legend('North acceleration error');
xlabel('T[s]');
ylabel('North acceleration error[m/s^2]');

grid on;
figure(9); grid on;
plot(Tc,error_ay); grid on;
legend('East acceleration error');
xlabel('T[s]');
ylabel('East acceleration error[m/s^2]');

grid on;
figure(10); grid on;
plot(Tc,error_az); grid on;
legend('Down acceleration error');
xlabel('T[s]');
ylabel('Down acceleration error[m/s^2]');

%Prediction step: it been used acceleration measures from IMU
function  [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,log_vars,k)
F = feval(f,dt); %F matrix depends on the sampling time
X_hat(7:9,1) = log_vars.acceleration_gen(:,k); 
X_hat = F*X_hat;
P = F*P*F'+Q;
end

function [actual_meas, selection_vector, flag] = getActualMeas(ts,ta,flag, selection_vector,t,rand_pos,rand_acc)
    count = 0;
    actual_meas = [];
    count_size_meas = 0;
    %for gps
    while(((flag(1)) < size(ta.data,3)) && (ta.time(flag(1)+1) <= t))
        count = count + 1;
        flag(1) = flag(1) + 1;
    end
    count_size_meas = 0;
    if(count == 0)
        selection_vector(1) = false;    % there isn't available measure
    else
        count_size_meas = count_size_meas + 1;
        selection_vector(1) = true;     % available measure
        actual_meas = ta.data(:,flag(1))+0.1*rand(3,1);    % measure saving in actual_meas
        if t == 5 || t == 10 || t == 15 || t == 20 || t == 25 || t == 100 || t == 110 || t == 115
             actual_meas = actual_meas+1*rand(size(actual_meas));
        end
    end


    %for imu
    count= 0;
    while(((flag(2)) < size(ts.data,3)) && (ts.time(flag(2)+1) <= t))
        count = count + 1;
        flag(2) = flag(2) + 1;
    end
    if(count == 0)
        selection_vector(2) = false;    % there isn't available measure
    else
        if(count_size_meas > 0)
            selection_vector(2) = true;    % available measure
            actual_meas = [actual_meas;ts.data(:,flag(2))]+[0.1*rand(3,1);0.05*rand(3,1)];    % % measure saving in actual_meas
        else
            selection_vector(2) = true;    % available measure
            actual_meas = ts.data(:,flag(2))+0.05*rand(3,1);   % measure saving in actual_meas 
        end
    end
end

%Correction step: after seen which measures are available 

function [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t)
    counter = 0;
    if selection_vector(1) == false  %if there aren't any information of position
        H(1:3,:) = [];
        R(1:3,:) = [];
        R(:,1:3) = [];
        counter = counter+3;
    end

    if selection_vector(2) == false  %if there aren't any information of acceleration
        H(4-counter:6-counter,:) = [];
        R(4-counter:6-counter,:) = [];
        R(:,4-counter:6-counter) = [];
    end

    
    
innovation = actual_meas-H*X_hat;
if(isempty(innovation) == true)  % if there aren't any measures
        X_hat = X_hat;
        P = P;
else
S = R+H*P*H'; %6x6
L = P*H'*inv(S); %9x6
X_hat = X_hat + L*innovation; %9x1
P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9
end
end