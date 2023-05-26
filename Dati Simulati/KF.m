%In this Script there is an implementation of a Kalman Filter for GPS/IMU data fusion
%in the standard form prediction/correction without the addition of contextual aspect

%Measure
pos = log_vars.trajectory_gen;
vel = log_vars.velocity_gen;
acc = log_vars.acceleration_gen;
meas = [pos;acc];
Tc = 0:1/75:t_max;

%Initialization of Matlab Function
f = matlabFunction(F);
k = 1;
log_EKF = [];

%Variable initialization for correction when there are different time sampling

selection_vector = [false false]';  % measurement selection at current iteration
flag = [0 0]';  % keeps track of the index of the most recent measurements already used for each sensor
actual_meas = [0 0 0 0 0 0]';   %contains measures at current time
count = 0;
n = 100;
log_EKF.x_hat(:,1) = X_hat;
i = 1;
for t = 0:dt:t_max
    %prediction step
    
    rand_pos = 0.01*randn(3,1);
    rand_acc = 0.01*randn(3,1);

    [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,acceleration_gen,k,rand_acc);
    log_EKF.x_hat(:,k) = X_hat;

    %[actual_meas, selection_vector, flag] = getActualMeas(x,y,z,vx,vy,vz,ax,ay,az,flag,selection_vector, k, dt);
                                                       
 %getActualMeas returns sensor measures at each step. If the measure has already been used for correction, it is not taken
 %If the measurement does not arrive, then it is not corrected with that sensor.
    
    [actual_meas, selection_vector, flag] = getActualMeas(ts,ta, flag, selection_vector, t,rand_acc,rand_pos);
    if size(actual_meas,1) == 3
        vera_acc(1:3,k) = actual_meas;
    else
        vera_acc(1:3,k) = actual_meas(4:6);
        vera_pos(1:3,i) = actual_meas(1:3);
        i = i+1;
    end


    % correction step
    [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k);


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

pos_gps_interp_x = interp1(linspace(0, 1, size(vera_pos,2)), vera_pos(1,:), linspace(0, 1, size(trajectory_gen,2)));
pos_gps_interp_y = interp1(linspace(0, 1, size(vera_pos,2)), vera_pos(2,:), linspace(0, 1, size(trajectory_gen,2)));
pos_gps_interp_z = interp1(linspace(0, 1, size(vera_pos,2)), vera_pos(3,:), linspace(0, 1, size(trajectory_gen,2)));
pos_gps_interp = [pos_gps_interp_x;pos_gps_interp_y;pos_gps_interp_z];

grid on;

figure(1);
plot(Tc,trajectory_gen(1,:),'r');hold on;
plot(Tc,pos_gps_interp_x,'c');hold on;
% plot(Tc,trajectory_gen(2,:),'g', LineWidth=4.5); hold on;  grid on;
% plot(Tc,trajectory_gen(3,:),'k', LineWidth=4.5); hold on;
plot(Tc,log_EKF.x_hat(1,:),'b'); hold on;  grid on;
% plot(Tc,log_EKF.x_hat(2,:),'r', LineWidth=1.5); hold on;
% plot(Tc,log_EKF.x_hat(3,:),'m',LineWidth=1.5); hold on; grid on;
% legend('gps North position','gps East position', 'gps Down position','estimated North position','estimated East position','estimated Down position');
legend('vera','rumorosa','stimata');
xlabel('T[s]'); ylabel('Position[m]')
figure(2)
plot(Tc,trajectory_gen(1,:)-pos_gps_interp_x,'r'); hold on; grid on; 
plot(Tc,trajectory_gen(1,:)-log_EKF.x_hat(1,:),'b');hold on; grid on; 
 
figure(3);
plot(Tc,velocity_gen(1,:),'c', LineWidth=4.5);hold on;
plot(Tc,log_EKF.x_hat(4,:),'b', LineWidth=1.5); hold on;  grid on;
legend('gps North velocity','estimated North velocity')
xlabel('T[s]'); ylabel('Velocity[m/s]')
figure(4)
plot(Tc,velocity_gen(2,:),'g', LineWidth=4.5); hold on;  grid on;
plot(Tc,log_EKF.x_hat(5,:),'r', LineWidth=1.5); hold on;
xlabel('T[s]'); ylabel('Velocity[m/s]')
legend('gps East velocity','estimated East velocity')

figure(5)
plot(Tc,velocity_gen(3,:),'k', LineWidth=4.5); hold on;
plot(Tc,log_EKF.x_hat(6,:),'m',LineWidth=1.5); hold on; grid on;
legend('gps Down velocity','estimated Down velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]')

figure(6);
plot(Tc,acceleration_gen(1,:),'c', LineWidth=4.5);hold on;
plot(Tc,log_EKF.x_hat(7,:),'b', LineWidth=1.5); hold on;  grid on;
legend('gps North acceleration','estimated North acceleration')
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]')
figure(7);
plot(Tc,acceleration_gen(2,:),'c', LineWidth=4.5);hold on;
plot(Tc,log_EKF.x_hat(8,:),'r', LineWidth=1.5); hold on; grid on;

legend('gps East acceleration','estimated East acceleration')
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]')
figure(8);
plot(Tc,acceleration_gen(3,:),'k', LineWidth=4.5); hold on;
plot(Tc,log_EKF.x_hat(9,:),'m',LineWidth=1.5); hold on; grid on;
legend('gps Down acceleration','estimated Down acceleration')
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]')

 

grid on;
figure(9);
plot(Tc,error_x,'c'); grid on;
legend('North position error');
xlabel('T[s]'); ylabel('Error position [m]');

figure(10); grid on;
plot(Tc,error_y,'c');grid on;
legend('East position error');
xlabel('T[s]'); ylabel('Error position [m]');

figure(11); grid on;
plot(Tc,error_z,'c');grid on;
legend('Down position error');
xlabel('T[s]'); ylabel('Error position [m]');

%Prediction step: it been used acceleration measures from IMU
function  [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,acceleration_gen,k,rand_acc)
F = feval(f,dt); %F matrix depends on the sampling time
X_hat(7:9,1) = acceleration_gen(:,k) + rand_acc ; 
X_hat = F*X_hat;
P = F*P*F'+Q;
end

function [actual_meas, selection_vector, flag] = getActualMeas(ts,ta,flag, selection_vector,t,rand_acc,rand_pos)
    count = 0;
    actual_meas = [];
    count_size_meas = 0;
    %for gps
    while(((flag(1)) < size(ta.data,3)) && ta.time(flag(1)+1)-t <= eps)
        count = count + 1;
        flag(1) = flag(1) + 1;
    end
    count_size_meas = 0;
    if(count == 0)
      selection_vector(1) = false;    
    else
        count_size_meas = count_size_meas + 1;
        selection_vector(1) = true;     
        actual_meas = ta.data(:,flag(1))+ rand_pos;
%         if t == 5 || t == 10 || t == 15 || t == 20 || t == 25 || t == 100 || t == 110 || t == 115
%             actual_meas = actual_meas+30*rand(size(actual_meas));
%         end     
    end


    %for imu
    count= 0;
    while(((flag(2)) < size(ts.data,3)) && ts.time(flag(2)+1)-t <= eps)
        count = count + 1;
        flag(2) = flag(2) + 1;
    end
    if(count == 0)
        selection_vector(2) = false;    
    else
        if(count_size_meas > 0)
            selection_vector(2) = true;    
            actual_meas = [actual_meas;ts.data(:,flag(2))]+[rand_pos;rand_acc];    
        else
            selection_vector(2) = true;    
            actual_meas = ts.data(:,flag(2))+ rand_acc;     
        end
    end
end
%Correction step: after seen which measures are available 

function [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k)
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