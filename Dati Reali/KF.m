%In this Script there is an implementation of a Kalman Filter for GPS/IMU data fusion
%in the standard form prediction/correction without the addition of contextual aspect

%Measure
% pos = log_vars.trajectory_gen;
% vel = log_vars.velocity_gen;
% acc = log_vars.acceleration_gen;
% meas = [pos;vel;acc];
rand_pos = 0.01*randn(3,1);
rand_acc = 0.05*randn(3,1);
Tc = 0:0.02:t_max;

%Initialization of Matlab Function
f = matlabFunction(F);
k = 1;
log_EKF = [];

%Variable initialization for correction when there are different time sampling

selection_vector = [false false]';  % measurement selection at current iteration
flag = [0 0]';  % keeps track of the index of the most recent measurements already used for each sensor
actual_meas = [0 0 0 0 0 0]';   %contains measures at current time

log_EKF.x_hat(:,1) = X_hat;
for t = dt:dt:t_max
    %prediction step
    [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,k,Imu);
    log_EKF.x_hat(:,k+1) = X_hat;

    %[actual_meas, selection_vector, flag] = getActualMeas(x,y,z,vx,vy,vz,ax,ay,az,flag,selection_vector, k, dt);
                                                       
 %getActualMeas returns sensor measures at each step. If the measure has already been used for correction, it is not taken
 %If the measurement does not arrive, then it is not corrected with that sensor.
    
    [actual_meas, selection_vector, flag] = getActualMeas(ts,ta,flag, selection_vector, t);
    
    % correction step
    [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t);


%     error_x(1,k) = position(1,k)-log_EKF.x_hat(1,k);
%     error_y(1,k) = position(2,k)-log_EKF.x_hat(2,k);
%     error_z(1,k) = position(3,k)-log_EKF.x_hat(3,k);

    k = k + 1;
end
[x_estimation]=[log_EKF.x_hat(1,:)];
[y_estimation]=[log_EKF.x_hat(2,:)];
[z_estimation] = [log_EKF.x_hat(3,:)];
[ax_estimation] = [log_EKF.x_hat(7,:)];
[ay_estimation] = [log_EKF.x_hat(8,:)];
[az_estimation] = [log_EKF.x_hat(9,:)];
[vx_estimation] = [log_EKF.x_hat(4,:)];
[vy_estimation] = [log_EKF.x_hat(5,:)];
[vz_estimation] = [log_EKF.x_hat(6,:)];

cutOffFreq = 1; % Frequenza di taglio del filtro (in Hz)
samplingFreq = 50; % Frequenza di campionamento (in Hz)
[b, a] = butter(2, cutOffFreq / (samplingFreq / 2), 'low');
grid on;
figure(1);
%figure(2);
%z_estimation = filter(b, a, z_estimation);

plot(t_gps,GPS(1,:),'c');hold on;
plot(t_gps,GPS(2,:),'g'); hold on;  grid on;
plot(t_gps,GPS(3,:),'k'); hold on;
plot(Tc,x_estimation,'b'); hold on;  grid on;
plot(Tc,y_estimation,'r'); hold on;
plot(Tc,z_estimation,'m'); hold on; grid on;



%legend('gps East position','estimated East position');


%figure(3);


legend('gps North position','gps East position', 'gps Down position','estimated North position','estimated East position','estimated Down position');
xlabel('T[s]'); ylabel('Position[m]')


delta_posizione_x = diff(x_estimation);
velocita_x = [delta_posizione_x./dt delta_posizione_x(end)/dt];
velocita_x = filter(b, a, velocita_x);


delta_posizione_y = diff(y_estimation);
velocita_y = [delta_posizione_y./dt delta_posizione_y(end)/dt];
velocita_y = filter(b, a, velocita_y);


delta_posizione_z = diff(z_estimation);
velocita_z = [delta_posizione_z./dt delta_posizione_z(end)/dt];
velocita_z = filter(b, a, velocita_z);

figure(3);
plot(Tc,velocita_x,'c');hold on;
plot(Tc,vx_estimation,'b'); hold on;  grid on;
legend('gps North velocity','estimated North velocity')
xlabel('T[s]'); ylabel('Velocity[m/s]')
figure(4)
plot(Tc,velocita_y,'g'); hold on;  grid on;
plot(Tc,vy_estimation,'r'); hold on;
xlabel('T[s]'); ylabel('Velocity[m/s]')
legend('gps East velocity','estimated East velocity')
figure(5)
plot(Tc,velocita_z,'k'); hold on;
plot(Tc,vz_estimation,'m'); hold on; grid on;
legend('gps Down velocity','estimated Down velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]')

figure(6);
plot(t_imu,Imu(1,:),'c', LineWidth=4.5);hold on;
plot(Tc,ax_estimation,'b', LineWidth=1.5); hold on;  grid on;
legend('gps North acceleration','estimated North acceleration')
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]')
figure(7);
plot(t_imu,Imu(2,:),'c', LineWidth=4.5);hold on;
plot(Tc,ay_estimation,'r', LineWidth=1.5); hold on;
legend('gps East acceleration','estimated East acceleration')
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]')
figure(8);
plot(t_imu,Imu(3,:),'k', LineWidth=4.5); hold on;
plot(Tc,az_estimation,'m',LineWidth=1.5); hold on; grid on;
legend('gps Down acceleration','estimated Down acceleration')
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]')

figure(9)
plot(DATA(12290:24577,[7]+19),'b');hold on
plot(z_estimation);

figure(10)
plot(DATA(12290:24577,[1]+19),'b');hold on
plot(x_estimation);

figure(11)
plot(DATA(12290:24577,[4]+19),'b');hold on
plot(y_estimation);


%Prediction step: it been used acceleration measures from IMU
function  [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,k,Imu)
F = feval(f,dt); %F matrix depends on the sampling time
X_hat(7:9,1) = Imu(:,k);
X_hat = F*X_hat;
P = F*P*F'+Q;
end

function [actual_meas, selection_vector, flag] = getActualMeas(ts,ta,flag, selection_vector,t)
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
        actual_meas = ta.data(:,flag(1));
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
            actual_meas = [actual_meas;ts.data(:,flag(2))];    
        else
            selection_vector(2) = true;    
            actual_meas = ts.data(:,flag(2));     
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