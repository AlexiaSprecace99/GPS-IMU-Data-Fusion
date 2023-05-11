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
log_KF = [];

%Variable initialization for correction when there are different time sampling

selection_vector = [false false]';  % measurement selection at current iteration
flag = [0 0]';  % keeps track of the index of the most recent measurements already used for each sensor
actual_meas = [0 0 0 0 0 0]';   %contains measures at current time

log_KF(1).x_hat_pred = X_hat;
for t = 0:dt:t_max
    %prediction step
    [X_hat, P, F] = prediction_KF(X_hat, P, Q, dt,f,k,acceleration);
    log_KF(k).x_hat_pred= X_hat;
    log_KF(k).F_matrix = F;
    log_KF(k).P_pred = P;
    %[actual_meas, selection_vector, flag] = getActualMeas(x,y,z,vx,vy,vz,ax,ay,az,flag,selection_vector, k, dt);
                                                       
 %getActualMeas returns sensor measures at each step. If the measure has already been used for correction, it is not taken
 %If the measurement does not arrive, then it is not corrected with that sensor.
    
    [actual_meas, selection_vector, flag] = getActualMeas(ts,ta, flag, selection_vector, t);
    
    % correction step
    [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t);
    log_KF(k).x_hat_corr= X_hat;
    log_KF(k).P_corr = P;


%     error_x(1,k) = position(1,k)-log_EKF.x_hat(1,k);
%     error_y(1,k) = position(2,k)-log_EKF.x_hat(2,k);
%     error_z(1,k) = position(3,k)-log_EKF.x_hat(3,k);

    k = k + 1;
end


save('KF_struct', 'log_KF');
% [x_estimation]=[log_EKF(:,k).x_hat(1,:)];
% [y_estimation]=[log_EKF.x_hat(2,:)];
% [z_estimation] = [log_EKF.x_hat(3,:)];
% 

% figure(1);
% plot(Tc,position_complete(1,:),'r');hold on; grid on;
% plot(Tc,x_estimation,'b'); 
% legend('gps x','estim x');
% 
% figure(2);
% plot(Tc,position_complete(2,:),'g'); hold on; grid on;
% plot(Tc,y_estimation,'y');
% legend('gps y','estim y');
% 
% figure(3);
% plot(Tc,position_complete(3,:),'k'); hold on; grid on;
% plot(Tc,z_estimation,'m'); hold on;
% legend('gps z','estim z');

%Prediction step: it been used acceleration measures from IMU
function  [X_hat, P, F] = prediction_KF(X_hat, P, Q, dt,f,k,acceleration)
F = feval(f,dt); %F matrix depends on the sampling time
X_hat(7:9,1) = acceleration(:,k); 
X_hat = F*X_hat;
P = F*P*F'+Q;
end

function [actual_meas, selection_vector, flag] = getActualMeas(ts,ta,flag, selection_vector,t)
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
        actual_meas = ta.data(:,flag(1));    % measure saving in actual_meas
        if t == 5 || t == 10 || t == 15 || t == 20 || t == 25 || t == 100 || t == 110 || t == 115 
            actual_meas = actual_meas+30*rand(size(actual_meas));
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
            actual_meas = [actual_meas;ts.data(:,flag(2))];    % % measure saving in actual_meas
        else
            selection_vector(2) = true;    % available measure
            actual_meas = ts.data(:,flag(2));   % measure saving in actual_meas 
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
 %Compute innovation for imu
%  R_i = inv(R_imu(7:9,7:9));
%  Rimu_inv = blkdiag(0,0,0,0,0,0,R_i);
%  K_imu = P*H_imu'*Rimu_inv; %Matrix 9x9
%  innovation_imu = K_imu*(meas(:,k)-H_imu*X_hat);
%  P_innovation_imu = H_imu'*Rimu_inv*H_imu;

% innovation_gps = pos(:,k)-H_gps*X_hat;
% S_gps = R_gps+H_gps*P*H_gps';
% L_gps = P*H_gps'*inv(S_gps);
%X_hat = X_hat + L*innovation_gps;
%P = (eye(9)-L_gps*H_gps)*P*(eye(9)-L_gps*H_gps)'+L_gps*R_gps*L_gps';
end