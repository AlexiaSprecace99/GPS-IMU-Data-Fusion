%In this Script there is an implementation of a Kalman Filter for GPS/IMU data fusion
%in the standard form prediction/correction without the addition of contextual aspect

%Measure
% pos = log_vars.trajectory_gen;
% vel = log_vars.velocity_gen;
% acc = log_vars.acceleration_gen;
% meas = [pos;vel;acc];
Tc = 0:0.02:t_max;


%Initialization of Matlab Function
f = matlabFunction(F);
k = 1;
log_KF = [];

%Variable initialization for correction when there are different time sampling

selection_vector = [false false false]';  % measurement selection at current iteration
flag = [0 0 0]';  % keeps track of the index of the most recent measurements already used for each sensor
actual_meas = [0 0 0 0 0 0 0 0 0]';   %contains measures at current time

log_KF(1).x_hat_pred = X_hat;
for t = 0:dt:t_max
    %prediction step
    [X_hat, P, F] = prediction_KF(X_hat, P, Q, dt,f,k,Imu);
    log_KF(k).x_hat_pred= X_hat;
    log_KF(k).F_matrix = F;
    log_KF(k).P_pred = P;
    %[actual_meas, selection_vector, flag] = getActualMeas(x,y,z,vx,vy,vz,ax,ay,az,flag,selection_vector, k, dt);
                                                       
 %getActualMeas returns sensor measures at each step. If the measure has already been used for correction, it is not taken
 %If the measurement does not arrive, then it is not corrected with that sensor.
    
    [actual_meas, selection_vector, flag] = getActualMeas(ts,ta,tv, flag, selection_vector, t);
    
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


%Prediction step: it been used acceleration measures from IMU
function  [X_hat, P, F] = prediction_KF(X_hat, P, Q, dt,f,k,Imu)
F = feval(f,dt); %F matrix depends on the sampling time
X_hat(7:9,1) = Imu(:,k); 
X_hat = F*X_hat;
P = F*P*F'+Q;
end

function [actual_meas, selection_vector, flag] = getActualMeas(ts,ta,tv,flag, selection_vector,t)
    count = 0;
    actual_meas = [];
    count_size_meas = 0;
    %for gps
    while(((flag(1)) < size(ta.data,3)) && (ta.time(flag(1)+1) <= t))
        count = count + 1;
        flag(1) = flag(1) + 1;
        flag(2) = flag(2) + 1;
    end
    count_size_meas = 0;
    if(count == 0)
        selection_vector(1) = false;    % there isn't available measure
        selection_vector(2) = false;
    else
        count_size_meas = count_size_meas + 1;
        selection_vector(1) = true;     % available measure
        selection_vector(2) = true;     % available measure
        actual_meas = ta.data(:,flag(1));    % measure saving in actual_meas
        actual_meas = [actual_meas;tv.data(:,flag(1))];
        if t == 5 || t == 10 || t == 15 || t == 20 || t == 25 || t == 100 || t == 110 || t == 115 
            actual_meas = actual_meas+10*rand(size(actual_meas));
        end
    end


    %for imu
    count= 0;
    while(((flag(3)) < size(ts.data,3)) && (ts.time(flag(3)+1) <= t))
        count = count + 1;
        flag(3) = flag(3) + 1;
    end
    if(count == 0)
        selection_vector(3) = false;    % there isn't available measure
    else
        if(count_size_meas > 0)
            selection_vector(3) = true;    % available measure
            actual_meas = [actual_meas;ts.data(:,flag(3))];    % % measure saving in actual_meas
        else
            selection_vector(3) = true;    % available measure
            actual_meas = ts.data(:,flag(3));   % measure saving in actual_meas 
        end
    end
end

%Correction step: after seen which measures are available 

function [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t)
    counter = 0;
    if selection_vector(1) == false  %if there aren't any information of position
        H(1:6,:) = [];
        R(1:6,:) = [];
        R(:,1:6) = [];
        counter = counter+6;
    end

    if selection_vector(3) == false  %if there aren't any information of acceleration
        H(7-counter:9-counter,:) = [];
        R(7-counter:9-counter,:) = [];
        R(:,7-counter:9-counter) = [];
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