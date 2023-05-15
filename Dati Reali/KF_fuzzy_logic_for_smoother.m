%In this Script there is an implementation of a Kalman Filter for GPS/IMU data fusion
%with the introduction of contextual aspect. In particular, in particular, after introducing 
% the confidence interval using the chi-square distribution, 
% fuzzy logic was added to try to reject unreliable measures   

%Measure
% pos = log_vars.trajectory_gen;
% vel = log_vars.velocity_gen;
% acc = log_vars.acceleration_gen;
% meas = [pos;acc];
% rand_acc = 0.05*randn(3,1);
Tc = 0:0.02:t_max;
%Initialization of Matlab Function
f = matlabFunction(F);
k = 1;
log_KF = [];
selection_vector = [false false false]';  
flag = [0 0 0]'; 
actual_meas = [0 0 0 0 0 0 0 0 0]'; 
count = 0;
n= 100;
log_KF(1).x_hat_pred = X_hat;
for t = 0:dt:t_max
    %prediction step
    [X_hat, P, F] = prediction_KF(X_hat, P, Q, dt,f,k,Imu);
    log_KF(k).x_hat_pred= X_hat;
    log_KF(k).F_matrix = F;
    log_KF(k).P_pred = P;
    
    [actual_meas, selection_vector, flag] = getActualMeas(ts,ta,tv, flag, selection_vector,t);
    % correction step
    [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k);
    
    log_KF(k).x_hat_corr= X_hat;
    log_KF(k).P_corr = P;



    k = k + 1;
end
save('KF_struct', 'log_KF');


function  [X_hat, P, F] = prediction_KF(X_hat, P, Q, dt,f,k,Imu)
F = feval(f,dt);
X_hat(7:9,1) = Imu(:,k);
X_hat = F*X_hat;
P = F*P*F'+Q;
end

function [actual_meas, selection_vector, flag] = getActualMeas(ts,ta,tv,flag, selection_vector,t)
    count = 0;
    actual_meas = [];
    count_size_meas = 0;
    %for gps
    while(((flag(1)) < size(ta.data,3)) && ta.time(flag(1)+1)-t <= eps)
        count = count + 1;
        flag(1) = flag(1) + 1;
        flag(2) = flag(2) + 1;
    end
    count_size_meas = 0;
    if(count == 0)
      selection_vector(1) = false;
      selection_vector(2) = false;
    else
        count_size_meas = count_size_meas + 1;
        selection_vector(1) = true;
        selection_vector(2) = true;
        actual_meas = ta.data(:,flag(1)); 
        actual_meas = [actual_meas;tv.data(:,flag(2))];
%         if t == 5 || t == 10 || t == 15 || t == 20 || t == 25 || t == 100 || t == 110 || t == 115 
%             actual_meas = actual_meas+10*rand(size(actual_meas));
%         end
    end


    %for imu
    count= 0;
    while(((flag(3)) < size(ts.data,3)) && ts.time(flag(3)+1)-t <= eps)
        count = count + 1;
        flag(3) = flag(3) + 1;
    end
    if(count == 0)
        selection_vector(3) = false;  
    else
        if(count_size_meas > 0)
            selection_vector(3) = true;  
            actual_meas = [actual_meas;ts.data(:,flag(3))];  
        else
            selection_vector(3) = true;    
            actual_meas = ts.data(:,flag(3));    
        end
    end
end



function [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k)
    counter = 0;
    if selection_vector(1) == false 
        H(1:6,:) = [];
        R(1:6,:) = [];
        R(:,1:6) = [];
        counter = counter+6;
    end
    if selection_vector(3) == false  
        H(7-counter:9-counter,:) = [];
        R(7-counter:9-counter,:) = [];
        R(:,7-counter:9-counter) = [];
    end

if (selection_vector(1) == true && selection_vector(3) == true) 
    S = R+H*P*H';
    S_gps = S(1:6,1:6);
    S_imu = S(7:9,7:9);
    innovation = actual_meas-H*X_hat;
    innovation_gps = innovation(1:6);
    innovation_imu = innovation(7:9);
    q_gps = innovation_gps'*inv(S_gps)*innovation_gps;
    q_imu = innovation_imu'*inv(S_imu)*innovation_imu;

    %Evaluation of membership functions of fuzzy logic

    mu_gps = get_mf_valid(q_gps);
    mu_imu = get_mf_valid(q_imu);
    mu_gps_imu = mu_gps*mu_imu;

    %Evaluation of probabilities of exclusive validity

    beta_gps = mu_gps-mu_gps_imu;
    beta_imu = mu_imu-mu_gps_imu;
    beta_gps_imu = mu_gps_imu;
    beta0= 1-mu_gps-mu_imu+mu_gps_imu;
    if(q_gps > 7.8 && q_imu < 7.8) %only Imu measure
        H(1:6,:) = []; %3x9
        R(1:6,:) = [];
        R(:,1:6) = []; %3x3
        L = P*H'*inv(S_imu); %9x6
        X_hat_imu = X_hat + L*innovation_imu; % x_imu(k|k)
        X_hat = beta0*X_hat + beta_imu*X_hat_imu;
        P_imu = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9
        P = beta0*P + beta_imu*(P_imu+(X_hat-X_hat_imu)*(X_hat-X_hat_imu)');
    end
    if (q_gps < 7.8 && q_imu > 7.8) %only Gps measure
        H(7:9,:) = []; %3x9
        R(7:9,:) = [];
        R(:,7:9) = []; %3x3
        L = P*H'*inv(S_gps); %9x6
        X_hat_gps = X_hat + L*innovation_gps; % x_gps(k|k)
        X_hat = beta0*X_hat + beta_gps*X_hat_gps;
        P_gps = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9
        P = beta0*P + beta_gps*(P_gps+(X_hat-X_hat_gps)*(X_hat-X_hat_gps)');
    end
    if(q_gps > 7.8 && q_imu > 7.8) %no measure meets the constraint
        X_hat = beta0*X_hat; %x(k|k-1)
        P = beta0*P;
    end

    if(q_gps < 7.8 && q_imu < 7.8) %both measure meets the constrains
        L = P*H'*inv(S); %9x6
        X_hat_gps_imu = X_hat + L*innovation; %x_gps_imu(k|k)
        X_hat = beta0*X_hat + beta_gps_imu*X_hat_gps_imu;
        P_gps_imu = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9   
        P = beta0*P + beta_gps_imu*(P_gps_imu+(X_hat-X_hat_gps_imu)*(X_hat-X_hat_gps_imu)');
    end

end

if (selection_vector(1) == false && selection_vector(2) == true) %just acceleration information
    S_imu = R+H*P*H';
    innovation_imu = actual_meas-H*X_hat;
    q_imu = innovation_imu'*inv(S_imu)*innovation_imu;
    
    
    mu_imu = get_mf_valid(q_imu);
    beta_imu = mu_imu;
    beta0= 1-mu_imu;
    
    if(q_imu < 7.8)
        L = P*H'*inv(S_imu); %9x6
        X_hat_imu = X_hat + L*innovation_imu; %x_imu(k|k)
        X_hat = beta0*X_hat + beta_imu*X_hat_imu;
        P_imu = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9  
        P = beta0*P + beta_imu*(P_imu+(X_hat-X_hat_imu)*(X_hat-X_hat_imu)');
    end
    if(q_imu > 7.8)
        X_hat = beta0*X_hat;
        P = beta0*P;
    end
end

if (selection_vector(1) == true && selection_vector(2) == false) %just position information
    S_gps = R+H*P*H';
    innovation_gps = actual_meas-H*X_hat;
    q_gps = innovation_gps'*inv(S_gps)*innovation_gps;
    mu_gps = get_mf_valid(q_gps);
    beta_gps = mu_gps;
    beta0= 1-mu_gps;
    
    if(q_gps < 7.8)
        L = P*H'*inv(S_gps); %9x6
        X_hat_gps = X_hat + L*innovation_gps; %x_gps(k|k)
        X_hat = beta0*X_hat + beta_gps*X_hat_gps;
        P_gps = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9
        P = beta0*P + beta_gps*(P_gps+(X_hat-X_hat_gps)*(X_hat-X_hat_gps)');
    end
    if(q_gps > 7.8)
        X_hat = beta0*X_hat;
        P = beta0*P;
    end
end

if (selection_vector(1) == false && selection_vector(2) == false ) %no measure available
        beta0 = 1;
        X_hat = beta0*X_hat;
        P = beta0*P;
end
end