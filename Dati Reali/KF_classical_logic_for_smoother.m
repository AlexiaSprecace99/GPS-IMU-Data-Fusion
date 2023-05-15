%In this Script there is an implementation of a Kalman Filter for GPS/IMU data fusion
%with the introduction of contextual aspect. In particular, a confidence interval was 
% introduced within which the measures must be found for them to be used in the correction step. 
% This confidence interval is defined by the 'Chi square' distribution.

%Measure
% pos = log_vars.trajectory_gen;
% vel = log_vars.velocity_gen;
% acc = log_vars.acceleration_gen;
% meas = [pos;acc];
%rand_acc = 0.05*randn(3,1);
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
    %log_EKF.x_hat(:,k+1) = X_hat;
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
        actual_meas = ta.data(:,flag(1)); %+ 0.01*randn(3,1);
        actual_meas = [actual_meas;tv.data(:,flag(1))];
        if t == 5 || t == 10 || t == 15 || t == 20 || t == 25 || t == 100 || t == 110 || t == 115 
            actual_meas = actual_meas+10*rand(size(actual_meas));
        end
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
            actual_meas = ts.data(:,flag(3)) ;%+ 0.05*randn(3,1);     
        end
    end
end



function [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k)
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

if (selection_vector(1) == true && selection_vector(3) == true) %there are both measures
    S = R+H*P*H';
    S_gps = S(1:6,1:6);
    S_imu = S(7:9,7:9);
    innovation = actual_meas-H*X_hat;
    innovation_gps = innovation(1:6);
    innovation_imu = innovation(7:9);
    
    %Calculation of the values(q_gps,q_imu) to see if they are within the chosen confidence interval, 
    %constructed using Chi square distribution. In this case, the limit value chosen is 7.8, 
    %found with a 3-degree-of-freedom distribution and a 95% confidence interval 
    %If the measurements are below this limit then they are used for correction 
    %since they are considered more reliable

    q_gps = innovation_gps'*inv(S_gps)*innovation_gps;
    q_imu = innovation_imu'*inv(S_imu)*innovation_imu;

    if(q_gps > 7.8 && q_imu < 7.8) %takes only Imu measures
        H(1:6,:) = []; %3x9
        R(1:6,:) = [];
        R(:,1:6) = []; %3x3
        L = P*H'*inv(S_imu); %9x6
        X_hat = X_hat + L*innovation_imu; %9x1
        P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9
    end
    if (q_gps < 7.8 && q_imu > 7.8) %takes only Gps measures
        H(7:9,:) = []; %3x9
        R(7:9,:) = [];
        R(:,7:9) = []; %3x3
        L = P*H'*inv(S_gps); %9x6
        X_hat = X_hat + L*innovation_gps; %9x1
        P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9
    end
    if(q_gps > 7.8 && q_imu > 7.8) %takes nothing
        X_hat = X_hat;
        P = P;
    end

    if(q_gps < 7.8 && q_imu < 7.8) %takes both measures
        L = P*H'*inv(S); %9x6
        X_hat = X_hat + L*innovation; %9x1
        P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9   
    end
end

if (selection_vector(1) == false && selection_vector(2) == true ) % just Imu measures
    S_imu = R+H*P*H';
    innovation_imu = actual_meas-H*X_hat;
    q_imu = innovation_imu'*inv(S_imu)*innovation_imu;
    if(q_imu < 7.8) %takes measure
        L = P*H'*inv(S_imu); %9x6
        X_hat = X_hat + L*innovation_imu; %9x1
        P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9  
    end
    if(q_imu > 7.8) %doesn't take measure
        X_hat = X_hat;
        P = P;
    end
end

if (selection_vector(1) == true && selection_vector(2) == false) %just Gps measure
    S_gps = R+H*P*H';
    innovation_gps = actual_meas-H*X_hat;
    q_gps = innovation_gps'*inv(S_gps)*innovation_gps;
    if(q_gps < 7.8) %takes measure
        L = P*H'*inv(S_gps); %9x6
        X_hat = X_hat + L*innovation_gps; %9x1
        P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9  
    end
    if(q_gps > 7.8) %doesn't takes measure
        X_hat = X_hat;
        P = P;
    end
end

%if(isempty(innovation) == true)  % if there aren't any measures
if (selection_vector(1) == false && selection_vector(2) == false )
        X_hat = X_hat;
        P = P;
end
end

