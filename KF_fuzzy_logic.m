%In this Script there is an implementation of a Kalman Filter for GPS/IMU data fusion
%Measure
pos = log_vars.trajectory_gen;
vel = log_vars.velocity_gen;
acc = log_vars.acceleration_gen;
meas = [pos;acc];
rand_acc = 0.05*randn(3,1);

%Initialization of Matlab Function
f = matlabFunction(F);
k = 1;
log_EKF = [];
selection_vector = [false false]';  % seleziona quali misure sono state usate all'iterazione corrente
flag = [0 0]';  % tiene traccia dell'indice delle misure più recenti già utilizzate per ogni sensore
actual_meas = [0 0 0 0 0 0]';  % contiene i valori delle misure utilizzate all'iterazione corrente
count = 0;
n= 100;
log_EKF.x_hat(:,1) = X_hat;
for t = dt:dt:t_max
    %prediction step
    [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,log_vars,k);
    log_EKF.x_hat(:,k+1) = X_hat;
    % restituisce ad ogni passo il vettore con le misure dei sensori
    % effettive e disponibili che non erano già state prese
                                                       
   % error_x(1,k) = trajectory_gen(1,k)-log_EKF.x_hat(1,k); 
    %error_y(1,k) = trajectory_gen(2,k)-log_EKF.x_hat(2,k);
    %error_z(1,k) = trajectory_gen(3,k)-log_EKF.x_hat(3,k);
    
    [actual_meas, selection_vector, flag] = getActualMeas(ts,ta, flag, selection_vector,t);
    % correction step
    [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k);


    error_x(1,k) = trajectory_gen(1,k)-log_EKF.x_hat(1,k);
    error_y(1,k) = trajectory_gen(2,k)-log_EKF.x_hat(2,k);
    error_z(1,k) = trajectory_gen(3,k)-log_EKF.x_hat(3,k);

    error_ax(1,k) = acceleration_gen(1,k)-log_EKF.x_hat(7,k);
    error_ay(1,k) = acceleration_gen(2,k)-log_EKF.x_hat(8,k);
    error_az(1,k) = acceleration_gen(3,k)-log_EKF.x_hat(9,k);


    k = k + 1;
end
plot3(trajectory_gen(1,:),trajectory_gen(2,:),trajectory_gen(3,:)); hold on;
plot3(log_EKF.x_hat(1,:),log_EKF.x_hat(2,:),log_EKF.x_hat(3,:));
legend

figure(2); grid on; 
plot(error_x);

figure(3); grid on; 
plot(error_y);

figure(4); grid on; 
plot(error_z);


function  [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,log_vars,k)
F = feval(f,dt);
X_hat(7:9,1) = log_vars.acceleration_gen(:,k);
X_hat = F*X_hat+[0.01*randn(9,1)];
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
      selection_vector(1) = false;    % non c'è nessuna misura disponibile
    else
        count_size_meas = count_size_meas + 1;
        selection_vector(1) = true;     % la misura è disponibile
        actual_meas = ta.data(:,flag(1)); %+ 0.01*randn(3,1);    % salvo la misura su meas[]
        
    end


    %for imu
    count= 0;
    while(((flag(2)) < size(ts.data,3)) && ts.time(flag(2)+1)-t <= eps)
        count = count + 1;
        flag(2) = flag(2) + 1;
    end
    if(count == 0)
        selection_vector(2) = false;    % non c'è nessuna misura disponibile
    else
        if(count_size_meas > 0)
            selection_vector(2) = true;     % la misura è disponibile
            actual_meas = [actual_meas;ts.data(:,flag(2))]; %+[0.01*randn(3,1);0.05*randn(3,1)];    % salvo la misura su meas[]
        else
            selection_vector(2) = true;     % la misura è disponibile
            actual_meas = ts.data(:,flag(2)) ;%+ 0.05*randn(3,1);    % salvo la misura su meas[]  
        end
    end
end



function [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k)
    counter = 0;
    t
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

if (selection_vector(1) == true && selection_vector(2) == true) %ci sono entrambe le misure 
    S = R+H*P*H';
    S_gps = S(1:3,1:3);
    S_imu = S(4:6,4:6);
    innovation = actual_meas-H*X_hat;
    innovation_gps = innovation(1:3);
    innovation_imu = innovation(4:6);
    q_gps = innovation_gps'*inv(S_gps)*innovation_gps;
    q_imu = innovation_imu'*inv(S_imu)*innovation_imu;
    if(q_gps > 0.2 && q_imu < 0.2) %posso prendere solo le misure della imu
        H(1:3,:) = []; %ora ho una H di dimensione 3x9
        R(1:3,:) = [];
        R(:,1:3) = []; % la R è ora di dimensione 3x3
        L = P*H'*inv(S_imu); %9x6
        X_hat = X_hat + L*innovation_imu; %9x1
        P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9
    end
    if (q_gps < 0.2 && q_imu > 0.2)
        H(4:6,:) = []; %ora ho una H di dimensione 3x9
        R(4:6,:) = [];
        R(:,4:6) = []; % la R è ora di dimensione 3x3
        L = P*H'*inv(S_gps); %9x6
        X_hat = X_hat + L*innovation_gps; %9x1
        P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9
    end
    if(q_gps > 0.2 && q_imu > 0.2)
        X_hat = X_hat;
        P = P;
    end

    if(q_gps < 0.2 && q_imu < 0.2)
        L = P*H'*inv(S); %9x6
        X_hat = X_hat + L*innovation; %9x1
        P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9   
    end
end

if (selection_vector(1) == false && selection_vector(2) == true ) % ho solo informazioni di accelerazione e non di posizione
    S_imu = R+H*P*H';
    innovation_imu = actual_meas-H*X_hat;
    q_imu = innovation_imu'*inv(S_imu)*innovation_imu;
    if(q_imu < 0.2)
        L = P*H'*inv(S_imu); %9x6
        X_hat = X_hat + L*innovation_imu; %9x1
        P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9  
    end
    if(q_imu > 0.2)
        X_hat = X_hat;
        P = P;
    end
end

if (selection_vector(1) == true && selection_vector(2) == false) % ho solo informazioni di posizione e non di accelerazione
    S_gps = R+H*P*H';
    innovation_gps = actual_meas-H*X_hat;
    q_gps = innovation_gps'*inv(S_gps)*innovation_gps;
    if(q_gps < 0.2)
        L = P*H'*inv(S_gps); %9x6
        X_hat = X_hat + L*innovation_gps; %9x1
        P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9  
    end
    if(q_gps > 0.2)
        X_hat = X_hat;
        P = P;
    end
end

%if(isempty(innovation) == true)  % if there aren't any measures
if (selection_vector(1) == false && selection_vector(2) == false )
        X_hat = X_hat;
        P = P;
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

