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
log_EKF = [];
selection_vector = [false false]';  
flag = [0 0]'; 
actual_meas = [0 0 0 0 0 0]'; 
count = 0;
n= 100;
log_EKF.x_hat(:,1) = X_hat;
for t = 0:dt:t_max
    %prediction step
    [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,k,Imu);
    log_EKF.x_hat(:,k) = X_hat;
   
    
    [actual_meas, selection_vector, flag] = getActualMeas(ts,ta, flag, selection_vector,t);
    % correction step
    [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k);

%     graf_x(k) = position(1,k);
%     graf_y(k) = position(2,k);
%     graf_z(k) = position(3,k);
% 
%     graf_ax(k) = acceleration(1,k);
%     graf_ay(k) = acceleration(2,k);
%     graf_az(k) = acceleration(3,k);
% 
%     error_x(1,k) = position(1,k)-log_EKF.x_hat(1,k);
%     error_y(1,k) = position(2,k)-log_EKF.x_hat(2,k);
%     error_z(1,k) = position(3,k)-log_EKF.x_hat(3,k);
% 
%     error_ax(1,k) = acceleration(1,k)-log_EKF.x_hat(7,k);
%     error_ay(1,k) = acceleration(2,k)-log_EKF.x_hat(8,k);
%     error_az(1,k) = acceleration(3,k)-log_EKF.x_hat(9,k);


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


grid on;
figure(1);
plot(t_gps,Gps(1,:),'r');hold on; grid on;
plot(Tc,x_estimation,'b'); 
legend('gps North position','estimated North position');

figure(2);
plot(t_gps,Gps(2,:),'g'); hold on; grid on;
plot(Tc,y_estimation,'y');
legend('gps East position','estimated East position');

figure(3);
plot(t_gps,Gps(3,:),'k'); hold on; grid on;
plot(Tc,z_estimation,'m'); hold on;
legend('gps Down position','estimated Down position');

figure(4);
plot(t_imu,Imu(1,:),'k'); hold on; grid on;
plot(Tc,ax_estimation,'m'); hold on;
legend('Imu North acceleration','estimated North acceleration');

figure(5);
plot(t_imu,Imu(2,:),'k'); hold on; grid on;
plot(Tc,ay_estimation,'m'); hold on;
legend('Imu East acceleration','estimated East acceleration');

figure(6);
plot(t_imu,Imu(3,:),'k'); hold on; grid on;
plot(Tc,az_estimation,'m'); hold on;
legend('Imu Down acceleration','estimated Down acceleration');

figure(7);
plot(t_gps,VX,'k'); hold on; grid on;
plot(Tc,vx_estimation,'m'); hold on;
legend('North velocity','estimated North velocity');

figure(8);
plot(t_gps,VY,'k'); hold on; grid on;
plot(Tc,vy_estimation,'m'); hold on;
legend('East velocity','estimated East velocity');

figure(9);
plot(t_gps,VZ,'k'); hold on; grid on;
plot(Tc,vz_estimation,'m'); hold on;
legend('Down velocity','estimated Down velocity');


function  [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,k,Imu)
F = feval(f,dt);
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



function [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k)
    counter = 0;
    if selection_vector(1) == false 
        H(1:3,:) = [];
        R(1:3,:) = [];
        R(:,1:3) = [];
        counter = counter+3;
    end
    if selection_vector(2) == false  
        H(4-counter:6-counter,:) = [];
        R(4-counter:6-counter,:) = [];
        R(:,4-counter:6-counter) = [];
    end

if (selection_vector(1) == true && selection_vector(2) == true) 
    S = R+H*P*H';
    S_gps = S(1:3,1:3);
    S_imu = S(4:6,4:6);
    innovation = actual_meas-H*X_hat;
    innovation_gps = innovation(1:3);
    innovation_imu = innovation(4:6);
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
        H(1:3,:) = []; %3x9
        R(1:3,:) = [];
        R(:,1:3) = []; %3x3
        L = P*H'*inv(S_imu); %9x6
        X_hat_imu = X_hat + L*innovation_imu; % x_imu(k|k)
        X_hat = beta0*X_hat + beta_imu*X_hat_imu;
        P_imu = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9
        P = beta0*P + beta_imu*(P_imu+(X_hat-X_hat_imu)*(X_hat-X_hat_imu)');
    end
    if (q_gps < 7.8 && q_imu > 7.8) %only Gps measure
        H(4:6,:) = []; %3x9
        R(4:6,:) = [];
        R(:,4:6) = []; %3x3
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