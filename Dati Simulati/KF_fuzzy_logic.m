%In this Script there is an implementation of a Kalman Filter for GPS/IMU data fusion
%with the introduction of contextual aspect. In particular, in particular, after introducing 
% the confidence interval using the chi-square distribution, 
% fuzzy logic was added to try to reject unreliable measures   

%Measure
pos = log_vars.trajectory_gen;
vel = log_vars.velocity_gen;
acc = log_vars.acceleration_gen;
meas = [pos;acc];
Tc = 0:1/75:t_max;

%Initialization of Matlab Function
f = matlabFunction(F);
k = 1;
i = 1;
log_EKF = [];
selection_vector = [false false]';  
flag = [0 0]'; 
actual_meas = [0 0 0 0 0 0]'; 
count = 0;
n= 100;
log_EKF.x_hat(:,1) = X_hat;
for t = 0:dt:t_max

    rand_pos = 0.1*randn(3,1);
    rand_acc = 0.001*randn(3,1);

    %prediction step
    [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,acceleration_gen,k,rand_acc);
    log_EKF.x_hat(:,k) = X_hat;
   
    
    [actual_meas, selection_vector, flag] = getActualMeas(ts,ta, flag, selection_vector,t,rand_pos,rand_acc);
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

figure;
plot(Tc,trajectory_gen(1,:)-pos_gps_interp_x,'r'); hold on; grid on; 
plot(Tc,trajectory_gen(1,:)-log_EKF.x_hat(1,:),'b');hold on; grid on; 
legend('North error between real and measured trajectory','North error between real and estimated trajectory');

figure;
plot(Tc,trajectory_gen(2,:)-pos_gps_interp_y,'r'); hold on; grid on; 
plot(Tc,trajectory_gen(2,:)-log_EKF.x_hat(2,:),'b');hold on; grid on; 
legend('East error between real and measured trajectory','East error between real and estimated trajectory');

figure;
plot(Tc,trajectory_gen(3,:)-pos_gps_interp_z,'r'); hold on; grid on; 
plot(Tc,trajectory_gen(3,:)-log_EKF.x_hat(3,:),'b');hold on; grid on; 
legend('Down error between real and measured trajectory','Down error between real and estimated trajectory');

figure;
plot(Tc,velocity_gen(1,:),'r', LineWidth=1);hold on;
plot(Tc,log_EKF.x_hat(4,:),'b', LineWidth=1); hold on;  grid on;
legend('Real North velocity','estimated North velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]')

figure;
plot(Tc,velocity_gen(2,:),'r', LineWidth=1); hold on;  grid on;
plot(Tc,log_EKF.x_hat(5,:),'b', LineWidth=1); hold on;
xlabel('T[s]'); ylabel('Velocity[m/s]')
legend('Real East velocity','estimated East velocity')

figure;
plot(Tc,velocity_gen(3,:),'r', LineWidth=1); hold on;
plot(Tc,log_EKF.x_hat(6,:),'b',LineWidth=1); hold on; grid on;
legend('Real Down velocity','estimated Down velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]')

figure;
plot(Tc,vera_acc(1,:),'g', LineWidth=1); hold on;  grid on;
plot(Tc,log_EKF.x_hat(7,:),'b', LineWidth=1); hold on;  grid on;
plot(Tc,acceleration_gen(1,:),'r', LineWidth=1.5);hold on;
legend('Measured North acceleration','estimated North acceleration','Real North acceleration');%,'measured North acceleration');
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]')

figure;
plot(Tc,vera_acc(2,:),'g', LineWidth=1); hold on;  grid on;
plot(Tc,log_EKF.x_hat(8,:),'b', LineWidth=1); hold on; grid on;
plot(Tc,acceleration_gen(2,:),'r', LineWidth=1.5);hold on;
legend('Measured East acceleration','estimated East acceleration','Real East acceleration');%,'measured East acceleration');
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]')

figure;
plot(Tc,vera_acc(3,:),'g', LineWidth=1); hold on;  grid on;
plot(Tc,log_EKF.x_hat(9,:),'b',LineWidth=1); hold on; grid on;
plot(Tc,acceleration_gen(3,:),'r', LineWidth=1.5); hold on;
%plot(Tc,vera_acc(3,:),'b', LineWidth=1); hold on;  grid on;
legend('Measured Down acceleration','estimated Down acceleration','Real Down acceleration');%,'measured Down acceleration');
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]')

figure;
plot(Tc,trajectory_gen(1,:),'r');hold on;
plot(Tc,pos_gps_interp_x,'g');hold on;
% plot(Tc,trajectory_gen(2,:),'g', LineWidth=4.5); hold on;  grid on;
% plot(Tc,trajectory_gen(3,:),'k', LineWidth=4.5); hold on;
plot(Tc,log_EKF.x_hat(1,:),'b'); hold on;  grid on;
% plot(Tc,log_EKF.x_hat(2,:),'r', LineWidth=1.5); hold on;
% plot(Tc,log_EKF.x_hat(3,:),'m',LineWidth=1.5); hold on; grid on;
% legend('gps North position','gps East position', 'gps Down position','estimated North position','estimated East position','estimated Down position');
legend('Real North position','GPS North position','Estimated North position');
xlabel('T[s]'); ylabel('Position[m]');

figure;
plot(Tc,trajectory_gen(2,:),'r');hold on;
plot(Tc,pos_gps_interp_y,'g');hold on;
plot(Tc,log_EKF.x_hat(2,:),'b'); hold on;  grid on;
legend('Real East position','GPS East position','Estimated East position');
xlabel('T[s]'); ylabel('Position[m]');

figure;
plot(Tc,trajectory_gen(3,:),'r');hold on;
plot(Tc,pos_gps_interp_z,'g');hold on;
plot(Tc,log_EKF.x_hat(3,:),'b'); hold on;  grid on;
legend('Real Down position','GPS Down position','Estimated Down position');
xlabel('T[s]'); ylabel('Position[m]');


function  [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,acceleration_gen,k,rand_acc)
F = feval(f,dt); %F matrix depends on the sampling time
X_hat(7:9,1) = acceleration_gen(:,k)+rand_acc; 
X_hat = F*X_hat+[dt^3/6;dt^3/6;dt^3/6;dt^2/2;dt^2/2;dt^2/2;dt;dt;dt]*0.01*randn(1);
P = F*P*F'+Q;
end

function [actual_meas, selection_vector, flag] = getActualMeas(ts,ta,flag, selection_vector,t,rand_pos,rand_acc)
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
        actual_meas = ta.data(:,flag(1)) + rand_pos;
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
            actual_meas = [actual_meas;ts.data(:,flag(2))] +[rand_pos;rand_acc];    
        else
            selection_vector(2) = true;    
            actual_meas = ts.data(:,flag(2)) + rand_acc;     
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
end