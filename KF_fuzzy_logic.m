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
initialization
Tc = 0:0.02:t_max;
Td = 0:0.02:245.74;
Td = Td*(max(Tc)/max(Td));
%Initialization of Matlab Function
f = matlabFunction(F);
k = 1;
log_EKF = [];
selection_vector = [false false false]';  
flag = [0 0 0]'; 
actual_meas = [0 0 0 0 0 0 0 0]'; 
count = 0;
n= 100;
log_EKF.x_hat(:,1) = X_hat;
for t = 0:dt:t_max
    %prediction step
    [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,k,Imu);
    log_EKF.x_hat(:,k) = X_hat;
   
    
    [actual_meas, selection_vector, flag] = getActualMeas(ts,ta,tv, flag, selection_vector,t);
    % correction step
    [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k);


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

X_p_interp = interp1(timu,DATA(12290:24577,[1]+19),t_new_a,'linear');
Y_p_interp = interp1(timu,DATA(12290:24577,[4]+19),t_new_a,'linear');
Z_p_interp = interp1(timu,DATA(12290:24577,[7]+19),t_new_a,'linear');

VX_p_interp = interp1(timu,DATA(12290:24577,[1]+20),t_new_a,'linear');
VY_p_interp = interp1(timu,DATA(12290:24577,[1]+23),t_new_a,'linear');
VZ_p_interp = interp1(timu,DATA(12290:24577,[1]+26),t_new_a,'linear');

cutOffFreq = 1; % Frequenza di taglio del filtro (in Hz)
samplingFreq = 50; % Frequenza di campionamento (in Hz)
[b, a] = butter(2, cutOffFreq / (samplingFreq / 2), 'low');

figure;
plot(t_gps,GPS(1,:),'b');hold on;
plot(Tc,x_estimation,'r'); hold on;  grid on;
legend('gps North position','estimated North position');
xlabel('T[s]'); ylabel('Position[m]');

figure;
plot(t_gps,GPS(2,:),'b'); hold on;  grid on;
plot(Tc,y_estimation,'r'); hold on;
legend('gps East position','estimated East position');
xlabel('T[s]'); ylabel('Position[m]');

figure;
plot(t_gps,GPS(3,:),'b'); hold on;
plot(Tc,z_estimation,'r'); hold on; grid on;
legend('gps Down position','estimated Down position');
xlabel('T[s]'); ylabel('Position[m]');


delta_posizione_x = diff(x_estimation);
velocita_x = [delta_posizione_x./dt delta_posizione_x(end)/dt];
velocita_x = filter(b, a, velocita_x);


delta_posizione_y = diff(y_estimation);
velocita_y = [delta_posizione_y./dt delta_posizione_y(end)/dt];
velocita_y = filter(b, a, velocita_y);


delta_posizione_z = diff(z_estimation);
velocita_z = [delta_posizione_z./dt delta_posizione_z(end)/dt];
velocita_z = filter(b, a, velocita_z);


figure;
plot(Tc,velocita_x,'g');hold on;
plot(Tc,vx_estimation,'m'); hold on;  grid on;
legend('estimated North position derivative','estimated North velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]')

figure;
plot(Tc,velocita_y,'g'); hold on;  grid on;
plot(Tc,vy_estimation,'m'); hold on;
xlabel('T[s]'); ylabel('Velocity[m/s]');
legend('estimated East position derivative','estimated East velocity');

figure;
plot(Tc,velocita_z,'g'); hold on;
plot(Tc,vz_estimation,'m'); hold on; grid on;
legend('estimated Down position derivative','estimated Down velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]');

figure;
plot(Tc,VX_p_interp,'b'); hold on;
plot(Tc,vz_estimation,'r'); hold on; grid on;
legend('measured North velocity','estimated North velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]');

figure;
plot(Tc,VY_p_interp,'b'); hold on;
plot(Tc,vz_estimation,'r'); hold on; grid on;
legend('measured East velocity','estimated East velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]');


figure;
plot(t_imu,Imu(1,:),'b');hold on;
plot(Tc,ax_estimation,'r'); hold on;  grid on;
legend('gps North acceleration','estimated North acceleration');
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]');

figure;
plot(t_imu,Imu(2,:),'b');hold on;
plot(Tc,ay_estimation,'r'); hold on; grid on;
legend('gps East acceleration','estimated East acceleration');
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]');

figure;
plot(t_imu,Imu(3,:),'b'); hold on;
plot(Tc,az_estimation,'r'); hold on; grid on;
legend('gps Down acceleration','estimated Down acceleration');
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]');

figure;
plot(Tc,Z_p_interp,'b');hold on; grid on;
plot(Tc,z_estimation,'r');
legend('Prof Down position estimate','Our Down position estimate');
xlabel('T[s]'); ylabel('Position[m]');

figure;
plot(Tc,X_p_interp,'b');hold on; grid on;
plot(Tc,x_estimation,'r');
legend('Prof North position estimate','Our North position estimate');
xlabel('T[s]'); ylabel('Position[m]');

figure;
plot(Tc,Y_p_interp,'b');hold on; grid on;
plot(Tc,y_estimation);
legend('Prof East position estimate','Our East position estimate');
xlabel('T[s]'); ylabel('Position[m]');

figure;
plot(Tc, VZ_p_interp,'b'); hold on; grid on;
plot(Tc, vz_estimation,'r');
legend('Prof estimated Down velocity','Our estimated Down velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]');

figure;
plot(Tc, VY_p_interp,'b'); hold on; grid on;
plot(Tc, vy_estimation,'r');
legend('Prof estimated East velocity','Our estimated East velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]');

figure;
plot(Tc, VX_p_interp,'b'); hold on; grid on;
plot(Tc, vx_estimation,'r');
legend('Prof estimated North velocity','Our estimated North velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]');

function  [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,k,Imu)
F = feval(f,dt);
X_hat(7:9,1) = Imu(:,k);
X_hat = F*X_hat +[dt^3/6;dt^3/6;dt^3/6;dt^2/2;dt^2/2;dt^2/2;dt;dt;dt]*0.01*randn(1);
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
    else
        count_size_meas = count_size_meas + 1;
        selection_vector(1) = true;     
         actual_meas = [ta.data(:,flag(1));tv.data(1:2,flag(2))];    
        %actual_meas = [actual_meas;tv.data(:,flag(1))];
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
            actual_meas = [actual_meas;ts.data(:,flag(3))]; %+[0.01*randn(3,1);0.05*randn(3,1)];    
        else
            selection_vector(3) = true;    
            actual_meas = ts.data(:,flag(3)) ;%+ 0.05*randn(3,1);     
        end
    end
end



function [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k)
    counter = 0;
    if selection_vector(1) == false  %if there aren't any information of position
        H(1:5,:) = [];
        R(1:5,:) = [];
        R(:,1:5) = [];
        counter = counter+5;
    end
    if selection_vector(3) == false  %if there aren't any information of acceleration
        H(6-counter:8-counter,:) = [];
        R(6-counter:8-counter,:) = [];
        R(:,6-counter:8-counter) = [];
    end

if (selection_vector(1) == true && selection_vector(3) == true)
    S = R+H*P*H';
    S_gps = S(1:5,1:5);
    S_imu = S(6:8,6:8);
    innovation = actual_meas-H*X_hat;
    innovation_gps = innovation(1:5);
    innovation_imu = innovation(6:8);
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
        H(1:5,:) = []; %3x9
        R(1:5,:) = [];
        R(:,1:5) = []; %3x3
        L = P*H'*inv(S_imu); %9x6
        X_hat_imu = X_hat + L*innovation_imu; % x_imu(k|k)
        X_hat = beta0*X_hat + beta_imu*X_hat_imu;
        P_imu = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9
        P = beta0*P + beta_imu*(P_imu+(X_hat-X_hat_imu)*(X_hat-X_hat_imu)');
    end
    if (q_gps < 7.8 && q_imu > 7.8) %only Gps measure
        H(6:8,:) = []; %3x9
        R(6:8,:) = [];
        R(:,6:8) = []; %3x3
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

if (selection_vector(1) == false && selection_vector(3) == true) %just acceleration information
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

if (selection_vector(1) == true && selection_vector(3) == false) %just position information
    S_gps = R+H*P*H';
    innovation_gps = actual_meas-H*X_hat;
    q_gps = innovation_gps'*inv(S_gps)*innovation_gps;
    mu_gps = get_mf_valid(q_gps);
    beta_gps = mu_gps;
    beta0 = 1-mu_gps;
    
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

if (selection_vector(1) == false && selection_vector(3) == false ) %no measure available
        beta0 = 1;
        X_hat = beta0*X_hat;
        P = beta0*P;
end
end