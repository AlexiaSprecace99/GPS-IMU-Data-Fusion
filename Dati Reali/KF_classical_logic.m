%In this Script there is an implementation of a Kalman Filter for GPS/IMU data fusion
%with the introduction of contextual aspect. In particular, a confidence interval was 
% introduced within which the measures must be found for them to be used in the correction step. 
% This confidence interval is defined by the 'Chi square' distribution.
initialization
%Measure
% pos = log_vars.trajectory_gen;
% vel = log_vars.velocity_gen;
% acc = log_vars.acceleration_gen;
% meas = [pos;acc];
rand_acc = 0.05*randn(3,1);

%Initialization of Matlab Function
f = matlabFunction(F);
k = 1;
log_EKF = [];
selection_vector = [false false false]';  
flag = [0 0 0 ]'; 
actual_meas = [0 0 0 0 0 0 0 0]';  
count = 0;
n = 100;
Tc = 0:0.02:t_max;
Td = 0:0.02:245.74;
Td = Td*(max(Tc)/max(Td));
log_EKF.x_hat(:,1) = X_hat;
for t = 0:dt:t_max
    %prediction step
    [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,k,Imu);
    log_EKF.x_hat(:,k) = X_hat;

    
    [actual_meas, selection_vector, flag] = getActualMeas(ts,ta,tv,flag, selection_vector,t);
    % correction step
    [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k);


%     error_x(1,k) = trajectory_gen(1,k)-log_EKF.x_hat(1,k);
%     error_y(1,k) = trajectory_gen(2,k)-log_EKF.x_hat(2,k);
%     error_z(1,k) = trajectory_gen(3,k)-log_EKF.x_hat(3,k);
% 
%     error_ax(1,k) = acceleration_gen(1,k)-log_EKF.x_hat(7,k);
%     error_ay(1,k) = acceleration_gen(2,k)-log_EKF.x_hat(8,k);
%     error_az(1,k) = acceleration_gen(3,k)-log_EKF.x_hat(9,k);


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
plot(Tc,velocita_x,'b');hold on;
plot(Tc,vx_estimation,'r'); hold on;  grid on;
legend('estimated North position derivative','estimated North velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]')

figure;
plot(Tc,velocita_y,'b'); hold on;  grid on;
plot(Tc,vy_estimation,'r'); hold on;
xlabel('T[s]'); ylabel('Velocity[m/s]');
legend('estimated East position derivative','estimated East velocity');

figure;
plot(Tc,velocita_z,'b'); hold on;
plot(Tc,vz_estimation,'r'); hold on; grid on;
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
plot(Tc,x_estimation);
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
    else
        count_size_meas = count_size_meas + 1;
        selection_vector(1) = true;     
         actual_meas = [ta.data(:,flag(1));tv.data(1:2,flag(2))];    
        %actual_meas = [actual_meas;tv.data(:,flag(1))];
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

if (selection_vector(1) == true && selection_vector(3) == true) %there are both measures
    S = R+H*P*H';
    S_gps = S(1:5,1:5);
    S_imu = S(6:8,6:8);
    innovation = actual_meas-H*X_hat;
    innovation_gps = innovation(1:5);
    innovation_imu = innovation(6:8);
    
    %Calculation of the values(q_gps,q_imu) to see if they are within the chosen confidence interval, 
    %constructed using Chi square distribution. In this case, the limit value chosen is 7.8, 
    %found with a 3-degree-of-freedom distribution and a 95% confidence interval 
    %If the measurements are below this limit then they are used for correction 
    %since they are considered more reliable

    q_gps = innovation_gps'*inv(S_gps)*innovation_gps;
    q_imu = innovation_imu'*inv(S_imu)*innovation_imu;

    if(q_gps > 7.8 && q_imu < 7.8) %takes only Imu measures
        H(1:5,:) = []; %3x9
        R(1:5,:) = [];
        R(:,1:5) = []; %3x3
        L = P*H'*inv(S_imu); %9x6
        X_hat = X_hat + L*innovation_imu; %9x1
        P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9
    end
    if (q_gps < 7.8 && q_imu > 7.8) %takes only Gps measures
        H(6:8,:) = []; %3x9
        R(6:8,:) = [];
        R(:,6:8) = []; %3x3
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

if (selection_vector(1) == false && selection_vector(3) == true ) % just Imu measures
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

if (selection_vector(1) == true && selection_vector(3) == false) %just Gps measure
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
if (selection_vector(1) == false && selection_vector(3) == false )
        X_hat = X_hat;
        P = P;
end
end

