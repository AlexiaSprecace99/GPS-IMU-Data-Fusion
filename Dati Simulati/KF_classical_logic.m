%In this Script there is an implementation of a Kalman Filter for GPS/IMU data fusion
%with the introduction of contextual aspect. In particular, a confidence interval was 
% introduced within which the measures must be found for them to be used in the correction step. 
% This confidence interval is defined by the 'Chi square' distribution.

%Measure
pos = log_vars.trajectory_gen;
vel = log_vars.velocity_gen;
acc = log_vars.acceleration_gen;
meas = [pos;acc];
rand_acc = 0.05*randn(3,1);
Tc = 0:1/75:t_max;

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
    [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,log_vars,k);
    log_EKF.x_hat(:,k) = X_hat;

    
    [actual_meas, selection_vector, flag] = getActualMeas(ts,ta, flag, selection_vector,t);
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
grid on;
figure(1);
%figure(2);

plot(Tc,trajectory_gen(1,:),'c', LineWidth=4.5);hold on;
plot(Tc,trajectory_gen(2,:),'g', LineWidth=4.5); hold on;  grid on;
plot(Tc,trajectory_gen(3,:),'k', LineWidth=4.5); hold on;
plot(Tc,log_EKF.x_hat(1,:),'b', LineWidth=1.5); hold on;  grid on;
plot(Tc,log_EKF.x_hat(2,:),'r', LineWidth=1.5); hold on;
plot(Tc,log_EKF.x_hat(3,:),'m',LineWidth=1.5); hold on; grid on;


%legend('gps East position','estimated East position');


%figure(3);


legend('gps North position','gps East position', 'gps Down position','estimated North position','estimated East position','estimated Down position');
xlabel('T[s]'); ylabel('Position[m]')

figure(3);
plot(Tc,velocity_gen(1,:),'c', LineWidth=4.5);hold on;
plot(Tc,log_EKF.x_hat(4,:),'b', LineWidth=1.5); hold on;  grid on;
legend('gps North velocity','estimated North velocity')
xlabel('T[s]'); ylabel('Velocity[m/s]')
figure(4)
plot(Tc,velocity_gen(2,:),'g', LineWidth=4.5); hold on;  grid on;
plot(Tc,log_EKF.x_hat(5,:),'r', LineWidth=1.5); hold on;
xlabel('T[s]'); ylabel('Velocity[m/s]')
legend('gps East velocity','estimated East velocity')
figure(5)
plot(Tc,velocity_gen(3,:),'k', LineWidth=4.5); hold on;
plot(Tc,log_EKF.x_hat(6,:),'m',LineWidth=1.5); hold on; grid on;
legend('gps Down velocity','estimated Down velocity');
xlabel('T[s]'); ylabel('Velocity[m/s]')

figure(6);
plot(Tc,acceleration_gen(1,:),'c', LineWidth=4.5);hold on;
plot(Tc,log_EKF.x_hat(7,:),'b', LineWidth=1.5); hold on;  grid on;
legend('gps North acceleration','estimated North acceleration')
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]')
figure(7);
plot(Tc,acceleration_gen(2,:),'c', LineWidth=4.5);hold on;
plot(Tc,log_EKF.x_hat(8,:),'r', LineWidth=1.5); hold on;
legend('gps East acceleration','estimated East acceleration')
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]')
figure(8);
plot(Tc,acceleration_gen(3,:),'k', LineWidth=4.5); hold on;
plot(Tc,log_EKF.x_hat(9,:),'m',LineWidth=1.5); hold on; grid on;
legend('gps Down acceleration','estimated Down acceleration')
xlabel('T[s]'); ylabel('Acceleration[m/s^{2}]')

grid on;
figure(9); 
plot(Tc,trajectory_gen(1,:)-log_EKF.x_hat(1,:),'c'); grid on;
legend('North position error');
xlabel('T[s]'); ylabel('Error position [m]');

figure(10); grid on;
plot(Tc,trajectory_gen(2,:)-log_EKF.x_hat(2,:),'c');grid on;
legend('East position error');
xlabel('T[s]'); ylabel('Error position [m]');

figure(11); grid on;
plot(Tc,trajectory_gen(3,:)-log_EKF.x_hat(3,:),'c');grid on;
legend('Down position error');
xlabel('T[s]'); ylabel('Error position [m]');


function  [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,log_vars,k)
F = feval(f,dt);
X_hat(7:9,1) = log_vars.acceleration_gen(:,k);
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
        actual_meas = ta.data(:,flag(1)) + 0.1*randn(3,1);
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
            actual_meas = [actual_meas;ts.data(:,flag(2))]+[0.1*randn(3,1);0.01*randn(3,1)];    
        else
            selection_vector(2) = true;    
            actual_meas = ts.data(:,flag(2))+ 0.01*randn(3,1);     
        end
    end
end



function [X_hat, P] = correction_KF(X_hat, P, actual_meas,selection_vector,H,R,t,k)
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

if (selection_vector(1) == true && selection_vector(2) == true) %there are both measures
    S = R+H*P*H';
    S_gps = S(1:3,1:3);
    S_imu = S(4:6,4:6);
    innovation = actual_meas-H*X_hat;
    innovation_gps = innovation(1:3);
    innovation_imu = innovation(4:6);
    
    %Calculation of the values(q_gps,q_imu) to see if they are within the chosen confidence interval, 
    %constructed using Chi square distribution. In this case, the limit value chosen is 7.8, 
    %found with a 3-degree-of-freedom distribution and a 95% confidence interval 
    %If the measurements are below this limit then they are used for correction 
    %since they are considered more reliable

    q_gps = innovation_gps'*inv(S_gps)*innovation_gps;
    q_imu = innovation_imu'*inv(S_imu)*innovation_imu;

    if(q_gps > 7.8 && q_imu < 7.8) %takes only Imu measures
        H(1:3,:) = []; %3x9
        R(1:3,:) = [];
        R(:,1:3) = []; %3x3
        L = P*H'*inv(S_imu); %9x6
        X_hat = X_hat + L*innovation_imu; %9x1
        P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9
    end
    if (q_gps < 7.8 && q_imu > 7.8) %takes only Gps measures
        H(4:6,:) = []; %3x9
        R(4:6,:) = [];
        R(:,4:6) = []; %3x3
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

