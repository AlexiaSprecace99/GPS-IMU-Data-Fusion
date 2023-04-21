%In this Script there is an implementation of a Kalman Filter for GPS/IMU data fusion
%Measure
pos = log_vars.trajectory_gen;
vel = log_vars.velocity_gen;
acc = log_vars.acceleration_gen;
meas = [pos;vel;acc];

%Initialization of Matlab Function
f = matlabFunction(F);

k = 1;
log_EKF = [];
selection_vector = [false false false false false false false false false]';  % seleziona quali misure sono state usate all'iterazione corrente
flag = [0 0 0 0 0 0 0 0 0]';  % tiene traccia dell'indice delle misure più recenti già utilizzate per ogni sensore
actual_meas = [0 0 0 0 0 0 0 0 0]';   % contiene i valori delle misure utilizzate all'iterazione corrente
log_EKF.x_hat(:,1) = X_hat;
for t = dt:dt:t_max
    %prediction step
    [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,log_vars,k);
    log_EKF.x_hat(:,k+1) = X_hat;
    % restituisce ad ogni passo il vettore con le misure dei sensori
    % effettive e disponibili che non erano già state prese
    %[actual_meas, selection_vector, flag] = getActualMeas(x,y,z,vx,vy,vz,ax,ay,az,flag,selection_vector, k, dt);
                                                       
   % error_x(1,k) = trajectory_gen(1,k)-log_EKF.x_hat(1,k); 
    %error_y(1,k) = trajectory_gen(2,k)-log_EKF.x_hat(2,k);
    %error_z(1,k) = trajectory_gen(3,k)-log_EKF.x_hat(3,k);

    % correction step
    [X_hat, P] = correction_KF(X_hat, P, R_gps,R_imu,H_gps,H_imu, meas, pos,acc,selection_vector,k);


    error_x(1,k) = trajectory_gen(1,k)-log_EKF.x_hat(1,k); 
    error_y(1,k) = trajectory_gen(2,k)-log_EKF.x_hat(2,k);
    error_z(1,k) = trajectory_gen(3,k)-log_EKF.x_hat(3,k);

    k = k + 1;
end

plot3(trajectory_gen(1,:),trajectory_gen(2,:),trajectory_gen(3,:),'r');
hold on;
plot3(log_EKF.x_hat(1,:),log_EKF.x_hat(2,:),log_EKF.x_hat(3,:),'b');
grid on;
figure; grid on; 
plot(error_x);

figure; grid on; 
plot(error_y);

figure; grid on; 
plot(error_z);

function  [X_hat, P] = prediction_KF(X_hat, P, Q, dt,f,log_vars,k)
F = feval(f,dt);
X_hat(7:9,1) = log_vars.acceleration_gen(:,k);
X_hat = F*X_hat;
P = F*P*F'+Q;
end

function [X_hat, P] = correction_KF(X_hat, P, R_gps,R_imu,H_gps,H_imu, meas,pos,acc, selection_vector,k)

H = [eye(3) zeros(3) zeros(3);zeros(3) zeros(3) eye(3)];
R = blkdiag(R_gps,R_imu);
z = [pos+0.01*randn(3,1);acc+0.05*randn(3,1)];
innovation = z(:,k)-H*X_hat;

S = R+H*P*H'; %6x6
L = P*H'*inv(S); %9x6
X_hat = X_hat + L*innovation; %9x1
P = (eye(9)-L*H)*P*(eye(9)-L*H)'+L*R*L'; %9x9

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