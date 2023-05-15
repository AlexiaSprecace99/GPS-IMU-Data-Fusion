%%% Rauch–Tung–Striebel Smoother
clc

load('KF_struct.mat')

% The forward procedure is carried out in the EKF_smoother.mat file where
% we derive x_hat(k|k), x_hat(k+1|k), P(k|k), P(k+1|k)

%x_hat(n|n)
log_KF(size(log_KF,2)).x_hat_smoothed = log_KF(size(log_KF,2)).x_hat_corr;

%P(n|n)
log_KF(size(log_KF,2)).P_smoothed = log_KF(size(log_KF,2)).P_corr;

% Backward procedure
for k = size(log_KF,2)-1:-1:1
    Ck = log_KF(k).P_corr*(log_KF(k+1).F_matrix)'*(log_KF(k+1).P_pred)^-1;

    log_KF(k).x_hat_smoothed = log_KF(k).x_hat_corr + Ck* ... 
        (log_KF(k+1).x_hat_smoothed - log_KF(k+1).x_hat_pred);

    log_KF(k).P_smoothed = log_KF(k).P_corr + Ck* ... 
        (log_KF(k+1).P_smoothed - log_KF(k+1).P_pred)*Ck';


end

for k = 1:1:size(log_KF,2)
         
    x_hat_corr_x(k)= log_KF(:,k).x_hat_corr(1);
    x_hat_corr_y(k)= log_KF(:,k).x_hat_corr(2);
    x_hat_corr_z(k)= log_KF(:,k).x_hat_corr(3);
    x_hat_corr_vx(k)= log_KF(:,k).x_hat_corr(4);
    x_hat_corr_vy(k)= log_KF(:,k).x_hat_corr(5);
    x_hat_corr_vz(k)= log_KF(:,k).x_hat_corr(6);
    x_hat_corr_ax(k)= log_KF(:,k).x_hat_corr(7);
    x_hat_corr_ay(k)= log_KF(:,k).x_hat_corr(8);
    x_hat_corr_az(k)= log_KF(:,k).x_hat_corr(9);
    x_hat_smoothed_x(k)= log_KF(:,k).x_hat_smoothed(1);
    x_hat_smoothed_y(k)= log_KF(:,k).x_hat_smoothed(2);
    x_hat_smoothed_z(k)= log_KF(:,k).x_hat_smoothed(3);
    x_hat_smoothed_vx(k)= log_KF(:,k).x_hat_smoothed(4);
    x_hat_smoothed_vy(k)= log_KF(:,k).x_hat_smoothed(5);
    x_hat_smoothed_vz(k)= log_KF(:,k).x_hat_smoothed(6);
    x_hat_smoothed_ax(k)= log_KF(:,k).x_hat_smoothed(7);
    x_hat_smoothed_ay(k)= log_KF(:,k).x_hat_smoothed(8);
    x_hat_smoothed_az(k)= log_KF(:,k).x_hat_smoothed(9);
    
%     x_hat_corr_tot_in = x_hat_corr_tot;
%     x_hat_smooth_tot = [log_EKF(:,k).x_hat_smoothed];
end

[x_estimation]=[x_hat_smoothed_x(1,:)];
[y_estimation]=[x_hat_smoothed_y(1,:)];
[z_estimation] = [x_hat_smoothed_z(1,:)];
[ax_estimation] = [x_hat_smoothed_ax(1,:)];
[ay_estimation] = [x_hat_smoothed_ay(1,:)];
[az_estimation] = [x_hat_smoothed_az(1,:)];
[vx_estimation] = [x_hat_smoothed_vx(1,:)];
[vy_estimation] = [x_hat_smoothed_vy(1,:)];
[vz_estimation] = [x_hat_smoothed_vz(1,:)];

grid on;
figure(1);
plot(t_gps,GPS(1,:),'r');hold on; grid on;
plot(Tc,x_estimation,'b'); 
legend('gps North position','estimated North position');

figure(2);
plot(t_gps,GPS(2,:),'g'); hold on; grid on;
plot(Tc,y_estimation,'y');
legend('gps East position','estimated East position');

figure(3);
plot(t_gps,GPS(3,:),'k'); hold on; grid on;
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


% figure(4)
% plot(t_gps,Gps(1,:)-x_estimation); grid on;
% legend('North position error');
% 
% figure(5)
% plot(t_gps,Gps(2,:)-y_estimation); grid on;
% legend('East position error');
% 
% figure(6)
% plot(t_gps,Gps(3,:)-z_estimation); grid on;
% legend('Down position error');
% xlim([min(x_hat_smooth_x) max(x_hat_smooth_x)])
% ylim([min(x_hat_smooth_y) max(x_hat_smooth_y)])
% 
% estimated_plot = plot(x_hat_corr_x, x_hat_corr_y,'r', 'LineWidth', 1.2);
% smoothed_plot = plot(x_hat_smooth_x, x_hat_smooth_y,'b', 'LineWidth', 1.2);
% 
% 
% ground_init_truth = plot(log_vars.s_Tx(1), log_vars.s_Ty(1), 'go', 'MarkerFaceColor', 'g');
% estimated_init_plot = plot(log_EKF.x_hat_correction(1,1), log_EKF.x_hat_correction(1,2),'ro', 'MarkerFaceColor', 'r');
% smoothed_init_plot = plot(log_EKF.x_hat_smoothed(1,1), log_EKF.x_hat_smoothed(1,2),'bo', 'MarkerFaceColor', 'b');
%