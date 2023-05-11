%%% Rauch–Tung–Striebel Smoother
clc

load('KF_struct.mat')

% La procedura in avanti viene svolta nel file EKF_smoother.mat dove si
% ricavano x_hat(k|k), x_hat(k+1|k), P(k|k), P(k+1|k)

%x_hat(n|n)
log_KF(size(log_KF,2)).x_hat_smoothed = log_KF(size(log_KF,2)).x_hat_corr;

%P(n|n)
log_KF(size(log_KF,2)).P_smoothed = log_KF(size(log_KF,2)).P_corr;

%Procedura in indietro
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
grid on;
figure(1);
plot(Tc,position_complete(1,:),'r');hold on; grid on;
plot(Tc,x_estimation,'b'); 
legend('gps x','estim x');

figure(2);
plot(Tc,position_complete(2,:),'g'); hold on; grid on;
plot(Tc,y_estimation,'y');
legend('gps y','estim y');

figure(3);
plot(Tc,position_complete(3,:),'k'); hold on; grid on;
plot(Tc,z_estimation,'m'); hold on;
legend('gps z','estim z');

figure(4)
plot(Tc,position_complete(1,:)-x_estimation); grid on;
legend('position error x');

figure(5)
plot(Tc,position_complete(2,:)-y_estimation); grid on;
legend('position error y');

figure(6)
plot(Tc,position_complete(3,:)-z_estimation); grid on;
legend('position error z');
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