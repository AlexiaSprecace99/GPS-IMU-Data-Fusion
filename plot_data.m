% tarot test flight data plot
%% telemetria da TELEMETRY_KF_ARUCO_TEST, 46 segnali 50 Hz 
% SYSTEM TIME x1
% RPY_MPU x3
% GYRO_MPU x3
% ACC_MPU x3
% MAG_HMC x3
%% muxed signals, RC control e debug h x6
% TRPY ref x4
% MODE x1
% V batt x1

% debug_h x5
% H_d x1

%%
% NAVDATA x13
% KF_MEAS x15
% KF_flags x1 (init, gps, baro, aruco)

%% muxed signals, debug position 8 e 8
% debug_position x8 (pos_ref x, vel_ref x, controllo di velocità in x(cmd prop,
% cmd integr, pos_ref y, vel_ref y, controllo di velocità in y(cmd prop,
% cmd integr) 
% debug_position x8 (los_angle, ultrasound,downtrack,crosstrack, WPen, lat,
% lon , alt)



%% demux muxed data 
% adjust jitter in time values when close to sample time 
Tt = T;
for i=2:length(T),
    deltat = T(i)- Tt(i-1); 
    if deltat > 0.022 && deltat < 0.027,
        %no missing data BUT sample time a little bit longer -> restore good value
        Tt(i) = Tt(i-1)+0.02;
    elseif deltat < 0.018,
        %no missing data BUT sample time a little bit shorter -> restore good value
        Tt(i) = Tt(i-1)+0.02;
    elseif deltat >= 0.027 
        %missing data -> must find exact value that is multiple of sample time
        count = 0;
        while deltat >= 0.018,
            deltat = deltat-0.02;
            count = count+1;
        end
        Tt(i) = Tt(i-1)+count*0.02;
    else
        %this is a good sample already -> do nothing
    end
end
%debug plot
figure,
plot(diff(T),'bo'), hold on, plot(diff(Tt),'rs'), grid on
% demux
T_100 = Tt.*100;
t_100_demux = mod(T_100,4);
figure, plot(t_100_demux,'bo'), grid on;
%
x_idx  = find(t_100_demux < 1 | t_100_demux >3);
y_idx  = find(t_100_demux > 1 & t_100_demux <3);
%
% now length(x_idx)+length(y_idx) must be = length(T)

% SET FIRMWARE REVISION
%
%1 - firmware version with debug_pos_control only (muxed data in x and y at positions 41:45
%2 - firmware version with debug_pos_control (41:45) debug_h_control and debug_waypoints (35:39)
%2 - firmware version with debug_pos (39:46) debug_h_control and debug_waypoints (14:19)
%3 - firmware version with debug_position(39:46) debug_h(14:19)
firmware_version = 3;
%
% and process
if firmware_version == 1,
    %demux debug_pos_control 
    DEBUG_POS_CONTROL_x = DATA(:,41:45);
    DEBUG_POS_CONTROL_x(y_idx,:) = NaN;
    DEBUG_POS_CONTROL_y = DATA(:,41:45);
    DEBUG_POS_CONTROL_y(x_idx,:) = NaN;
    %replace NANs
    for i = 1:length(T),
        if isnan(DEBUG_POS_CONTROL_x(i,1)), if i>1, DEBUG_POS_CONTROL_x(i,:) = DEBUG_POS_CONTROL_x(i-1,:); else DEBUG_POS_CONTROL_x(i,:)=0; end; end;
        if isnan(DEBUG_POS_CONTROL_y(i,1)), if i>1, DEBUG_POS_CONTROL_y(i,:) = DEBUG_POS_CONTROL_y(i-1,:); else DEBUG_POS_CONTROL_y(i,:)=0; end; end;
    end    
    PILOT_COMMANDS = DATA(:,35:38);
    MODE = DATA(:,39);
elseif firmware_version == 2,
    %demux debug_pos_control 
    DEBUG_POS_CONTROL_x = DATA(:,41:45); DEBUG_POS_CONTROL_x(y_idx,:) = NaN;
    DEBUG_POS_CONTROL_y = DATA(:,41:45); DEBUG_POS_CONTROL_y(x_idx,:) = NaN;
    %demux pilot commands, mode and debug_waypoints
    PILOT_COMMANDS = DATA(:,35:38); PILOT_COMMANDS(x_idx,:) = NaN; 
    MODE = DATA(:,39); MODE(x_idx) = NaN;
    DEBUG_WAYP = DATA(:,35:39); DEBUG_WAYP(y_idx,:) = NaN;
    %replace NANs
    for i = 1:length(T),
        if isnan(DEBUG_POS_CONTROL_x(i,1)), if i>1, DEBUG_POS_CONTROL_x(i,:) = DEBUG_POS_CONTROL_x(i-1,:); else DEBUG_POS_CONTROL_x(i,:)=0; end; end;
        if isnan(DEBUG_POS_CONTROL_y(i,1)), if i>1, DEBUG_POS_CONTROL_y(i,:) = DEBUG_POS_CONTROL_y(i-1,:); else DEBUG_POS_CONTROL_y(i,:)=0; end; end;
        if isnan(PILOT_COMMANDS(i,1)), if i>1, PILOT_COMMANDS(i,:) = PILOT_COMMANDS(i-1,:); else PILOT_COMMANDS(i,:)=0; end; end;
        if isnan(MODE(i,1)), if i>1, MODE(i,:) = MODE(i-1,:); else MODE(i,:)=0; end; end;
        if isnan(DEBUG_WAYP(i,1)), if i>1, DEBUG_WAYP(i,:) = DEBUG_WAYP(i-1,:); else DEBUG_WAYP(i,:)=0; end; end;
    end    

elseif firmware_version == 3,
    %demux debug_position
    % extract the first 8 signals
    DEBUG_POS_UP = DATA(:,39:46); DEBUG_POS_UP(x_idx,:) = NaN;
    % extract the second 8 signals
    DEBUG_POS_DOWN = DATA(:,39:46); DEBUG_POS_DOWN(y_idx,:) = NaN;
    %demux pilot rc commands, altitude
    DEBUG_RC_CONTROLS = DATA(:,14:19); DEBUG_RC_CONTROLS(x_idx,:) = NaN;
    DEBUG_ALTITUDE = DATA(:,14:19); DEBUG_ALTITUDE(y_idx,:) = NaN; 
   
    %replace NANs
    for i = 1:length(T),
        if isnan(DEBUG_POS_UP(i,1)), if i>1, DEBUG_POS_UP(i,:) = DEBUG_POS_UP(i-1,:); else DEBUG_POS_UP(i,:)=0; end; end;
        if isnan(DEBUG_POS_DOWN(i,1)), if i>1, DEBUG_POS_DOWN(i,:) = DEBUG_POS_DOWN(i-1,:); else DEBUG_POS_DOWN(i,:)=0; end; end;
        if isnan(DEBUG_RC_CONTROLS(i,1)), if i>1, DEBUG_RC_CONTROLS(i,:) = DEBUG_RC_CONTROLS(i-1,:); else DEBUG_RC_CONTROLS(i,:)=0; end; end;
        if isnan(DEBUG_ALTITUDE(i,1)), if i>1, DEBUG_ALTITUDE(i,:) = DEBUG_ALTITUDE(i-1,:); else DEBUG_ALTITUDE(i,:)=0; end; end;
    end
    
    MODE = DEBUG_RC_CONTROLS(:,5);
end

%% plot time data firmware 3
%
% here a+we assume to have T and DATA 
% DATA = DATA_rerun(1:length(T),:);

figure, plot(T, DATA(:,2:4).*180/pi), legend('roll','pitch','yaw'),grid on;


%inertial data
figure, plot(T, DATA(:,5:7).*180/pi), legend('gyro x','gyro y','gyro z'),grid on;
figure, plot(T, DATA(:,8:10)), legend('acc x','acc y','acc z'),grid on;
figure, plot(T, DATA(:,11:13)), legend('mag x','mag y','mag z'),grid on;
figure, plot(T, sqrt(DATA(:,11).^2+DATA(:,12).^2+DATA(:,13).^2)),grid on, legend('mag norm');
%for mag calib
if 0,
    figure, plot(DATA(:,11:13));
    seg = 3000:10300;
    clear MNAV;
    MNAV(:,7:9) = double(DATA(seg,11:13));
end

%ultrasound and baro 
figure, plot(T, DEBUG_POS_DOWN(:,2)),grid on, legend('ultrasound');
figure, plot(T, DEBUG_ALTITUDE(:,5)),grid on, legend('h baro filtered');

%navigation data (KF)
figure, plot(T, DATA(:,[1 4 7]+19)),grid on, legend('x','y','z');
figure, plot(T, DATA(:,[1 4 7]+19+1)),grid on, legend('vx','vy','vz');
figure, plot(T, DATA(:,[1 4 7]+19+2)),grid on, legend('ax','ay','az');
figure, plot(T, DATA(:,38)),grid on, legend('KF flags');
%compute gps delta T
tgps = T(find(DATA(:,38)==111 | DATA(:,38)==101));
figure, plot(diff(tgps)),grid on;
disp(['Average GPS fix frequency = ', num2str(1/mean(diff(tgps))), ' Hz']);

%plot map view
figure, plot(DATA(:,[4]+19),DATA(:,[1]+19),'r'), hold on 
plot(DATA(:,[4]+19+9),DATA(:,[1]+19+9),'b') 
axis([-150 150 -150 150]), grid on, hold off
legend('estim xy','gps xy');

%map view 2D (x=E, y=N) with coloring based on mode
%divide in sections where mode changes
mode_changes_idx = [1; find(abs(diff(MODE))>0.5); length(T)];
colors = {'k','b','r','m','gx-'};% 
figure;
for i = 1:(length(mode_changes_idx)-1),
    idx_seg = (mode_changes_idx(i)+1):(mode_changes_idx(i+1));
    seg_color = MODE(mode_changes_idx(i)+1,1)+1;
    plot(DATA(idx_seg,[4]+19),DATA(idx_seg,[1]+19),colors{seg_color}), hold on 
end;
hold off
axis equal 
grid on

%lmeasurement data 
figure, plot(T, DATA(:,[1 4 7]+19+9)), legend('x','y','z'); axis([0 max(T) -150 150]), grid on;
figure, plot(T, DATA(:,[1 4 7]+19+1+9)),grid on, legend('vx','vy','vz');
figure, plot(T, DATA(:,[1 4 7]+19+2+9)),grid on, legend('ax','ay','az');

%altitude
figure, plot(T, DEBUG_ALTITUDE(:,5),'r'),hold on;
plot(T, -DATA(:,[7]+19),'b'), 
plot(T, -DATA(:,[7]+19+9),'m'), 
if exist('T_'),
    plot(T_, -DATA_(:,19+8),'g');
    legend('h feedback','h kalman','h baro','h baro filtered RECOMPUTED');
else
    legend('h feedback','h kalman','h baro');
end
hold off;
grid on;

% kf position filtering
figure, plot(T, DATA(:,[1]+19),'r','linewidth',4), hold on, plot(T, DATA(:,[4]+19),'g','linewidth',4), plot(T, DATA(:,[7]+19),'b','linewidth',4), 
plot(T, DATA(:,[1]+19+9),'y','linewidth',2), plot(T, DATA(:,[4]+19+9),'m','linewidth',2), plot(T, DATA(:,[7]+19+9),'c','linewidth',2), 
axis([0 max(T) -150 150]), grid on, hold off
legend('estim x','estim y','estim z','gps x','gps y', 'gps z');


%ultrasound and baro 
figure, plot(T, DEBUG_POS_DOWN(:,2))
hold on
plot(T, DEBUG_ALTITUDE(:,5),'r'),
hold on
plot(T, DEBUG_ALTITUDE(:,4),'g')
hold on
plot(T, -DATA(:,[7]+19),'c'),
hold on
plot(T,DEBUG_RC_CONTROLS(:,5),'k'),legend('ultrasound', 'h feedback' ,'h ref' ,'h kalman','ctrl mode');
grid on
% 

