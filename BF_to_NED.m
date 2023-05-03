%In this script there is the conversion from Body Frame to
%NED(north-east-down) frame.

%Angles of yaw,pitch,roll in degree

yaw = 10 ; 
pitch =10 ; 
roll = 10;  

% Conversion to radians
yaw_rad = deg2rad(yaw);
pitch_rad = deg2rad(pitch);
roll_rad = deg2rad(roll);

% Rotation Matrix
R = [cos(yaw_rad)*cos(pitch_rad) cos(yaw_rad)*sin(pitch_rad)*sin(roll_rad)-sin(yaw_rad)*cos(roll_rad)  cos(yaw_rad)*sin(pitch_rad)*cos(roll_rad)+sin(yaw_rad)*sin(roll_rad);
     sin(yaw_rad)*cos(pitch_rad) sin(yaw_rad)*sin(pitch_rad)*sin(roll_rad)+cos(yaw_rad)*cos(roll_rad)  sin(yaw_rad)*sin(pitch_rad)*cos(roll_rad)-cos(yaw_rad)*sin(roll_rad);
     -sin(pitch_rad) cos(pitch_rad)*sin(roll_rad) cos(pitch_rad)*cos(roll_rad)];

