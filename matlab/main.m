%%% Cheng Huimin
%%% Oct 2018

clear all;
close all;
% rosshutdown;

clear imu_sub;
clear odometry_sub;
clear reset_pub;

clc;
% rosinit('arena-glass.local','NodeHost','192.168.2.10')

global reset;

if (isempty(reset))
    reset = true;
end

%% Figures for Raw Inputs
global h_imu_ax h_imu_ay h_imu_az;
global h_imu_gx h_imu_gy h_imu_gz;
% global h_mag_x h_mag_y h_mag_z;

f1 = figure();
% Configure accerlation raw plot
% subplot(1,2,1);
h_imu_ax = animatedline('Color','r','MaximumNumPoints',60);
h_imu_ay = animatedline('Color','g','MaximumNumPoints',60);
h_imu_az = animatedline('Color','b','MaximumNumPoints',60);
axis tight
axis([-inf,inf,-4,4]);
title('Accelerometer in World');

grid on
legend('ax','ay','az');

% Configure gyroscope raw plot
% subplot(1,2,2);
% h_imu_gx = animatedline('Color','r','MaximumNumPoints',60);
% h_imu_gy = animatedline('Color','g','MaximumNumPoints',60);
% h_imu_gz = animatedline('Color','b','MaximumNumPoints',60);
% axis tight
% axis([-inf,inf,-10,10]);
% title('Gyroscope Raw Data');
% grid on
% legend('gx','gy','gz');



%% Figures for Integrated Position and Velocity

global h_vx h_vy h_vz h_pxy h_pz;
% Figure for velocity
f2 = figure();
h_vx = animatedline('Color','r','MaximumNumPoints',60);
h_vy = animatedline('Color','g','MaximumNumPoints',60);
h_vz = animatedline('Color','b','MaximumNumPoints',60);
axis tight
axis([-inf,inf,-3,3]);
title('Naive Method: ');
grid on
legend('vx','vy','vz');

% Figure for position
f3 = figure();
subplot(1,2,1);
h_pxy = animatedline('Color','r','MaximumNumPoints',1000, 'Marker', '+');
axis equal
% axis tight
% axis([-10,10,-10,10]);
title('Naive Method: Position XY');
grid on
legend('Position XY');

subplot(1,2,2);
h_pz = animatedline('Color','r','MaximumNumPoints',60);
axis tight
axis([-inf,inf,-10,10]);
title('Naive Method: Position Z');
grid on
legend('Position Z');
        
        
%% Register ROS subscriber callback
imu_sub = rossubscriber('/imu_world', 'sensor_msgs/Imu' ,@imu_callback);
odometry_sub = rossubscriber('/dead_reckoning', 'nav_msgs/Odometry', @odometry_callback);
% mag_sub = rossubscriber('/mag0', 'sensor_msgs/MagneticField', @mag_callback);

reset_pub = rospublisher('/reset','std_msgs/Header');
msg = rosmessage('std_msgs/Header');

%% Waiting
pause(1);
while true
    pause(1);
    reset = false;
    key = input('press return to reset, q and return to quit: ','s');  
    if (~isempty(key))
        break;
    end
    
    msg.Stamp = rostime('now','system');
    send(reset_pub,msg);
    reset = true;
    clearpoints(h_pxy);
    clearpoints(h_pz);
    clearpoints(h_vx);
    clearpoints(h_vy);
    clearpoints(h_vz);
    clearpoints(h_imu_ax);
    clearpoints(h_imu_ay);
    clearpoints(h_imu_az);
end
clear imu_sub;
clear odometry_sub;
clear reset_pub;
% rosshutdown;