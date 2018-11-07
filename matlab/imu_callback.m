%%% Cheng Huimin
%%% Oct 2018


function imu_callback(src, msg)

    persistent pre_time;
    
    global time_offset;
    global h_imu_ax h_imu_ay h_imu_az;
    global h_imu_gx h_imu_gy h_imu_gz;
%     global reset;
    
    time = msg.Header.Stamp.Sec + msg.Header.Stamp.Nsec / 1.0e9;

    if(isempty(time_offset))
        time_offset = time;
    end
    
    if(isempty(pre_time))
        pre_time = time;
    end
    
    % relative time
    t = time - time_offset;
    
    addpoints(h_imu_ax,t,msg.LinearAcceleration.X);
    addpoints(h_imu_ay,t,msg.LinearAcceleration.Y);
    addpoints(h_imu_az,t,msg.LinearAcceleration.Z);
    
%     addpoints(h_imu_gx,t,msg.AngularVelocity.X);
%     addpoints(h_imu_gy,t,msg.AngularVelocity.Y);
%     addpoints(h_imu_gz,t,msg.AngularVelocity.Z);
%     
%     accel_world = [ msg.LinearAcceleration.X msg.LinearAcceleration.Y msg.LinearAcceleration.Z]';
%     
%     naiveIntegrate(accel_world, t, time - pre_time);
    
    drawnow limitrate
    
    
    pre_time = time;
end