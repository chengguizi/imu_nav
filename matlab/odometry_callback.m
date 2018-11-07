%%% Cheng Huimin
%%% Oct 2018

function odometry_callback(src, msg)
    global time_offset;
    global h_vx h_vy h_vz h_pxy h_pz;
    global reset;
    
    time = msg.Header.Stamp.Sec + msg.Header.Stamp.Nsec / 1.0e9;
    if(isempty(time_offset))
        time_offset = time;
    end
    t = time - time_offset;
    
    if (reset)
        clearpoints(h_pxy);
        clearpoints(h_pz);
    end
    
    addpoints(h_vx,t,msg.Twist.Twist.Linear.X);
    addpoints(h_vy,t,msg.Twist.Twist.Linear.Y);
    addpoints(h_vz,t,msg.Twist.Twist.Linear.Z);
    
    addpoints(h_pxy,msg.Pose.Pose.Position.X, msg.Pose.Pose.Position.Y);
    addpoints(h_pz,t,msg.Pose.Pose.Position.Z);
    drawnow limitrate
end