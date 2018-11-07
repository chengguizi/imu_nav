function naiveIntegrate(accel_world, t, delta_t)
% quat is the rotation of imu frame in the world frame
    persistent velocity position;
    persistent f_naive_v f_naive_p h_vx h_vy h_vz h_pxy h_pz;
    global reset;
    
    
    if (isempty(f_naive_v))
        % Figure for velocity
        f_naive_v = figure();
        h_vx = animatedline('Color','r','MaximumNumPoints',60);
        h_vy = animatedline('Color','g','MaximumNumPoints',60);
        h_vz = animatedline('Color','b','MaximumNumPoints',60);
        axis tight
        axis([-inf,inf,-3,3]);
        title('Naive Method: Velocity');
        legend('vx','vy','vz');
        
        % Figure for position
        f_naive_p = figure();
        subplot(1,2,1);
        h_pxy = animatedline('Color','r','MaximumNumPoints',1000, 'Marker', '+');
        axis equal
        axis([-2,2,-2,2]);
        title('Naive Method: Position XY');
        legend('Position XY');
        
        subplot(1,2,2);
        h_pz = animatedline('Color','r','MaximumNumPoints',60);
        axis tight
        axis([-inf,inf,-10,10]);
        title('Naive Method: Position Z');
        legend('Position Z');
    end
    
    if (isempty(velocity) || isempty(position) || reset)
        velocity = zeros(3,1);
        position = zeros(3,1);
        clearpoints(h_pxy);
        clearpoints(h_pz);
    end
    
    
    
    position = position + velocity*delta_t + 0.5*accel_world*delta_t*delta_t;
    velocity = velocity*0.99 + accel_world*delta_t;
    
    addpoints(h_vx, t, velocity(1));
    addpoints(h_vy, t, velocity(2));
    addpoints(h_vz, t, velocity(3));
    
    addpoints(h_pxy, position(1), position(2));
    addpoints(h_pz, t, position(3));
end