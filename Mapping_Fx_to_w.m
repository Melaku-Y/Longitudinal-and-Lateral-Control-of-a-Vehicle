function w = Mapping_Fx_to_w(Fx, Vx)
    % MAPPING_FX_TO_W Converts longitudinal force to wheel angular velocity
    % Inputs:
    %   Fx: Longitudinal force (N)
    %   Vx: Longitudinal velocity (m/s)
    % Output:
    %   w: Wheel angular velocity (rad/s)
    
    %% Tire Parameters
    C_lambda = 66100;  % Longitudinal slip stiffness (N)
    R = 0.3;          % Wheel radius (m)
    
    %% Force to Angular Velocity Conversion
    if Fx >= 0
        % Driving condition (positive force)
        w = Vx/(R*(1-Fx/C_lambda));
    else
        % Braking condition (negative force)
        w = Vx*(Fx/C_lambda+1)/R;
    end
end