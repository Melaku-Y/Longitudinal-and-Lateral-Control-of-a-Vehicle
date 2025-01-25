function delta = MPC_Sim(vx, PsiDot, Beta, PsiDotRef, controller)
    % MPC_SIM Simulates vehicle dynamics with MPC control
    % Inputs:
    %   vx: Longitudinal velocity (m/s)
    %   PsiDot: Current yaw rate (rad/s)
    %   Beta: Vehicle sideslip angle (rad)
    %   PsiDotRef: Reference yaw rate (rad/s)
    %   controller: MPC controller object
    % Output:
    %   delta: Steering angle command (rad)

    %% Vehicle Parameters
    % Physical parameters
    m = 1400;    % Vehicle mass (kg)
    Iz = 1960;   % Yaw moment of inertia (kg*m^2)
    lf = 1.1770; % Distance from CG to front axle (m)
    lr = 1.3580; % Distance from CG to rear axle (m)
    Cf = 84085;  % Front cornering stiffness (N/rad)
    Cr = 87342;  % Rear cornering stiffness (N/rad)
    g = 9.8100;  % Gravitational acceleration (m/s^2)

    % Simulation parameter
    Ts = 0.01;   % Sample time (s)

    %% State Space Model
    % Compute continuous-time state matrices for current velocity
    A = [-(lf^2*Cf+lr^2*Cr)/(Iz*vx) (-lf*Cf+lr*Cr)/(Iz);
         -1+(-lf*Cf+lr*Cr)/(m*vx^2) -(Cf+Cr)/(m*vx)];
    B = [lf*Cf/Iz; Cf/(m*vx)];

    % Convert to discrete-time system
    sysd = c2d(ss(A,B,eye(2),0),Ts);
    Ad = sysd.A;  
    Bd = sysd.B; 
    
    %% MPC Control Computation
    % Current state vector
    x0 = [PsiDot; Beta];
    
    % Compute control input using MPC
    u = controller(x0, PsiDotRef, Ad, Bd);
    
    % Extract steering command
    delta = u(1);
end