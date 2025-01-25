function Ref = AdaptivepurePursuitRefGen(Xgl, Ygl, YawHead, Vx, Beta, tp, Track)
    % ADAPTIVEPUREPURSUITREFGEN Implements Adaptive Pure Pursuit path following
    % Inputs:
    %   Xgl, Ygl: Current vehicle global position (m)
    %   YawHead: Vehicle heading angle (rad)
    %   Vx: Longitudinal velocity (m/s)
    %   Beta: Vehicle sideslip angle (rad)
    %   tp: Preview time (s)
    %   Track: Structure containing track data
    % Output:
    %   Ref: [yaw_rate_reference, velocity_reference, lateral_error]
    
    %% Initialize Path Data
    Xpath = Track.X;  % Track X-coordinates
    Ypath = Track.Y;  % Track Y-coordinates
    
    %% Safety Checks and Velocity Calculations
    % Minimum velocity threshold
    if abs(Vx) < 0.05
        Vx = 0.05;
    end
    
    % Calculate vehicle velocities
    Vy = Vx*Beta;                % Lateral velocity (m/s)
    V = sqrt(Vx^2 + Vy^2);       % Total velocity magnitude (m/s)
    L = tp*V;                    % Look-ahead distance (m)
    
    %% Path Following
    global pathindex
    
    % Find closest point on path
    pathindex = findMinDist(Xgl, Ygl, Xpath, Ypath, pathindex);
    
    %% Velocity Reference Generation
    kph2ms = 1/3.6;  % Convert km/h to m/s
    
    % Determine reference velocity based on track position
    Vxref = getVelocityReference(pathindex, kph2ms);
    
    %% Pure Pursuit Calculations
    % Calculate look-ahead point
    XL = Xgl + L*cos(YawHead + Beta);  % Look-ahead X position
    YL = Ygl + L*sin(YawHead + Beta);  % Look-ahead Y position
    
    % Find pursuit point on path
    pursuitindex = findMinDist(XL, YL, Xpath, Ypath, pathindex);
    Xpursuit = Xpath(pursuitindex);
    Ypursuit = Ypath(pursuitindex);
    
    % Calculate steering geometry
    alpha = atan2(Ypursuit - Ygl, Xpursuit - Xgl) - YawHead;
    k = 2*sin(alpha)/norm([Xgl,Ygl] - [Xpursuit,Ypursuit]);  % Curvature
    Yawrate_Ref = k*Vx;  % Reference yaw rate
    
    % Calculate lateral tracking error
    sgn = sign(sin(alpha));
    ye = sgn*norm([XL,YL] - [Xpursuit,Ypursuit]);
    
    % Compile output
    Ref = [Yawrate_Ref, Vxref, ye];
end

function index = findMinDist(X, Y, Xpath, Ypath, index)
    % FINDMINDIST Finds the closest point on path using iterative search
    % Inputs:
    %   X, Y: Query point coordinates
    %   Xpath, Ypath: Path coordinates
    %   index: Starting search index
    % Output:
    %   index: Index of closest point
    
    DO = true;
    while DO
        di = norm([X,Y] - [Xpath(index),Ypath(index)]);
        index = index + 1;
        dnew = norm([X,Y] - [Xpath(index),Ypath(index)]);
        DO = di > dnew;
    end
    index = index - 1;
end

function Vxref = getVelocityReference(pathindex, kph2ms)
    % GETVELOCITYREFERENCE Determines reference velocity based on track position
    % Inputs:
    %   pathindex: Current position along track
    %   kph2ms: Conversion factor from km/h to m/s
    % Output:
    %   Vxref: Reference velocity (m/s)
    
    if pathindex <= 22000
        Vxref = 120 * kph2ms;
    elseif pathindex <= 77000
        Vxref = 50 * kph2ms;
    elseif pathindex <= 84000
        Vxref = 65 * kph2ms;
    elseif pathindex <= 98000
        Vxref = 50 * kph2ms;
    elseif pathindex <= 107000
        Vxref = 65 * kph2ms;
    elseif pathindex <= 113000
        Vxref = 50 * kph2ms;
    elseif pathindex <= 139600
        Vxref = 65 * kph2ms;
    elseif pathindex <= 140700
        Vxref = 50 * kph2ms;
    elseif pathindex <= 150000
        Vxref = 70 * kph2ms;
    elseif pathindex <= 160000
        Vxref = 50 * kph2ms;
    elseif pathindex <= 195000
        Vxref = 100 * kph2ms;
    elseif pathindex <= 216100
        Vxref = 50 * kph2ms;
    elseif pathindex <= 268500
        Vxref = 65 * kph2ms;
    elseif pathindex <= 272100
        Vxref = 50 * kph2ms;
    else
        Vxref = 180 * kph2ms;
    end
end