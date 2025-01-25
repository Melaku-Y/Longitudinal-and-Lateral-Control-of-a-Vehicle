%% Controller Parameters
% PI Controller
Kp = 1500;   % Proportional gain
Ki = 200;    % Integral gain
ts = 0.1;    % Sample time (s)
Fmax = 9000; % Maximum force (N)
Ka = 1/(Ki*ts/3); % Anti-windup gain

% MPC Parameters
Q = 10;      % State weight matrix
R = 2;       % Control weight matrix
N = 5;       % Prediction horizon

%% Load Required Data
% Run prerequisite scripts
MPC_Script;  % Initialize MPC controller
Montmelo;    % Load track data

%% Initialize Simulation
% Set global variables
global pathindex
pathindex = 1;

% Set initial vehicle states
X0  = 0;         % Initial X position (m)
Y0  = 0;         % Initial Y position (m)
Psi0 = 245*pi/180; % Initial heading angle (rad)
V0  = 20;        % Initial velocity (m/s)


%% Visualization
figure(2);
% Plot reference track
plot(Track.X, Track.Y, 'LineWidth', 1.5, 'Color', 'b');
hold on

% Plot simulated vehicle path
plot(out.Xgl, out.Ygl, 'LineWidth', 1.5, 'Color', 'r');

% Format plot
legend('Reference Path', 'Actual Path', 'Location', 'best');
axis equal
grid on
title('Comparison of Reference vs. Actual Path');
hold off