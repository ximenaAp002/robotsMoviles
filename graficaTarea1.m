clear all
close all
clc

%% ChatGPT prompt:
% Create a program to simulate a differential robot in matlab

% Define robot parameters
wheel_radius = 0.05; % radius of wheels (m)
wheelbase = 0.2; % distance between wheels (m) (l)
dt = 0.01; % time step (s)
v = 1; % maximum linear velocity (m/s)
kpr = 1;
kpt = 1;

% Define desired positions
positions = [1 1; 10 10; 5 15]; % x, y coordinates
num_positions = size(positions,1);

% Initialize robot state
x = 0; % x-position (m)
y = 0; % y-position (m)
theta = 0; % orientation (rad)

i = 1;
j = 1;
% Simulate robot motion
while j <= num_positions

    % Calculate desired position and orientation
    xd = positions(j,1);
    yd = positions(j,2);
    thetad = atan2((yd-y),(xd-x));
    d = sqrt((xd-x)^2 + (yd-y)^2);

    % Calculate control signals
    thetae = (theta-thetad);
    if thetae > pi
        thetae = thetae - 2*pi;
    elseif thetae < -pi
        thetae = thetae + 2*pi;
    end

    w = -kpr*thetae;
    v = kpt*d;
    
    vr = v + (wheelbase*w)/2; %% Esto iría al PWM
    vl = v - (wheelbase*w)/2; %% Esto iría al PWM

    % Simulate robot motion
    v = (vr+vl)/2;
    w = (vr-vl)/wheelbase;
    % Compute robot motion
    vx = v*cos(theta);
    vy = v*sin(theta);
    % We integrate wrt time
    x = x + vx*dt;
    y = y + vy*dt;
    theta = theta + w*dt;
    
    % Plot robot motion
    plot(x,y,'o')
    axis([-1 20 -1 20])
    drawnow

    % Check if robot has reached the current reference position
    if d < 0.1
        j = j+1;
        pause(1); % wait for 1 second before moving to the next reference
    end

    Thetae(i) = thetae;
    i = i+1;
end