% Clear all variables, console, and plot objects
clear;
clc;
cla;

% Define SCARA robot links
L1 = Link([0 0 8 0 0]);  % Link 1: revolute, length 8
L2 = Link([0 0 10 0 0]); % Link 2: revolute, length 10
L3 = Link([0 0 0 0 1]);  % Link 3: prismatic (along Z, not used here)

% Create SCARA robot
global R;  % Declare R as a global variable
R = SerialLink([L1, L2, L3], 'name', 'SCARA / Robot Dynamics / gabrielhuav');

% Workspace for visualization
global workSpaceLimits;  % Declare workSpaceLimits as a global variable
workSpaceLimits = [-20 20 -20 20 -20 20];

% Initial robot position
global h;  % Declare h as a global variable
if isempty(h) || ~isvalid(h)  % If the figure doesn't exist or isn't valid
    h = figure;  % Create a new figure and store its handle
end
figure(h);  % Set the robot figure as the current figure
R.plot([6, 8, 0],'workspace', workSpaceLimits);


% Link lengths, used in inverse kinematics
l1 = 8;  % Length of link 1
l2 = 10; % Length of link 2

tf = 20;    % Final time
ts = 0.1;   % Time step
t = 0:ts:tf;    % Time vector
N = length(t); % Number of points in the time vector
%G %%
hx = zeros(1,N+1);  % End effector x-coordinate vector
hy = zeros(1,N+1);  % End effector y-coordinate vector
hz = zeros(1,N+1);  % End effector z-coordinate vector

M = round(N);  % Maximum limit for the first segment of the drawing

%% Calculate joint positions / First segment
for k=1:M
    hx(k) = 15*cosd(k);  % End effector x-coordinate -4 + 15*cos(k)
    hy(k) = 15*sind(k);  % End effector y-coordinate 7 + 15*sin(k)
    hz(k) = 0;
    %pause(0.01); % Short pause for animation visualization
    A = hx(k);
    B = hy(k);
end

% Plot the trajectory
hold on;
for i = 1:N
    i;
    % Use 'inverse_kinematics' function to convert XY points to joint angles
    [q1, q2] = inverse_kinematics(hx(i), hy(i), l1, l2);
    % plot3(hx, hy, hz, 'r.');
    p = [hx(i), hy(i), hz(i)]; % Get the current point in the trajectory
    plot_sphere(p, 0.2, 'g'); % Plot a sphere at the current point to visualize the trajectory

    R.plot([q1 q2 5],'workspace', workSpaceLimits); % Third value corresponds to d3 which is zero
    drawnow;

end

% *Line from left to right (green color)*
hx_der = zeros(1,N+1);  % End effector x-coordinate vector (right to left)
hy_der = zeros(1,N+1);  % End effector y-coordinate vector
hz_der = zeros(1,N+1);  % End effector z-coordinate vector

for i = 1:N
    hx_der(i) = -16 + (14/N) * i;  % End effector x-coordinate (right to left)
    hy_der(i) = -5.5;% Constant y-coordinate for a straight line
    hz_der(i) = 0;
end

% Plot the trajectory (right to left)
hold on;
for i = 1:(N/1.2)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2);
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Get the current point in the trajectory
    plot_sphere(p_der, 0.2, 'g'); % Plot a sphere at the current point to visualize the trajectory (green color)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % Green color
    drawnow;
end

% *Line from bottom to top (green color)*
hx_der = zeros(1,N+1);  % End effector x-coordinate vector
hy_der = zeros(1,N+1);  % End effector y-coordinate vector
hz_der = zeros(1,N+1);  % End effector z-coordinate vector

for i = 1:N
    hx_der(i) = -4;  % End effector x-coordinate
    hy_der(i) = -5.5 + (14/N) * i;% y-coordinate
    hz_der(i) = 0;
end

% Plot the trajectory (bottom to top)
hold on;
for i = 1:N
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2);
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Get the current point in the trajectory
    plot_sphere(p_der, 0.2, 'g'); % Plot a sphere at the current point to visualize the trajectory (green color)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % Green color
    drawnow;
end

% H %%
% *Line from left to right (white color)*
hx_izq = zeros(1,N+1);  % End effector x-coordinate vector (left to right)
hy_izq = zeros(1,N+1);  % End effector y-coordinate vector
hz_izq = zeros(1,N+1);  % End effector z-coordinate vector

for i = 1:N
    hx_izq(i) = -3 + (14/N) * (N-i);  % End effector x-coordinate (left to right)
    hy_izq(i) = -7;% Constant y-coordinate for a straight line
    hz_izq(i) = 0;
end

% Plot the trajectory (left to right)
hold on;
for i = 1:(N/2)
    [q1_izq, q2_izq] = inverse_kinematics(hx_izq(i), hy_izq(i), l1, l2);
    p_izq = [hx_izq(i), hy_izq(i), hz_izq(i)]; % Get the current point in the trajectory
    plot_sphere(p_izq, 0.2, 'w'); % Plot a sphere at the current point to visualize the trajectory (white color)
    R.plot([q1_izq q2_izq 5],'workspace', workSpaceLimits); % White color
    drawnow;
end

% *Line from bottom to top (white color)*
hx_der = zeros(1,N+1);  % End effector x-coordinate vector
hy_der = zeros(1,N+1);  % End effector y-coordinate vector
hz_der = zeros(1,N+1);  % End effector z-coordinate vector

for i = 1:N
    hx_der(i) = 8;  % End effector x-coordinate
    hy_der(i) = -6.5 - (14/N) * i;% y-coordinate
    hz_der(i) = 0;
end

% Plot the trajectory (bottom to top)
hold on;
for i = 1:(N/3)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2);
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Get the current point in the trajectory
    plot_sphere(p_der, 0.2, 'w'); % Plot a sphere at the current point to visualize the trajectory (white color)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % White color
    drawnow;
end

% *Line from right to left (white color)*
hx_der = zeros(1,N+1);  % End effector x-coordinate vector (right to left)
hy_der = zeros(1,N+1);  % End effector y-coordinate vector
hz_der = zeros(1,N+1);  % End effector z-coordinate vector

for i = 1:N
    hx_der(i) = -3 + (14/N) * (N-i);  % End effector x-coordinate (right to left)
    hy_der(i) = -11.2;% Constant y-coordinate for a straight line
    hz_der(i) = 0;
end

% Plot the trajectory (right to left)
hold on;
for i = 1:(N/2)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2);
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Get the current point in the trajectory
    plot_sphere(p_der, 0.2, 'w'); % Plot a sphere at the current point to visualize the trajectory (white color)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % White color
    drawnow;
end

% A %%
% *Line from right to left downward diagonal (red color)*
hx_der = zeros(1,N+1);  % End effector x-coordinate vector (right to left)
hy_der = zeros(1,N+1);  % End effector y-coordinate vector
hz_der = zeros(1,N+1);  % End effector z-coordinate vector

for i = 1:N
    hx_der(i) = -6 + (25/N) * i;  % End effector x-coordinate (right to left)
    hy_der(i) = -9 - (11/N) * i;% Constant y-coordinate for a straight line (-8)
    hz_der(i) = 0;
end

% Plot the trajectory (right to left)
hold on;
for i = 1:(N/4)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2);
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Get the current point in the trajectory
    plot_sphere(p_der, 0.2, 'r'); % Plot a sphere at the current point to visualize the trajectory (red color)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % Red color
    drawnow;
end

% *Line from right to left upward diagonal (red color)*
hx_der = zeros(1,N+1);  % End effector x-coordinate vector (right to left)
hy_der = zeros(1,N+1);  % End effector y-coordinate vector
hz_der = zeros(1,N+1);  % End effector z-coordinate vector

for i = 1:N
    hx_der(i) = -6 + (25/N) * i;  % End effector x-coordinate (right to left)
    hy_der(i) = -14.6 + (11/N) * i;% Constant y-coordinate for a straight line (-8)
    hz_der(i) = 0;
end

% Plot the trajectory (right to left)
hold on;
for i = 1:(N/4)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2);
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Get the current point in the trajectory
    plot_sphere(p_der, 0.2, 'r'); % Plot a sphere at the current point to visualize the trajectory (red color)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % Red color
    drawnow;
end

% *Line from bottom to top (red color)*
hx_der = zeros(1,N+1);  % End effector x-coordinate vector
hy_der = zeros(1,N+1);  % End effector y-coordinate vector
hz_der = zeros(1,N+1);  % End effector z-coordinate vector

for i = 1:N
    hx_der(i) = -3;  % End effector x-coordinate
    hy_der(i) = -10 - (14/N) * i;% y-coordinate
    hz_der(i) = 0;
end

% Plot the trajectory (bottom to top)
hold on;
for i = 1:(N/3.8)
    [q1_der, q2_der] = inverse_kinematics(hx_der(i), hy_der(i), l1, l2);
    p_der = [hx_der(i), hy_der(i), hz_der(i)]; % Get the current point in the trajectory
    plot_sphere(p_der, 0.2, 'r'); % Plot a sphere at the current point to visualize the trajectory (red color)
    R.plot([q1_der q2_der 5],'workspace', workSpaceLimits); % Red color
    drawnow;
end

hold off;

% Create text fields for q1 and q2
global q1_text q2_text;
q1_text = uicontrol('Style', 'edit', 'Position', [20 60 100 20], 'String', '0');
q2_text = uicontrol('Style', 'edit', 'Position', [20 30 100 20], 'String', '0');

% Create a button to update the robot's position
update_button = uicontrol('Style', 'pushbutton', 'Position', [20 90 100 20], 'String', 'Update');

% Set the callback function for the button
set(update_button, 'Callback', @updateRobot);

% Callback function to update the robot's position
function updateRobot(src, event)
    % Get the text fields from the figure
    global q1_text q2_text R workSpaceLimits h; % Add workSpaceLimits and h to the global variables list
    q1 = str2double(get(q1_text, 'String')) * (pi / 180);
    q2 = str2double(get(q2_text, 'String')) * (pi / 180);
    z = 0.1;

    % Delete the previous robot
    robot_graphics = findobj(h, 'Type', 'Patch'); % Find the graphical objects that represent the robot
    delete(robot_graphics); % Delete only the robot, not the whole figure

    % Draw the robot with the new values of q1 and q2
    R.plot([q1, q2, z],'workspace', workSpaceLimits);
end

% Inverse kinematics function
function [q1, q2] = inverse_kinematics(x, y, L1, L2)
    % Calculate theta2
    c2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);

    if c2 < -1 || c2 > 1
        error('The point [%f, %f] is out of reach of the robot.', x, y);
    end
    theta2 = acos(c2); % Primary solution

    % Calculate theta1
    k1 = L1 + L2 * cos(theta2);
    k2 = L2 * sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);

    % Assign results to output variables
    q1 = theta1;
    q2 = theta2;
end
