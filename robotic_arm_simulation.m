%%Robotic Arm Simulation%%
%% Clear all variables and output
clear all;close all;clc
%% Load robot
robot = importrobot("sawyer.urdf");
%"sawyer" is a model of robotic arm with 7 dof (degree of freedom)
fig = show(robot);% This shows the robotic arm in visual
fig.CameraPositionMode = 'auto';% the camera position to visualize the arm is automated

disp(robot.BodyNames)

%% Define wayPoints
% (RANDOM)
%wayPoints = [0.2 0 0;0 -0.1 0.5;-0.5 0 0;0 0.1 -0.5;0.2 0 0] + [0 0.5 0.75];
% the waypoints are stored in the form of matrix/list with 3d coordinates.
%wayPointsVel = [0 -1/sqrt(5) 2/sqrt(5);0 -1 0;0 1/sqrt(5) -2/sqrt(5);0 1 0;0 0 0]*0.5;
% the velocity at waypoints are stored in the form of matrix/list.

% (PENTAGON)
wayPoints = [0.5000    0.0000         0;0.5000    0.3804    0.2764; 0.5000    0.2351    0.7236;0.5000   -0.2351    0.7236;0.5000   -0.3804    0.2764;0.5000   -0.0000         0];
% the waypoints are stored in the form of matrix/list with 3d coordinates.
wayPointsVel = [0    0.1000    0.0000;0    0.0309    0.0951;0   -0.0809    0.0588;0   -0.0809   -0.0588;0    0.0309   -0.0951;0    0.1000   -0.0000]*0.5;
% the velocity at waypoints are stored in the form of matrix/list.

% (square in 3d)
%wayPoints = [0    0.0000         0;0    1 1;1.414    1    1;1.414   0    0;0 0 0]*0.6+[-0.5 0 0.2];
% the waypoints are stored in the form of matrix/list with 3d coordinates.
%wayPointsVel = [0    0.1000    0.1;0.1414   0    0;0   -0.1    -0.1;-0.1414   0    0; 0 0 0]*0.5;
% the velocity at waypoints are stored in the form of matrix/list.

%% Make trajectory
numTotalPoints = size(wayPoints,1)*10;%if X is a matrix of order [m,n] then size(X,1) returns m.
waypointTime = 4;

% For trapezoidal interpolation of velocity
%[trajectory, trajectory_vel, trajectory_acc] = trapveltraj(wayPoints',numTotalPoints,'EndTime',waypointTime);

% Cubic interpolation
wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
%m:n it returns a list of integers (if n>=m) starting from m and ends at n with an increment of 1 otherwise returns empty.

trajTimes = linspace(0,wpTimes(end),numTotalPoints);
%linspace(start, end, increment)
[trajectory, trajectory_vel, trajectory_acc] = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
                    'VelocityBoundaryCondition',wayPointsVel');
%Generate third-order polynomial trajectories through multiple waypoints.

hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);
%3d plot with plot(x,y,z) and indicates red line.

%% Compute inverse kinematics
ik = robotics.InverseKinematics('RigidBodyTree', robot);
% Import Iverse Kinematic model for the this arm

weights = [0.1 0.1 0.1 1 1 1];
% These weights are given input to solver, [x, y, z, vx, vy, vz]. Since we
% may not always find a solution for links/joints. Having weights will give
% us approximate solution.

currPosition = robot.homeConfiguration;
% Define current position as Initial position of robot

for idx = 1:size(trajectory,2) % Loop through all the trajectory points
    tform = trvec2tform(trajectory(:,idx)');
    % Convert the translational vector to homogeneous transformation
    configSoln(idx,:) = ik('right_l6',tform,weights,currPosition);
    % Find the solution for all joints/links using inverse kinematic model
    currPosition = configSoln(idx,:);
    % Update the current position to latest rendered solution
end

%% Visualise the motion
title('Robot Arm waypoint tracking visualization')
for idx = 1:size(trajectory,2)
    show(robot,configSoln(idx,:), 'PreservePlot', false,'Frames','off');
    % Show the robot configuration for each time step
    pause(0.1)
    % Pause for 0.1
end
hold off
%% Profiles for cubic interpolation interpolation
% Compute the trajectories to avoid overwrites
wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
trajTimes = linspace(0,wpTimes(end),numTotalPoints);
[trajectory, trajectory_vel, trajectory_acc] = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
                    'VelocityBoundaryCondition',wayPointsVel');
figure;
% Plotting X, Y, Z coordinates vs Time
subplot(3, 1, 1);
plot(trajectory(1, :))
hold on
plot(trajectory(2, :))
plot(trajectory(3, :))
title("Coordinates")
legend("x", "y", "z")
grid()

% Plotting X, Y, Z Velocities vs Time
subplot(3, 1, 2);
plot(trajectory_vel(1, :))
hold on
plot(trajectory_vel(2, :))
plot(trajectory_vel(3, :))
title("Velocity profiles")
legend("Vx", "Vy", "Vz")
grid()

% Plotting X, Y, Z Accelerations vs Time
subplot(3, 1, 3)
plot(trajectory_acc(1, :))
hold on
plot(trajectory_acc(2, :))
plot(trajectory_acc(3, :))
title("Acceleration profiles")
legend("Ax", "Ay", "Az")
grid()

sgtitle('Cubic Interpolation')
%% Profiles for trapezoidal interpolation
% Compute the trajectories to avoid overwrites
[trajectory, trajectory_vel, trajectory_acc] = trapveltraj(wayPoints',numTotalPoints,'EndTime',waypointTime);

figure;
% Plotting X, Y, Z coordinates vs Time
subplot(3, 1, 1);
plot(trajectory(1, :))
hold on
plot(trajectory(2, :))
plot(trajectory(3, :))
title("Coordinates")
legend("x", "y", "z")
grid()

% Plotting X, Y, Z Velocities vs Time
subplot(3, 1, 2);
plot(trajectory_vel(1, :))
hold on
plot(trajectory_vel(2, :))
plot(trajectory_vel(3, :))
title("Velocity profiles")
legend("Vx", "Vy", "Vz")
grid()

% Plotting X, Y, Z Accelerations vs Time
subplot(3, 1, 3)
plot(trajectory_acc(1, :))
hold on
plot(trajectory_acc(2, :))
plot(trajectory_acc(3, :))
title("Acceleration profiles")
legend("Ax", "Ay", "Az")
grid()

sgtitle('Trapezoidal Interpolation')