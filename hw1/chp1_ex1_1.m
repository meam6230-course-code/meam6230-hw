%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exercise Script for  Chapter 1 of:                                      %
% "Robots that can learn and adapt" by Billard, Mirrazavi and Figueroa.   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2020 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Alberic de Lajarte                                             %
% email:   alberic.lajarte@epfl.ch                                        %
% website: http://lasa.epfl.ch                                            %
%                                                                         %
% Permission is granted to copy, distribute, and/or modify this program   %
% under the terms of the GNU General Public License, version 2 or any     %
% later version published by the Free Software Foundation.                %
%                                                                         %
% This program is distributed in the hope that it will be useful, but     %
% WITHOUT ANY WARRANTY; without even the implied warranty of              %
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General%
% Public License for more details                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Task 4: Compute closed-form trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
filepath = fileparts(which('chp1_ex1_1.m'));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-robot-simulation')));
addpath(genpath(fullfile(filepath, 'lib')));

% Define time vector
time = linspace(0, 5, 50);

% Create robot from the custom RobotisWrapper class
robot = RobotisWrapper();

% Define initial position and target position by sampling two points from
% the workspace of the robot. 
% If you want to manually set them, make sure they are in the workspace,
% and use column vector notation.
initialPosition = robot.sampleRandomPosition();
targetPosition = robot.sampleRandomPosition();

% Adding waypoints between initial and target
waypoints = zeros(3, 2);
max_deviation = 0.05;
for i = 1:3 % for each dimension
    traj4 = linspace(initialPosition(i), targetPosition(i), 4);
    waypoints(i, :) = traj4(2:3) + (-max_deviation + 2 * max_deviation * rand(1, 2));
end
%% ------ Write your code below ------
%  vvvvvvvvvvvvvvvvv vvvvvvvvvvvvvvv %%

% Fill the 'trajectory' array with 3D position as column vectors
% The array 'trajectory' should start at 'initialPosition' 
% and end at 'targetPosition'.
cartesianTrajectory = fillTrajectory(time, initialPosition, waypoints, targetPosition);


%  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ %%
%% ------ Write your code above ------

% Plot resulting trajectory
figure;
plot3(cartesianTrajectory(1,:), cartesianTrajectory(2,:), cartesianTrajectory(3,:))
axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Closed-form time-dependent trajectory based on third-order polynomial');
legend('Trajectory');
disp("Press space to continue..."); pause();
close;

% Animate trajectory with robot
jointTrajectory = robot.computeInverseKinematics(cartesianTrajectory);
robot.animateTrajectory(jointTrajectory, cartesianTrajectory, targetPosition, 'Polynomial trajectory');
