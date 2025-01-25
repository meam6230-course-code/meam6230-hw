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
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Question 1: Initial trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
filepath = fileparts(which('chp1_ex2.m'));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-robot-simulation')));
addpath(genpath(fullfile(filepath, 'lib')));


% Create robot and optimal control
robot = RobotisWrapper();
optimal_control = MPC4DOF(robot);

% define target position, maximal time and cost function for the solver
initial_joint_configuration = [0; 0; 0; 0];
target_position = [0.1; -0.3; 0.1];
max_time = 3;

%% Cost functions
%% ------ Write your code in minimumJointDistance.m for Question 1 ------
%  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %%
optimal_control.nlSolver.Optimization.CustomCostFcn = @minimumJointDistance;

% find the optimal trajectory to the target
optimal_solution_full = optimal_control.solveOptimalTrajectory(target_position, ...
    initial_joint_configuration, max_time);
optimal_control.showResults(optimal_solution_full, target_position, 'Full trajectory');
disp(optimal_solution_full)

disp("Press space to continue..."); pause(); close all;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Question 3a: Add disturbance  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ------ Write your code in addDisturbance.m for Question 2 ------
%  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %%

[disturbance_idx, q_mid] = addDisturbance(optimal_solution_full, 2, 10);
disp(disturbance_idx)

%  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ %%
%% ------ Write your code above ------

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Question 3b: Generate complete trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Reallocate rest of time to still meet deadline
time_left = max_time - optimal_solution_full.Topt(disturbance_idx);

% Compute new trajectory starting from midpath joint state by providing
% joint state midpath as initial configuration for the solver
fprintf("--------- Reached intermediate target in %1.1f seconds --------- \n ", optimal_solution_full.Topt(disturbance_idx));
optimal_solution_after_disturbance = optimal_control.solveOptimalTrajectory(target_position, q_mid, time_left);

% Stitch trajectories together
%% ------ Write your code in generateTraj.m for Question 3 ------
%  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %%

optimal_solution_final = generateTraj(disturbance_idx, optimal_solution_full, optimal_solution_after_disturbance);

%  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ %%
%% ------ Write your code above ------

% Plot complete trajectory
f = figure('Name', 'Full trajectory with disturbance'); 
robot.animateTrajectory(optimal_solution_final.Xopt(:, 1:disturbance_idx), ...
    optimal_solution_final.Yopt(:, 1:disturbance_idx), target_position, 'Full trajectory with disturbance', f);
pause(0.3);
plot3([optimal_solution_full.Yopt(1, disturbance_idx) optimal_solution_after_disturbance.Yopt(1,1)], ...
      [optimal_solution_full.Yopt(2, disturbance_idx) optimal_solution_after_disturbance.Yopt(2,1)], ...
      [optimal_solution_full.Yopt(3, disturbance_idx) optimal_solution_after_disturbance.Yopt(3,1)], ...
      'color', 'r', 'LineWidth', 4);
pause(0.3);
robot.animateTrajectory(optimal_solution_final.Xopt(:, disturbance_idx:end), ...
    optimal_solution_final.Yopt(:, disturbance_idx:end), target_position, 'Full trajectory with disturbance', f);


