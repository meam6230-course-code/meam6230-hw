%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Aude Billard
%   email:   aude.billard@epfl.ch
%   website: lasa.epfl.ch
%                                                                         % 
% Modified by Nadia Figueroa on March 2025, University of Pennsylvania    %
% email: nadiafig@seas.upenn.edu                                          %
%                                                                         %
%    
%   Permission is granted to copy, distribute, and/or modify this program
%   under the terms of the GNU General Public License, version 2 or any
%   later version published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful, but
%   WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%   Public License for more details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    MIT Press book 
%    Learning for Adaptive and Reactive Robot Control
%    Chapter 8 -  Dynamical system based compliant control: 
%                 Programming exercises 1 & 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%% TASK 2.2: Evaluate Multiple Locally Rotating Modulations %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization of Grid and Nominal DS
% Create grid to evaluate and visualize DS
% clear; close all; clc;
x_limits = [-5, 5];
y_limits = [-5, 5];
nb_gridpoints = 30;

% mesh domain
[X, Y] = meshgrid(linspace(x_limits(1), x_limits(2), nb_gridpoints), ...
                  linspace(y_limits(1), y_limits(2), nb_gridpoints));

% Construct Nominal Linear DS
A = -eye*(2); 
target = [0; 0];  
ds_lin = @(x) lin_ds(x,target, A);

% Select Multiple Modulation Strategy
multi_strategy = 1; % 0: Product of modulations, 1: Custom partitioned modulations
ls_all = 3.5;

% Plot DS
f = figure('Color', [1 1 1]); 
screensize = get(groot, 'Screensize'); 
f.Position = [0.05  * screensize(3), 0.1  * screensize(4), 0.6 * screensize(3), 0.8 * screensize(4)];  
subplot(2,2,1); hold on; 
title('Nominal Linear DS', 'Interpreter', 'LaTex', 'FontSize', 15);
limits = [x_limits y_limits];
plot_ds_model_mod(f, ds_lin, target, limits,'high'); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Evaluate Two local rotations %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Coordinates of 2 modulators
z1      = [2; 2];
z2      = [-2; -2];
% Length-scales and thetas of 2 modulators
ls_2    = ls_all*ones(1,2);
theta_2 = [deg2rad(100) deg2rad(-100)];

for i=1:nb_gridpoints 
    for j=1:nb_gridpoints
        x = [X(i,j) Y(i,j)]';
        [h_x(i,j), v] = multi_locally_rotate_2d(x, ds_lin(x), theta_2, ls_2, [z1 z2], multi_strategy);
        x_dot(i,j) = v(1); y_dot(i,j) = v(2); 
    end
end

% Plot DS
subplot(2,2,2); hold on;
plot_ds_h_modulation(X, Y, x_dot, y_dot, h_x, target, ...
  'Modulated DS with two rotations', 'streamslice', [z1, z2]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Evaluate Three local rotations %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Coordinates of 3 modulators
z1      = [0; -3];
z2      = [2; 3];
z3      = [-2; 3];

% Length-scales and thetas of 3 modulators
ls_3    = ls_all*ones(1,3);
theta_3 = deg2rad(90)*[2 -1 1];

for i=1:nb_gridpoints 
    for j=1:nb_gridpoints
        x = [X(i,j) Y(i,j)]';
        [h_x(i,j), v] = multi_locally_rotate_2d(x, ds_lin([X(i,j) Y(i,j)]'), theta_3, ls_3, [z1 z2 z3], multi_strategy);
        x_dot(i,j) = v(1); y_dot(i,j) = v(2); 
    end
end

% Plot DS
subplot(2,2,3); hold on; 
plot_ds_h_modulation(X, Y, x_dot, y_dot, h_x, target, ...
    'Modulated DS with 3 rotations', 'streamslice', [z1, z2, z3])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Evaluate Three local rotations %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Coordinates of 4 modulators
z1      = [-2; -3];
z2      = [2; 3];
z3      = [-2; 3];
z4      = [2; -3];
% Length-scales and thetas of 4 modulators
ls_4    = ls_all*ones(1,4);
theta_4 = deg2rad(180)*[1 1 -1 -1]; %[deg]

for i=1:nb_gridpoints 
    for j=1:nb_gridpoints
        x = [X(i,j) Y(i,j)]';
        [h_x(i,j), v] = multi_locally_rotate_2d(x, ds_lin([X(i,j) Y(i,j)]'), theta_4, ls_4, [z1 z2 z3 z4], multi_strategy);
        x_dot(i,j) = v(1); y_dot(i,j) = v(2); 
    end
end

% Plot DS
subplot(2,2,4); hold on; 
plot_ds_h_modulation(X, Y, x_dot, y_dot, h_x, target, ...
    'Modulated DS with 4 rotations', 'streamslice', [z1, z2, z3, z4])
