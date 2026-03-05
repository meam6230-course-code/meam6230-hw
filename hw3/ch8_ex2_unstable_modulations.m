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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%% TASK 2.3: Evaluate Desired Unstable Modulations %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization of Grid and Nominal DS
% Create grid to evaluate and visualize DS
% clear; close all; clc;
x_limits = [-5, 5];
y_limits = [-5, 5];
nb_gridpoints = 50;

% mesh domain
[X, Y] = meshgrid(linspace(x_limits(1), x_limits(2), nb_gridpoints), ...
                  linspace(y_limits(1), y_limits(2), nb_gridpoints));

% Construct Nominal Linear DS
A = -eye*(2); 
target = [0; 0];  
ds_lin = @(x) lin_ds(x,target, A);

% Plot DS
f = figure('Color', [1 1 1]); 
screensize = get(groot, 'Screensize'); 
f.Position = [0.05  * screensize(3), 0.1  * screensize(4), 0.6 * screensize(3), 0.8 * screensize(4)];  
subplot(1,3,1); hold on; 
title('Nominal Linear DS', 'Interpreter', 'LaTex', 'FontSize', 15);
limits = [x_limits y_limits];
plot_ds_model_mod(f, ds_lin, target, limits,'medium'); hold on;
set(f, 'Position', [231  573  1276  288]); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create a stable limit cycle around the attractor while keeping it stable %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% See Lecture 9 Slide 12
x0     = [0; 0]; % Coordinates of modulator
sigma  = 1;  % Kernel width for effect of modulator
radius = 3; % Radius of the limit cycle
theta  = deg2rad(90); % angle of rotation

for i=1:nb_gridpoints
    for j=1:nb_gridpoints
        x = [X(i,j) Y(i,j)]';
        [gamma(i,j), v] = stable_limit_cycle(x, ds_lin([X(i,j) Y(i,j)]'), x0, sigma, radius, theta);
        x_dot(i,j) = v(1); y_dot(i,j) = v(2);
    end
end

% Plot DS
subplot(1,3,2); hold on;
plot_ds_h_modulation(X, Y, x_dot, y_dot, gamma, target, ...
  'Stable Limit Cycle', 'streamslice', x0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create an unstable point while keeping the origin stable %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% See Lecture 9 Slide 15
x0    = [2.5; 1]; % Unstable point coordinate
sigma = 0.5; % Kernel width for effect of modulator

for i=1:nb_gridpoints
    for j=1:nb_gridpoints
        x = [X(i,j) Y(i,j)]';
        [gamma(i,j), v] = unstable_point(x, ds_lin([X(i,j) Y(i,j)]'), x0, sigma);
        x_dot(i,j) = v(1); y_dot(i,j) = v(2); 
    end
end

% Plot DS
subplot(1,3,3); hold on;
plot_ds_h_modulation(X, Y, x_dot, y_dot, gamma, target, ...
  'Saddle Point', 'streamslice', x0);


