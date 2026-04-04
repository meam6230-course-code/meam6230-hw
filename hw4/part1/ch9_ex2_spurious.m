%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Aude Billard
%   email:   aude.billard@epfl.ch
%   website: lasa.epfl.ch
%
%                                                                         % 
% Modified by Nadia Figueroa on March 2025, University of Pennsylvania    %
% email: nadiafig@seas.upenn.edu                                          %
%                                                                         %
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('../lib');
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%% TASK 2.1: Construct spurious attractor with obstacle modulation %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization of Grid and Nominal DS
% Create grid to evaluate and visualize DS
clear; close all; clc;
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

% Plot DS
f = figure('Color', [1 1 1], 'Position', [292 547 905 319]); 
subplot(1,2,1); hold on; 
title('Nominal Linear DS', 'Interpreter', 'LaTex', 'FontSize', 15);
limits = [x_limits y_limits];
plot_ds_model_mod(f, ds_lin, target, limits,'low'); 

%% Exercise 2.1 - Local Spurious Attractor 

c = [2.5; 2.5];  % Spurious attractor coordinates
r = 2.5;         % influence radius ( r < |x01-x1|)
p = 4;           % strength of modulation (reactivity) p > 1

for i=1:nb_gridpoints
    for j=1:nb_gridpoints
        x = [X(i,j); Y(i,j)]; % query point
        local_vec = x-c;     % distance to spurious attractor
        [~, ~, M] = spurious_modulation(local_vec, r, p);
        v = apply_modulation(ds_lin(x), M);
        x_dot(i,j) = v(1); y_dot(i,j) = v(2);
    end
end

% Plot DS
subplot(1,2,2); hold on;
plot_speed = 0; %Set to 1 if you want to plot the absolute speed
plot_ds_modulation(X, Y, x_dot, y_dot, c, ...
    'Modulated DS with spurious attractor', 'streamslice', c, plot_speed);
