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
%  Name of the chapter:  Dynamical system based compliant control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    MIT Press book 
%    Learning for Adaptive and Reactive Robot Control
%    Chapter 9 - Obstacle Avoidance: Programming exercise 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
addpath('../lib');
%% Initialize system
x_limits = [-5, 5];
y_limits = [-5, 5];
nb_gridpoints = 200;

% mesh domain
[x, y] = meshgrid(linspace(x_limits(1), x_limits(2), nb_gridpoints), ...
                  linspace(y_limits(1), y_limits(2), nb_gridpoints));

%% Compute and draw nominal linear DS
A = [-2, 0; ...
     0, -2];  % Linear DS with stable attractor 
x0 = [-1; 1]; % Attractor coordinate
b = -A * x0;  

ds_fun = @(x)(A*x + b);
data = feval(ds_fun, [reshape(x, 1, []); reshape(y, 1, [])]);
x_dot = reshape(data(1,:), nb_gridpoints, nb_gridpoints);
y_dot = reshape(data(2,:), nb_gridpoints, nb_gridpoints);

%% Obstacle DS % <== Set DS for obstacle motion
A_obs = [-1.0, 0; 0, -1.0];  %% Linear DS with stable attractor 
x0_obs = [-1; -3] ;
% Attractor coordinate are at Ax+b=0
b_obs = -A_obs * x0_obs ;  

%% Modulated DS - Reference approach with elliptic obstacle
figure('Color', [1 1 1]); 
hold on; axis equal;
% Exercise 2.1.3 
% Set reference point to ensure flow is not stuck
reference_offset = [0; 0]; % now this act as offset instead of point
dt = 0.2;
object_center = [2; 1];  % Coordinate of center of obstacle
n_iter = 40;
for t = 1:n_iter
    % Construct and plot ellipse
    ellipse_axes = [0.5; 2]; % half lengths of ellipse
    theta = linspace(0, 2*pi);
    x_object = ellipse_axes(1) * cos(theta) + object_center(1);
    y_object = ellipse_axes(2) * sin(theta) + object_center(2);
    reference_point = object_center + reference_offset;

    % Compute obstacle velocity for the modulation
    object_vel = A_obs * object_center + b_obs;

    % Initialize variables
    x_dot_mod = zeros(nb_gridpoints, nb_gridpoints);
    y_dot_mod = zeros(nb_gridpoints, nb_gridpoints);
    for i=1:size(x, 1)
        for j=1:size(x, 2)
    
            % Compute distance function
            [gamma(i,j), gradient_gamma] = gamma_ellipse(x(i,j), y(i,j), object_center, ellipse_axes);
            
            % Compute reference direction
            [~, ~, M] = reference_modulation(x(i,j), y(i,j), reference_point, gamma(i,j), gradient_gamma);
            
            % If we are outside the obstacle:
            if gamma(i,j) >= 1
                % In practice modulation should be done on the relative obstacle velocity
                xd = [x_dot(i,j), y_dot(i,j)]';
                v = apply_modulation(xd, M);
                x_dot_mod(i,j) = v(1); y_dot_mod(i,j) = v(2);
            end
        end
    end
    % Update Obstacle Position
    object_center = object_center + dt * object_vel;
    % Plots and updates your plot 
    plot_with_gamma = 1; % <== To plot \Gamma(x) of obstacle
    if plot_with_gamma
        [h_cont, h_stream] = plot_ds_gamma(x, y, x_dot_mod, y_dot_mod, gamma);
        h_center = plot(reference_point(1), reference_point(2), 'r+', 'LineWidth', 3);
    else
        [h_cont, h_stream] = plot_ds(x, y, x_dot_mod, y_dot_mod);
        h_center = plot(reference_point(1), reference_point(2), 'y+', 'LineWidth', 3);
    end
    
    h_att = plot(x0(1), x0(2), 'b+','LineWidth', 3);
    h_obj = plot(x_object, y_object, 'b', 'LineWidth', 2);
    h_att_obs = plot(x0_obs(1), x0_obs(2), 'b*');
    if plot_with_gamma
        legend('\Gamma(x)', 'Reference Point', 'Attractor','Obstacle', ...
        'Obstacle Attractor', 'Location', 'SouthWest');
    else
         legend('Modulated DS', 'Modulated DS attractor', 'Obstacle', ...
            'Obstacle attractor', 'Obstacle reference point', 'Location', 'SouthWest')
    end

    xlim(x_limits);
    ylim(y_limits);


    drawnow;
    if t < n_iter
        delete(h_cont); delete(h_stream); delete(h_obj); delete(h_att); delete(h_att_obs); delete(h_center);
    end
end

function [h, h_stream] = plot_ds(x, y, x_dot, y_dot)
[~, h] = contourf(x, y, sqrt(x_dot.^2 + y_dot.^2), 80);
set(h, 'LineColor', 'none');
colormap('summer');
c_bar = colorbar;
c_bar.Label.String = 'Absolute velocity';
h_stream = streamslice(x, y, x_dot,y_dot, 2, 'method', 'cubic');
set(h_stream, 'LineWidth', 1);
set(h_stream, 'color', [0. 0. 0.]);
set(h_stream, 'HandleVisibility', 'off');
end

function [h, h_stream] = plot_ds_gamma(x, y, x_dot, y_dot, gamma)
[~, h] = contour(x, y, gamma, 80);
colormap('summer');
c_bar = colorbar;
c_bar.Label.String = '\Gamma(x)';
h_stream = streamslice(x, y, x_dot,y_dot, 2, 'method', 'cubic');
set(h_stream, 'LineWidth', 1);
set(h_stream, 'color', [0 0 0]);
set(h_stream, 'HandleVisibility', 'off');
end
