%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2026 University of Pennsylvania
%   Author: Nadia Figueroa/Yifan Xue
%   email:   nadiafig@seas.upenn.edu
%                                                                         %
%   Permission is granted to copy, distribute, and/or modify this program
%   under the terms of the GNU General Public License, version 2 or any
%   later version published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful, but
%   WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%   Public License for more details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('../lib');
%% %%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize simulation  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
x_limits = [-10, 10];
y_limits = [-10, 10];
nb_gridpoints = 400;

% mesh domain
[x, y] = meshgrid(linspace(x_limits(1), x_limits(2), nb_gridpoints), ...
                  linspace(y_limits(1), y_limits(2), nb_gridpoints));

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define and draw nominal linear DS f(x) %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = [-1, 0; ...
     0, -1];  % Linear DS with stable attractor 
b = [-6; 0];  
x0 = -inv(A) * b; % Attractor coordinate are at Ax+b=0 

ds_fun = @(x)(A*x + b);
data = feval(ds_fun, [reshape(x, 1, []); reshape(y, 1, [])]);
x_dot = reshape(data(1,:), nb_gridpoints, nb_gridpoints);
y_dot = reshape(data(2,:), nb_gridpoints, nb_gridpoints);

% Plot DS
figure('Color', [1 1 1]);
hold on; axis equal;
title('Nominal Linear DS','Interpreter','latex','FontSize',15);
plot_ds(x, y, x_dot, y_dot, 0);
plot(x0(1), x0(2), 'r*');
legend('Nominal DS', 'Attractor', 'Location', 'SouthWest');
xlim(x_limits);
ylim(y_limits);
drawnow;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% On-Manifold Modulated DS - Simple elliptic obstacle % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Color', [1 1 1]);
hold on; axis equal;

% Construct and plot ellipse
object_center = [1.5; 0];    % Coordinate of center of obstacle
ellipse_axes = [2; 5];     % half lengths of ellipse
theta = linspace(0, 2*pi);
x_object = (ellipse_axes(1)) * cos(theta) + object_center(1);
y_object = (ellipse_axes(2)) * sin(theta) + object_center(2);

% Set reference point
reference_point = [1; 3]; % Set reference point to create reference direction

% Set on-manifold modulation parameter <== Modify these parameters for HW
isoline = 2;
c_gamma = 10;
epsilon = 0.1;

% Initialize variables
x_dot_mod = zeros(nb_gridpoints, nb_gridpoints);
y_dot_mod = zeros(nb_gridpoints, nb_gridpoints);

for i=1:size(x, 1)
    for j=1:size(x, 2)

        % Compute distance function \Gamma(x) for a ellipse obstacle
        [gamma(i,j), gradient_gamma, hessian_gamma] = gamma_ellipse(x(i,j), y(i,j), object_center, ellipse_axes);
        
        % Query DS and compute tangent vector
        tangent_gamma = compute_tangent(gradient_gamma);
        ds = [x_dot(i,j); y_dot(i,j)];

        % Construct on-Manifold Modulation Matrix
        gradient_ds = A;
        [~, ~, M] = on_manifold_modulation_hess(ds, gradient_ds, gamma(i,j), gradient_gamma, tangent_gamma, hessian_gamma, isoline, c_gamma, epsilon);

        % If we are outside the obstacle (can use code from circular_modulation):
        if gamma(i,j) > 1
           xd = [x_dot(i,j) y_dot(i,j)]';
           xd_mod = apply_modulation(xd, M);
           x_dot_mod(i,j) = xd_mod(1); y_dot_mod(i,j) = xd_mod(2);
        end
        
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plotting options & function calls %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
title('On-Manifold Modulated DS with one elliptic obstacle','Interpreter','latex','FontSize',15);
plot_with_gamma = 1; % <== To plot \Gamma(x) of obstacle
if plot_with_gamma
    plot_ds_gamma(x, y, x_dot_mod, y_dot_mod, gamma);
else
    plot_speed = 1; %<== Set to 1 if you want to plot the absolute speed
    plot_ds(x, y, x_dot_mod, y_dot_mod, plot_speed);
end

plot(x_object, y_object, 'k', 'LineWidth', 2); hold on;
plot(object_center(1), object_center(2), 'bd'); hold on;
plot(x0(1), x0(2), 'r*');
legend('Modulated DS', 'Obstacle', 'Obstacle center', ...
    'Attractor', 'Location', 'NorthWest');
xlim(x_limits);
ylim(y_limits);

% Optional: Visualize trajectory rollouts
sim_traj = 1; %<=- set to 0 if you do not want to simulate rollouts

if sim_traj

    % Initial points for simulation
    x0_all = [ 8 8 8 8 8 8 8 8;
               10 5 0 -5 -10 2 -1.8 -1.5] + object_center;

    % Sanity Check using different initial points
    % Make function handle for batch evaluation of modulated DS
    ds_mod = @(x)batch_on_manifold_modulation_ellipse(x, ds_fun, gradient_ds, object_center, isoline, c_gamma, epsilon, ellipse_axes);

   
    opt_sim = [];
    opt_sim.dt    = 0.01;  %<== dt of simulation  
    opt_sim.i_max = 1000;  %<== max iterations
    opt_sim.tol   = 0.005; %<== convergence to attractor tolerance
    opt_sim.plot  = 0;
    [x_sim, ~]    = Simulation(x0_all ,[], ds_mod, opt_sim); 
    
    % Plot simulations on top of vector field with h(x)
    for i=1:size(x0_all,2)
        plot(x_sim(1,:,i),x_sim(2,:,i), '.', 'Color', [rand rand rand], 'LineWidth',10,'HandleVisibility','off'); hold on;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Appendix: Plotting and Auxiliary functions %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function tangent_gamma = compute_tangent(gradient_gamma)
    if norm(gradient_gamma) == 0
        tangent_gamma = [0;0];
    else
        R = [0 -1; 1 0];   % 90 degree rotation matrix
        tangent_gamma = R * gradient_gamma; % rotate gradient
    end
end

function [h, h_stream] = plot_ds(x, y, x_dot, y_dot, plot_speed)
    if plot_speed
        [~, h] = contourf(x, y, sqrt(x_dot.^2 + y_dot.^2), 80);
        set(h, 'LineColor', 'none');
        colormap('summer');
        c_bar = colorbar;
        c_bar.Label.String = 'Absolute velocity';
    end

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

function [v] = batch_on_manifold_modulation_ellipse(x, ds_fun, gradient_ds, object_center, isoline, c_gamma, epsilon, ellipse_axes)
    xd  = ds_fun(x);   %Querying the nominal linear DS f(x)
    [~,N] = size(xd);
    v = zeros(2, N);
    for i=1:N
         [gamma, gradient_gamma, hessian_gamma] = gamma_ellipse(x(1,i), x(2,i), object_center, ellipse_axes);
         tangent_gamma = compute_tangent(gradient_gamma);
         [~, ~, M] = on_manifold_modulation_hess(xd(:,i), gradient_ds, gamma, gradient_gamma, tangent_gamma, hessian_gamma,isoline, c_gamma, epsilon);
         v(:,i) = apply_modulation(xd(:,i), M);
    end
end