%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2025 Univerity of Pennsylvania                         %
%   Author: Nadia Figueroa                                               %
%   email:   nadiafig@seas.upenn.edu                                     %    
%                                                                                                                   %
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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%% TASK 4: Externally De-Active Limit Cycle  %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization of Grid and Nominal DS
% Create grid to evaluate and visualize DS
clear; close all; clc;
x_limits = [-5, 5];
y_limits = [-5, 5];
nb_gridpoints = 50;

% mesh domain
[X, Y] = meshgrid(linspace(x_limits(1), x_limits(2), nb_gridpoints), ...
                  linspace(y_limits(1), y_limits(2), nb_gridpoints));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulate stable limit cycle around the attractor while keeping it stable %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% From Part 2 (ch8_ex2_unstable_modulations)

% Construct Nominal Linear DS
A = -eye*(2); 
target = [0; 0];  
ds_lin = @(x) lin_ds(x,target, A);

x0     = [0; 0]; % Coordinates of modulator
sigma  = 1;      % Kernel width for effect of modulator
radius = 3;      % Radius of the limit cycle
theta  = deg2rad(90); % angle of rotation

for i=1:nb_gridpoints
    for j=1:nb_gridpoints
        x = [X(i,j) Y(i,j)]';
        [gamma(i,j), v] = stable_limit_cycle(x, ds_lin([X(i,j) Y(i,j)]'), x0, sigma, radius, theta);
        x_dot(i,j) = v(1); y_dot(i,j) = v(2);
    end
end

% Make function handle for batch evaluation of stable limit cycle
ds_fun = @(x)batch_stable_limit_cycle(x, ds_lin, x0, sigma, radius, theta);

% Initial points for simulation
x0_all = [5 -5 -5 5 0 0 -5 5;
          5 -5 5 -5 5 -5 0 0;];

% Simulation settings
opt_sim = [];
opt_sim.dt    = 0.01; %<== dt of simulation  
opt_sim.i_max = 1000;  %<== max iterations
opt_sim.tol   = 0.005;%<== convergence to attractor tolerance
opt_sim.plot  = 1;
[x_sim, ~]    = Simulation(x0_all ,[], ds_fun, opt_sim); 

% Plot simulations on top of vector field with h(x)
figure('Color',[1 1 1]); hold on;
plot_ds_h_modulation(X, Y, x_dot, y_dot, gamma, target, ...
  'DS simulation of Stable Limit Cycle', 'streamslice', x0);hold on;
for i=1:size(x0_all,2)
    plot(x_sim(1,:,i),x_sim(2,:,i),'b.'); hold on;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Test your External modulation function to untrap flow in  %
%  limit cycle after T_start sec                             % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Construct Nominal Linear DS
A      = -eye*(2); 
target = [0; 0];  
ds_lin = @(x) lin_ds(x,target, A);

% Define time when you want to de-activate the limit cycle modulation at T_start sec
dt          = 0.01;       % dt used for simulation and decay rate
T_start     = 0.5;        % second (time to start deactivation)
decay_rate  = 1/(20*dt);  % Hz (using dt above)


% Make function handle for batch evaluation of externally activated stable limit cycle
ds_fun = @(x,s)external_stable_limit_cycle(x, s, ds_lin, target, sigma, radius, theta, T_start, decay_rate);

% Initial points for simulation
x0_all = [5 -5 -5 5 0 0 -5 5;
          5 -5 5 -5 5 -5 0 0;];

% Simulation settings
opt_sim = [];
opt_sim.dt    = dt;    %<== dt of simulation  
opt_sim.i_max = 1000;  %<== max iterations
opt_sim.tol   = 0.005; %<== convergence to attractor tolerance
opt_sim.plot  = 0;
[x_sim]       = external_Simulation(x0_all ,[], ds_fun, opt_sim); 

% Plot simulations on top of vector field with h(s)
figure('Color',[1 1 1], 'Position',[561 446 560 420]); hold on;
plot_ds_h_modulation(X, Y, x_dot, y_dot, gamma, target, ...
  strcat('Externally activated DS simulation with h(s) and $T_{start}=$',num2str(T_start)), 'streamslice', x0);hold on;
for i=1:size(x0_all,2)
    plot(x_sim(1,:,i),x_sim(2,:,i),'b.'); hold on;
end

% Plot external activation function h(s) with s = t
figure('Color',[1 1 1],'Position',[1 678 559 188]); hold on;
s   = linspace(0, 10, 1/dt);
for i=1:size(s,2)
    h_s(1,i) = external_activation(s(1,i), T_start, decay_rate);
end
plot(s, h_s, "Color",[1 0 0],'LineWidth',3); 
title(strcat('External Activation Function h(s) with $T_{start}=$',num2str(T_start)),'Interpreter','latex')
xlabel('s','Interpreter','latex'); ylabel('h(s)','Interpreter','latex'); 
axis([min(s) max(s) 0 1])
grid on; axis tight;

%% Simulation function definitions
function [v] = batch_stable_limit_cycle(x, ds_fun, x0, sigma, radius, theta)
    xd  = ds_fun(x);   %Querying the nominal linear DS f(x)
    [N,M] = size(xd);
    for i=1:M
     [~, v(:,i)] = stable_limit_cycle(x(:,i), xd(:,i), x0, sigma, radius, theta);
    end
end