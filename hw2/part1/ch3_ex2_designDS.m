%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exercise Script for Chapter 3 of:                                       %
% "Robots that can learn and adapt" by Billard, Mirrazavi and Figueroa.   %
% Published Textbook Name: Learning for Adaptive and Reactive Robot       %
% Control, MIT Press 2022                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2020 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Nadia Figueroa                                                 %
% email:   nadia.figueroafernandez@epfl.ch                                %
% website: http://lasa.epfl.ch                                            %
%                                                                         % 
% Modified by Nadia Figueroa on Feb 2025, University of Pennsylvania      %
% email: nadiafig@seas.upenn.edu                                          %
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
%% %%%%%%%%%%% Generate a linear or non-linear DS  %%%%%%%%%%%
clear; close all; clc;
filepath = fileparts(which('ch3_ex2_designDS.m'));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-ds-opt')));
% cd(filepath); %<<== This might be necessary in some machines

% This script can be used to design and visualize simple linear systems 
% as well as non-linear systems with a constant gamma functions
% ds_case = 1: xdot = A(x-x*)
% ds_case = 2: xdot = (gamma_1A_1 + gamma_2A_2)(x-x*)
ds_case = 1; 

switch ds_case
    case 1
        % Here you can design an A matrix for a linear DS of the form 
        % xdot = A(x-x*) with x* at the origin
        
        A = [-1 0; 0 -1];  % <=== Modify this matrix!
        P = [1 0;   0 1]; % <=== Modify this matrix!         
        
        % Check GAS conditions
        A_symm_eigs = checkGAS_QLF(A);
        Q_eigs      = checkGAS_PQLF(A,P);

        ds_fun = @(x)(A*x); % This is f(x)
        ds_title = '$\dot{x}=A(x-x^*)$';
        
    case 2
        % Here you can design an A matrix for a linear DS of the form 
        % xdot = (gamma_1A_1 + gamma_2A_2)(x-x*) with x* at the origin
        
        % Modify these parameters !!
        % Current params are those to replicate example 3.1 in textbook
        % which yields an unstable system
        gamma1 = 0.5;
        gamma2 = 0.5;
        A1 = [-1 -10; 1 -1];
        A2 = [-1 1; -10 -1];
        P = [1 0; 0 1];

        % constant weighted sum of two matrices
        A_sum = gamma1*A1 + gamma2*A2;
        
        % Check GAS conditions
        A_symm_eigs = checkGAS_QLF(A_sum);
        Q_eigs      = checkGAS_PQLF(A_sum,P); 
        
        ds_fun = @(x)(A_sum*x); % This is f(x)
        ds_title = '$\dot{x}=(\gamma_1A^1 + \gamma_2A^2)(x-x^*)$';
end

%%%%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
% Fill in plotting options
fig1 = figure('Color', [1 1 1]);
fig_limits = [-1 1 -1 1];
[hs] = plot_ds_model(fig1, ds_fun, [0 0]', fig_limits, 'high'); hold on;
[ha] = scatter(0, 0, 200, 'kd', 'filled'); hold on
[ha] = text(0, -0.25, '$x^*$', 'Interpreter', 'LaTex', 'FontSize', 30, 'FontWeight', 'bold', ...
    'BackgroundColor', [1 1 1], 'EdgeColor', [0 0 0]); hold on
axis(fig_limits)
box on
grid on
xlabel('$x_1$', 'Interpreter', 'LaTex', 'FontSize', 35);
ylabel('$x_2$', 'Interpreter', 'LaTex', 'FontSize', 35);
title(ds_title, 'Interpreter', 'LaTex', 'FontSize', 35);


function [A_symm_eigs] = checkGAS_QLF(A)
    A_symm_eigs = eig(0.5*(A+A')); 
    if any(A_symm_eigs > 0)
        disp('System is not GAS wrt. QLF');
    else
        disp('System is GAS wrt. QLF');
    end
end

function [Q_eigs] = checkGAS_PQLF(A,P)
    Q = P*A+A'*P;
    Q_eigs = eig(Q); 
    if any(Q_eigs > 0) 
        disp('System is not GAS wrt. P-QLF');
    else
        disp('System is GAS wrt. P-QLF');
    end
end
