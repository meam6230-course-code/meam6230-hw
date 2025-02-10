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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all; clc;
filepath = fileparts(which('ch3_ex4_designlpvDS.m'));
cd(filepath); %<<== This might be necessary in some machines

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Manually Design LPV-DS parameters %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Parameterize GMM variables %%%%
% Priors is a 1xK vector
Priors = zeros(1,2); % [pi_1, pi_2]
Priors = [0.5 0.5]; % [pi_1, pi_2] <== MODIFY ME

% Mu is a 2xK matrix and Sigma is a 2x2xK matrix
Mu = zeros(2,2);
Sigma = zeros(2,2,2);

Mu(:,1) = [0;0] ;% mu^1 <== MODIFY ME
Mu(:,2) = [0;0];% mu^2  <== MODIFY ME
Sigma(:,:,1) = eye(2);% Sigma^1 <== MODIFY ME
Sigma(:,:,2) = eye(2);% Sigma^2 <== MODIFY ME

% Create GMM data structure for DS
clear ds_gmm; ds_gmm.Mu = Mu; ds_gmm.Sigma = Sigma; ds_gmm.Priors = Priors; 

%%%%% Parameterize linear system variables %%%%
% A_k is a 2x2xK matrix
A_k = zeros(2,2,2);
A_k(:,:,1) = -eye(2); % A_1 <== MODIFY ME
A_k(:,:,2) = -eye(2); % A_2 <== MODIFY ME

% b_k is a 2xK matrix
b_k = zeros(2,2); % Assuming attractor is origin
att = [0;0];      % Attractor

%%%%% Define P matrix for P-QLF %%%%;
P = eye(2); % <== MODIFY ME

%%%%% Check GAS conditions %%%%
[Q1_eigs] = checkGAS_PQLF(A_k(:,:,1),P);
[Q2_eigs] = checkGAS_PQLF(A_k(:,:,2),P);

%%% Define the LPV-DS f(x) function %%%
ds_lpv = @(x) lpv_ds(x, ds_gmm, A_k, b_k);
ds_title = '$\dot{x}=\sum_{k=1}^2\gamma_k(x)A_k(x-x^*)$';

%%%%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
% Fill in plotting options
fig1 = figure('Color', [1 1 1]);
fig_limits = [-2 2 -2 2];
[hs] = plot_ds_model(fig1, ds_lpv, [0 0]', fig_limits, 'high'); hold on;
[ha] = scatter(0, 0, 200, 'kd', 'filled'); hold on
[ha] = text(0, -0.25, '$x^*$', 'Interpreter', 'LaTex', 'FontSize', 30, 'FontWeight', 'bold', ...
    'BackgroundColor', [1 1 1], 'EdgeColor', [0 0 0]); hold on
axis(fig_limits)
box on
grid on
xlabel('$x_1$', 'Interpreter', 'LaTex', 'FontSize', 35);
ylabel('$x_2$', 'Interpreter', 'LaTex', 'FontSize', 35);
title(ds_title, 'Interpreter', 'LaTex', 'FontSize', 35);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% (Optional - Stability Check 2D-only): Plot Lyapunov Function and derivative  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Type of plot
contour = 1; % 0: surf, 1: contour
clear lyap_fun_comb lyap_der 

% Lyapunov function
lyap_fun = @(x)lyapunov_function_PQLF(x, att, P);
title_string = {'$V(x) = (x-x^*)^TP(x-x^*)$'};

% Derivative of Lyapunov function (gradV*f(x))
lyap_der = @(x)lyapunov_derivative_PQLF(x, att, P, ds_lpv);
title_string_der = {'Lyapunov Function Derivative $\dot{V}(x)$'};

% Plots
h_lyap     = plot_lyap_fct(lyap_fun, contour, fig_limits,  title_string, 0);
h_lyap_der = plot_lyap_fct(lyap_der, contour, fig_limits,  title_string_der, 1);


function [Q_eigs] = checkGAS_PQLF(A,P)
    Q = A'*P+P*A;
    Q_eigs = eig(Q); 
    if any(Q_eigs > 0) 
        disp('System is not GAS wrt. P-QLF');
    else
        disp('System is GAS wrt. P-QLF');
    end
end
