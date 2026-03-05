%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Sina Mirrazavi
%   email:   sina.mirrazavi@epfl.ch
%   website: lasa.epfl.ch
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
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  Public License for more details
%  Name of the chapter: Modulate dynamical systems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Example: locally_modulating_dynamical_systems
%           Dx=M(x)*A*(x-Target)     x(0)=Initial
%   input -----------------------------------------------------------------
%       o A           : (2 x 2), The linear nominal system.   (Optional)
%
%  How to run -------------------------------------------------------------
%    locally_modulating_dynamical_systems(-4*eye(2))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%% TASK 3.1: Evaluate Learned Locally Rotated-Scaled Modulation %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Define Linear System Parameters x_dot = A(x-x^*)
A=-40*eye(2);     % linear system matrix, Default A = -4*eye(2)
Target = [0;0];  % attractor x^*, Default Target = [0, 0]'

%%%% Define Parameters to construct locally rotating modulation matrix
% GPR RBF Kernel = exp(-1/(2*ls^2)||x-x'||^2)
ls_gpr = 0.25;  % ls: length-scale for kappa(x) and phi(x) GPR norm-scaling regressor

% Figure Definitions
fig1 = figure('Color',[1 1 1]);
axis equal
limits = [-2+Target(1) 2+Target(1) -2+Target(2) 2+Target(2)];

% This will pop up the GUI to draw data on a linear DS and reshape
data = []; % When empty GUI will pop-up to draw
data = simulate_lmds_gp_learned(Target, limits,fig1, ls_gpr, A, data);

%% Try another value of ls with the same data

%%%% Define Parameters to construct locally rotating modulation matrix
% GPR RBF Kernel = exp(-1/(2*ls^2)||x-x'||^2)
ls_gpr = 0.15;  % ls: length-scale for kappa(x) and phi(x) GPR norm-scaling regressor

% Figure Definitions
fig1 = figure('Color',[1 1 1]);
axis equal
limits = [-2+Target(1) 2+Target(1) -2+Target(2) 2+Target(2)];

% This will pop up the GUI to draw data on a linear DS and reshape
data = simulate_lmds_gp_learned(Target, limits,fig1, ls_gpr, A, data);
