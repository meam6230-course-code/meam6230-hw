%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exercise Script for Chapter 8 of:                                       %
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
% Modified by Nadia Figueroa on Mar 2025, University of Pennsylvania      %
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
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%% TASK 1: Visualiza your Modulation matrices %%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%% Generate a linear or non-linear DS  %%%%%%%%%%%
clear; close all; clc;
filepath = fileparts(which('ch8_ex0_design_modulations.m'));
addpath(genpath(fullfile(filepath, '..', 'libraries')));
% cd(filepath); %<<== This might be necessary in some machines

% Define linear DS parameters: A, x^*
A = -eye(2);
target = zeros(2,1);

% This is M(x)f(x)
ds_fun = @(x)modulated_DS(x,A,target); 

%%%%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
% Fill in plotting options
ds_title = '$\dot{x}=M(x)A(x-x^*)$';
plot_range = 5;
[fig1] = plot_simple_ds(ds_fun, target, ds_title, plot_range);

% ===> Use this function to test your different modulation matrices!
function [v] = modulated_DS(x,A,target)
nX  = size(x,2); dim = size(x,1);
v = zeros(dim,nX);
f = A*(x-target);
for j=1:nX
    % Construct modulation matrix
    M = eye(2); %<<== Construct your modulation matrix here
   
    % Modulated dyamics
    v(:,j) = M*f(:,j);
end
end

