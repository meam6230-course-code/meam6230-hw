%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exercise Script for Chapter 3 of:                                       %
% "Robots that can learn and adapt" by Billard, Mirrazavi and Figueroa.   %
% Published Textbook Name: Learning for Adaptive and Reactive Robot       %
% Control: A Dynamical Systems Approach, MIT Press 2022                   %
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
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Draw 2D Dataset with GUI  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clc;
filepath = fileparts(which('ch3_ex0_drawData.m'));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-ds-opt')));
% cd(filepath); %<<== This might be necessary in some machines


% Create Figure
fig1 = figure('Color',[1 1 1]);

% Axis limits
limits = [-2.5 0.5 -0.45 1.25];
axis(limits)
% set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.25, 0.55, 0.2646 0.4358]);
grid on
axis equal
set(gcf, 'Units', 'Normalized','Position', [0.0208 0.4750 0.4646 0.4358])


% Draw Reference Trajectories
[data, hp] = draw_mouse_data_on_DS(fig1, limits);

% Process Drawn Data for DS learning
% [Data, Data_sh, att, x0_all, dt] = processDrawnData(data);


Data = []; Data_sh = []; x0_all = []; x0_end = [];
for l=1:length(data)    
    % Check where demos end and shift
    data_ = data{l};
    x0_end = [x0_end data_(1:2,end)];
    Data = [Data data_];
    x0_all = [x0_all data_(1:2,1)];    
    
    
    % Shift data to origin
    data_(1:2,:) = data_(1:2,:) - repmat(data_(1:2,end), [1 length(data_)]);
    data_(3:4,end) = zeros(2,1);

    Data_sh = [Data_sh data_];
end

% Position/Velocity Trajectories
M          = size(Data,1)/2;   
Xi_ref     = Data(1:2,:);
Xi_dot_ref = Data(3:end,:);

% Global Attractor of DS
att = mean(x0_end,2);


% Position/Velocity Trajectories
vel_samples = 10; vel_size = 0.5;
[h_data, h_att, h_vel] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);

% % Extract Position and Velocities
% M          = size(Data,1)/2;    
% Xi_ref     = Data(1:M,:);
% Xi_dot_ref = Data(M+1:end,:);
