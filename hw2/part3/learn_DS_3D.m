%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is a template file to import 3D dataset to learn a DS from    %
% trajectories, and exporting the DS to be deployed on the Panda robot.   %
%  
% ====>> You should use functions from part 2 of this homework!           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Import dependencies
close all; clear; clc
filepath = fileparts(which('learn_DS_3D.m'));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-ds-opt')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-sods-opt')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-phys-gmm')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-thirdparty')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-robot-simulation')));
addpath(genpath(fullfile(filepath, 'dataset')));
% cd(filepath); %<<== This might be necessary in some machines

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 1 (DATA LOADING): Choose among the predifined datasets %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load and convert 3D dataset
% Import from the dataset folder either:
% - Task 1: 'theoretical_DS_dataset.mat'
% - Task 2: 'MPC_train_dataset.mat'
% - Task 2: 'MPC_test_dataset.mat'
% - Task 3: '3D_Cshape_bottom_processed.mat'
% - Task 3: 'raw_demonstration_dataset.mat'

load("theoretical_DS_dataset.mat"); % --> Modify me to load different datasets!!
% usingSEDS --> Modify me if you will use seds or lpvds! 
% (necessary for parameter storing method used in robot_DS_control.m)
usingSEDS = false;
% filter --> Modify me if you want to pre-process the datasets 
% (relevant for task 3)
filter = false;

% All code below is used to extract trajectories in format amenable to
% learning the DS with the codes provided in part 2 .m scripts
nTraj = size(trajectories, 3);
nPoints = size(trajectories, 2);

Data = [];
attractor = zeros(3, 1);
x0_all = zeros(3, nTraj);

% When filter = true the next lines of code will apply a savitzky golay
% filter to your data (this is recommended for raw human demonstrations)
for i = 1:nTraj
    traj = trajectories(:,:,i);
    if filter
        % Filter Trajectories and Compute Derivativess with Savitzky Golay filter
        %   traj: The trajectory you want to filter
        %   sample_step: subsample the traj before filtering
        %   nth_order :     max order of the derivatives 
        %   n_polynomial :  Order of polynomial fit
        %   window_size :   Window length for the filter
        traj = sgolay_filter_smoothing(trajectories(:,:,i), 5, 1, 2, 10);
    end

    Data = [Data traj];
    x0_all(:,i) = traj(1:3,1);
    attractor = attractor + traj(1:3,end);
end
attractor = attractor / nTraj;

% Normalizing dataset attractor position
M = size(Data, 1) / 2; 
Data(1:M,:) = Data(1:M,:) - attractor;
x0_all = x0_all - attractor;
att = [0; 0; 0];

% Plot position/velocity Trajectories
vel_samples = 5; vel_size = 0.75; 
[h_data, h_att, ~] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);

% Extract Position and Velocities
M = size(Data,1) / 2;    
Xi_ref = Data(1:M,:);
Xi_dot_ref  = Data(M+1:end,:);   
axis_limits = axis;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Step 2: ADD YOUR CODE BELOW TO LEARN 3D DS      %%
%% vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %%%%%

%%% MODIFY ME!!!
%%% MODIFY ME!!!
%%% MODIFY ME!!!


%%   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ %%%
%%    Step 2: ADD YOUR CODE ABOVE TO LEARN 3D DS      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 3 (SAVE DS): Save learned DS parameters for robot control %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save DS for simulation using 'DS_control.m'
filename = strcat(filepath,'/ds_control.mat');
if usingSEDS
    ds_control = @(x) ds_seds(x - attractor);
    save('ds_control.mat', "ds_control", "attractor", "Priors", "Mu", "Sigma", "att", "M")
else
    ds_control = @(x) ds_lpv(x - attractor);
    save('ds_control.mat', "ds_control", "attractor", "ds_gmm", "A_k", "b_k", "att")
end