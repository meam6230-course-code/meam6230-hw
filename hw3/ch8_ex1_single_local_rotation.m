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
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Example: interactive_locally_rotating_dynamical_systems
%           Dx=M(x, theta, influence)*A*(x-target)     x(0)=Initial
%   input -----------------------------------------------------------------
%
%       o Target      : (2 x 1), The target Position.  (Optional)
%       o Influence   : (1 x 1), The influence of the modulation function.  
%                               (Optional)
%       o Theta       : (1 x 1), The rotation angle of the modulation function.  
%                               (Optional)
%       o A           : (2 x 2), The linear nominal system.   (Optional)
%  How to run -------------------------------------------------------------
%    interactive_locally_rotating_dynamical_systems() 
%    interactive_locally_rotating_dynamical_systems([0;0])
%    interactive_locally_rotating_dynamical_systems([0;0], 10)
%    interactive_locally_rotating_dynamical_systems([0;0], 10, 1.57)
%    interactive_locally_rotating_dynamical_systems([2;2], 10, 1.57, -4*eye(2))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%% TASK 2.1: Evaluate Locally Rotating Modulation %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%% Define parameters for a locally rotated linear DS  %%%%%%%%%%%

% --> When parameters are left empty [] the default values will be created
%%%% Define Linear System Parameters x_dot = A(x-x^*)
A = [] ;     % linear system matrix, Default A = -4*eye(2)
Target = []; % attractor x^*, Default Target = [0, 0]'

%%%% Define Parameters to construct locally rotating modulation matrix
% M(\phi(x)) with \phi(x) = h(x)\theta and h(x) = exp(-1/(2*ls^2)||x-c||^2)
Theta     = [];   % Desired Rotation Angle (rad), Default will be a random angle
ls        = [];  % Desired Influence Region, Default will be ls=0.5

% When modulation_centr c is not defined you will see the interactive GUI
% to define it, to set it manually you can define a (2 x 1) vector
c = [];

% Run interactive GUI to visualize a locally rotating modulation
interactive_locally_rotating_dynamical_systems(Target, ls, Theta, A, c)