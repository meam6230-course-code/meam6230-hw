%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Sina Mirrazavi
%   email:   sina.mirrazavi@epfl.ch
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
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  Public License for more details
%  Name of the chapter: Modulate dynamical systems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Example: locally_modulating_dynamical_systems
%           Dx=M(x, phi_c, Influence)*A*(x-Target)     x(0)=Initial
%   input -----------------------------------------------------------------
%
%       o Target      : (2 x 1), The target Position.  (Optional)
%       o Influence   : (1 x 1), The influence of the modulation function.  
%                               (Optional)
%       o Phi_c       : (1 x 1), The rotation angle of the modulation function.  
%                               (Optional)
%       o A           : (2 x 2), The linear nominal system.   (Optional)
%  How to run -------------------------------------------------------------
%    modulating_dynamical_systems() 
%    modulating_dynamical_systems([0;0])
%    modulating_dynamical_systems([0;0], 10)
%    modulating_dynamical_systems([0;0], 10, 1.57)
%    modulating_dynamical_systems([2;2], 10, 1.57, -4*eye(2))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function interactive_locally_rotating_dynamical_systems(Target, Influence, Phi_c, A, modulationCenter)

fig1 = figure('Color', [1 1 1]);
axis equal
title('Feasible Robot Workspace','Interpreter','LaTex')

if ((exist('Target')==0) || isempty(Target))
    % The target position of the DS
    Target = [0;0];
else
    if all(size(Target)~=[2,1])
        disp('The dimension of the target is not right')
        return;
    end
end

if ((exist('Influence')==0) || isempty(Influence))
    % The influence of the modulation function
    Influence=0.5;
else
    if all(size(Influence)~=[1,1])
        disp('The dimension of Influence is not right')
        return;
    end
    if Influence<0
        disp('Influence should be positive.')
        return;
    end
end

if ((exist('Phi_c')==0) || isempty(Phi_c)) 
    % Compute Random Rotation Angle phi_c
    a = 0; b =3.14;
    Phi_c = a+(b-a)*rand(1,1);
else
    if all(size(Phi_c)~=[1,1])
        disp('The dimension of phi_c is not right')
        return;
    end
    if Phi_c>6.2832
        disp('phi_c should be between 0 and 2\pi')
        return;
    end
    if Phi_c<0
        disp('phi_c should be between 0 and 2\pi')
        return;
    end
end

if ((exist('A')==0) || isempty(A))
    % nominal dynamical system
    A=-4*eye(2);
else
    if all(size(A)~=[2,2])
        disp('The dimension of A is not right')
        return;
    end
end

% Axis limits
limits = [-2+Target(1) 2+Target(1) -2+Target(2) 2+Target(2)];

% Plot Attractor
scatter(Target(1),Target(2),50,[0 0 0],'+'); hold on;

% IMPORTANT : You should also call "simulate_lmds" with a 7th parameter 
% corresponding to the position of modulation location 
% to deactivate the user input in the GUI.
% Example : simulate_lmds_rotation(Target, limits,fig1, Influence, Phi_c, A, [1;1]);
% vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
simulate_lmds_rotation(Target, limits,fig1, Influence, Phi_c, A, modulationCenter);


end