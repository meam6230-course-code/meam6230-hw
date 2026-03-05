%% %%%%%%%%%%%%%%%%%%
%%  unstable_point %%
%%%%%%%%%%%%%%%%%%%%%
% See Lecture 9 Slide 15
% Input Shape: 
%      x:                  2x1 vector (2-dimensional state vector x)
%      xd:                 2x1 vector (2-dimensional vectors f(x))
%      x0:                 2x1 vector (coordinate of modulator)
%      sigma:              scalar (Kernel width for effect of modulator)

% Output Shape: 
%      gamma:              scalar (activation function)
%      v:                  2x1 matrix (modulated dynamics)

function [gamma, v] = unstable_point(x, xd, x0, sigma)   
    gamma = 0; v = [0;0];
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
end

