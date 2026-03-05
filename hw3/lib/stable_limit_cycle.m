%% %%%%%%%%%%%%%%%%%%%%%%
%%  stable_limit_cycle %%
%%%%%%%%%%%%%%%%%%%%%%%%%
% See Lecture 9 Slide 12
% Input Shape: 
%      x:                  2x1 vector (2-dimensional state vector x)
%      xd:                 2x1 vector (2-dimensional vectors f(x))
%      x0:                 2x1 vector (coordinate of modulator)
%      sigma:              scalar (Kernel width for effect of modulator)
%      radius:             scalar (radius of desired limit cycle)
%      theta:              scalar (in radian)

% Output Shape: 
%      gamma:              scalar (activation function)
%      v:                  2x1 matrix (modulated dynamics)

function [gamma, v] = stable_limit_cycle(x, xd, x0, sigma, radius, theta)   
     gamma = 0; v = [0;0];
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
end

