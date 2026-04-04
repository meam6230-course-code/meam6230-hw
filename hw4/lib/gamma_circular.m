%% %%%%%%%%%%%%%%%%%%
%%  gamma_circular %%
%%%%%%%%%%%%%%%%%%%%%
% Implementation of a \Gamma(x) function for a 2D circular obstacle

% Input Shape: 
%      x:                  scalar (position of x)
%      y:                  scalar (position of y)
%      object_center:      2x1 matrix
%      radius:             scalar

% Output Shape: 
%      gamma:               2x1 matrix
%      gradient_gamma:      1x1 matrix
%      hessian_gamma:       2x2 matrix

function [gamma, gradient_gamma, hessian_gamma] = gamma_circular(x, y, object_center, radius)
    gamma = 0; gradient_gamma = zeros(2,1); hessian_gamma = zeros(2,2);
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
end


