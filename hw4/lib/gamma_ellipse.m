%% %%%%%%%%%%%%%%%%%
%%  gamma_ellipse %%
%%%%%%%%%%%%%%%%%%%%
% Implementation of a \Gamma(x) function for a 2D elliptical obstacle

% Input Shape: 
%      x:                  scalar (position of x)
%      y:                  scalar (position of y)
%      object_center:      2x1 matrix
%      ellipse_axes:       2x1 matrix

% Output Shape: 
%      gamma:               2x1 matrix
%      gradient_gamma:      1x1 matrix
%      hessian_gamma:       2x2 matrix


function [gamma, gradient_gamma, hessian_gamma] = gamma_ellipse(x, y, object_center, ellipse_axes)
    gamma = 0; gradient_gamma = zeros(2,1); hessian_gamma = zeros(2,2);
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%

end

