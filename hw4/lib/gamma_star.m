%% %%%%%%%%%%%%%%%
%%  gamma_star  %%
%%%%%%%%%%%%%%%%%%=
% Implementation of a \Gamma(x) function for a 2D star-shaped obstacle
% Note: This is provided to students, no need to implement or modify:

% Input Shape: 
%      x:                  scalar (position of x)
%      y:                  scalar (position of y)
%      object_center:      2x1 matrix
%      coef_b, coef_c:     parameters b and c

% Output Shape: 
%      gamma:               1x1 matrix
%      gradient_gamma:      2x1 matrix
%      hessian_gamma:       2x1 matrix

function [gamma, gradient_gamma, hessian_gamma] = gamma_star(x, y, object_center, coef_c, coef_b)

    % Obstacle Center
    cx = object_center(1);
    cy = object_center(2);
    
    % Compute gamma
    f = (((y - cy)^2 - coef_c)^2 + (x - cx)^4);
    gamma = f^(1/4) - (coef_c^2 + coef_b)^(1/4) + 1;
    
    
    % Avoid division by zero
    if f == 0
        gradient_gamma = [0; 0];
    else
        dgamma_dx = (x - cx)^3 / f^(3/4);
        dgamma_dy = ((y - cy)^2 - coef_c)*(y - cy) / f^(3/4);  % simplified
        gradient_gamma = [dgamma_dx; dgamma_dy];
    end
    
    % Hessian
    if f == 0
        hessian_gamma = zeros(2,2);
    else
        d2_xx = (-3*(x-cx)^6 + 3*(x-cx)^2 * f) / f^(7/4);
        d2_xy = -3*(x-cx)^3*(y-cy)*((y-cy)^2-coef_c)/f^(7/4);
        d2_yy = (-3*((y-cy)^2 - coef_c)^2*(y-cy)^2 + (3*(y-cy)^2 - coef_c)*f) / f^(7/4);
        hessian_gamma = [d2_xx, d2_xy;
                         d2_xy, d2_yy];
    end
end

