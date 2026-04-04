%% %%%%%%%%%%%%%%%
%%  gamma_star  %%
%%%%%%%%%%%%%%%%%%=
% Implementation of a \Gamma(x) function for a 2D star-shaped obstacle that 
% is monotonically increasing in the radial from the center of the obstacle.
% Note: This is provided to students, no need to implement or modify:
%
% Input Shape: 
%      x:                  scalar (position of x)
%      y:                  scalar (position of y)
%      object_center:      2x1 matrix
%      coef_b, coef_c:     parameters b and c

% Gamma:
%   gamma(x,y) = 1 + rho - R(theta)
%
% Outputs:
%   gamma           - scalar
%   gradient_gamma  - 2x1 gradient
%   hessian_gamma   - 2x2 Hessian
%
% The boundary is same as gamma_star implementation:
%   ((y-cy)^2 - coef_c)^2 + (x-cx)^4 = coef_c^2 + coef_b

function [gamma, gradient_gamma, hessian_gamma] = gamma_star_radial(x, y, object_center, coef_c, coef_b)

    % Obstacle Center
    cx = object_center(1);
    cy = object_center(2);
    
    % Relative coordinates and ray direction
    dx = x - cx;
    dy = y - cy;
    theta = atan2(dy, dx);

    % Closed-form R(theta), grad R, hessian R
    [R, grad_R, hess_R] = radial_boundary_and_derivatives(theta, coef_c, coef_b);

    % Radial length    
    rho2 = dx^2 + dy^2;
    rho  = sqrt(rho2);
    
    % Radial Gamma Function
    gamma = 1 + rho - R;
    
    % rho derivatives
    grad_rho = [dx; dy] / rho;
    hess_rho = (eye(2) - grad_rho * grad_rho.') / rho;
    
    % theta derivatives
    grad_theta = [-dy; dx] / rho2;
    
    % Gradient of Gamma
    gradient_gamma = grad_rho - grad_R * grad_theta;

    % Hessian of atan2(dy, dx)
    rho4 = rho2^2;
    hess_theta = [ 2*dx*dy / rho4,   (dy^2 - dx^2) / rho4;
                  (dy^2 - dx^2) / rho4,  -2*dx*dy / rho4 ];
        
    % Hessian of Gamma
    hessian_gamma = hess_rho - ( hess_R * (grad_theta * grad_theta.') ...
                  + grad_R * hess_theta );
end


function [R, grad_R, hess_R] = radial_boundary_and_derivatives(theta, coef_c, coef_b)
    % A(theta) = cos^4(theta) + sin^4(theta)
    % B(theta) = 2*c*sin^2(theta)
    A   = cos(theta)^4 + sin(theta)^4;
    Ap  = -sin(4*theta);
    App = -4*cos(4*theta);

    B   = 2*coef_c*sin(theta)^2;
    Bp  = 2*coef_c*sin(2*theta);
    Bpp = 4*coef_c*cos(2*theta);

    Q   = B^2 + 4*A*coef_b;
    D   = sqrt(Q);

    Qp  = 2*B*Bp + 4*coef_b*Ap;
    Qpp = 2*(Bp^2 + B*Bpp) + 4*coef_b*App;

    Dp  = Qp / (2*D);
    Dpp = Qpp / (2*D) - (Qp^2) / (4*D^3);

    % u = R^2 = (B + D) / (2*A)
    N   = B + D;
    Np  = Bp + Dp;
    Npp = Bpp + Dpp;

    u   = N / (2*A);

    % u' and u''
    M   = Np*A - N*Ap;
    up  = M / (2*A^2);

    Mp  = Npp*A - N*App;
    upp = (Mp*A - 2*M*Ap) / (2*A^3);
    
    % R, grad R, hessian R
    R       = sqrt(u);
    grad_R  = up / (2*R);
    hess_R  = upp / (2*R) - (up^2) / (4*R^3);
end