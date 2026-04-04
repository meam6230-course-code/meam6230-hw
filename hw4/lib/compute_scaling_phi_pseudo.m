%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute Scaling Matrix Phi for Pseudo-Hessian Approximation  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute Matrix D_\phi, in charge of tangent space dynamics in 
% on-manifold modulation. 

% Reference: 
% C. K. Fourie, N. Figueroa and J. A. Shah, "On-Manifold Strategies for 
% Reactive Dynamical System Modulation With Nonconvex Obstacles," in 
% IEEE Transactions on Robotics, vol. 40, pp. 2390-2409, 2024 

% Input Shape: 
%   ds                     : 2x1 nominal dynamical system vector field f(x)
%   gradient_ds            : 2x2 Jacobian of ds (∂f/∂x)
%   gamma                  : scalar distance value γ(x)
%   gradient_gamma         : 2x1 gradient of γ (normal direction of obstacle)
%   tangent_gamma          : 2x1 tangent vector along obstacle boundary
%   tangent_gamma_approx   : 2x1 tangent vector along the obstacle's circular apprximated boundary 
%   hessian_gamma_approx   : 2x2 Hessian of circle γ(x)
%   isoline                : scalar γ value defining the obstacle boundary
%   c_gamma                : tuning coefficient controlling gamma modulation strength

% Output Shape: 
%      D_phi                :2x2 matrix (Scaling matrix for on-manifold dynamics)

function D_phi = compute_scaling_phi_pseudo(ds, gradient_ds, gamma, gradient_gamma, ...
    tangent_gamma, tangent_gamma_approx, hessian_gamma_approx, isoline, c_gamma)
    % Initialize variable
    D_phi = zeros(2,2); 
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%

end