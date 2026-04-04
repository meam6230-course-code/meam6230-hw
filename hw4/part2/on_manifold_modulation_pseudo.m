%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% On Manifold Modulation with Pseudo-Hession Approximation %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implementation of the on-manifold modulation approach; i.e., utilizing 
% isoline surface tracking via tangent direction \phi(x) and isoline
% surface convergence via gradient descent to achieve no-local-minima
% concave obstacle avoidance from:

% C. K. Fourie, N. Figueroa and J. A. Shah, "On-Manifold Strategies for 
% Reactive Dynamical System Modulation With Nonconvex Obstacles," in 
% IEEE Transactions on Robotics, vol. 40, pp. 2390-2409, 2024 

% This function is implementing the pseudo-hessian approximation exit strategy 
% suitable for non-convex obstacles (star-shaped, planar surfaces, small concavities)

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
%   epsilon                : small scalar user-defined coefficient

% Output Shape: 
%      D                   :2x2 matrix (Eigenvalue for Modulation matrix)
%      E                   :2x2 matrix (Eigenbasis for Modulation matrix)
%      M                   :2x2 matrix (Modulation matrix)

function [D, E, M] = on_manifold_modulation_pseudo(ds, gradient_ds, gamma, gradient_gamma, tangent_gamma, tangent_gamma_approx, hessian_gamma_approx,isoline, c_gamma, epsilon)

    % Construct local coordinate frame matrix
    % Columns are the normal and tangent directions of the obstacle
    E = [gradient_gamma, tangent_gamma];
    
    % Compute psi and psi' terms for lambda_delta component
    psi_combined = compute_psi(ds, gamma, gradient_gamma, tangent_gamma, isoline);
    
    % Compute D_grad component
    D_grad = compute_scaling_grad(ds, gamma, gradient_gamma, ...
                                        psi_combined, isoline, epsilon);
    
    % Compute D_phi component
    D_phi = compute_scaling_phi_pseudo(ds, gradient_ds, gamma, gradient_gamma, tangent_gamma, ...
                                    tangent_gamma_approx, hessian_gamma_approx, isoline,c_gamma);
    
    % Compute D_gamma component
    D_gamma = compute_scaling_gamma(gamma, isoline, c_gamma);
    
    % Combine all Lambda components (under Eq. 32)
    gamma_condition = double(gamma > isoline);
    D = D_phi+ D_grad + gamma_condition * D_gamma;
   
    % Transform D from local frame back to world frame 
    M = E * D * E.';

end