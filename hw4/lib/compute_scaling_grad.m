%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute Scaling Gradient Ascent Matrix  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute Matrix D_\nabla, in charge of gradient ascent dynamics in 
% on-manifold modulation. 

% Reference: 
% C. K. Fourie, N. Figueroa and J. A. Shah, "On-Manifold Strategies for 
% Reactive Dynamical System Modulation With Nonconvex Obstacles," in 
% IEEE Transactions on Robotics, vol. 40, pp. 2390-2409, 2024 

% Input Shape: 
%   ds              : 2x1 nominal dynamical system vector field f(x)
%   gamma           : scalar distance value \Gamma(x)
%   gradient_gamma  : 2x1 gradient of \Gamma (normal direction of obstacle)
%   psi_combined    : 2x1 vector correcting gradient ascent direction
%   isoline         : scalar \gamma value defining the obstacle boundary
%   epsilon         : small scalar user-defined coefficient

% Output Shape: 
%      D_grad       :2x2 matrix (Scaling Matrix for Gradient Ascent Dynamics)

function D_grad = compute_scaling_grad(ds, gamma, gradient_gamma, ...
                                             psi_combined, isoline, epsilon)
    % Initialize variable
    D_grad = zeros(2,2); 
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%

end