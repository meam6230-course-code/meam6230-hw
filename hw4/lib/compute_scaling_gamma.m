%% %%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute Scaling Gamma  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute Matrix D_\gamma, in charge of classic modulation behavior
% in on-manifold modulation. 

% Reference: 
% C. K. Fourie, N. Figueroa and J. A. Shah, "On-Manifold Strategies for 
% Reactive Dynamical System Modulation With Nonconvex Obstacles," in 
% IEEE Transactions on Robotics, vol. 40, pp. 2390-2409, 2024 

% Input Shape: 
%   gamma           : scalar distance value \Gamma(x)
%   isoline         : scalar \gamma value defining the obstacle boundary
%   epsilon         : small scalar user-defined coefficient
%   c_gamma         : tuning coefficient controlling gamma modulation strength

% Output Shape: 
%      D_gamma      :2x2 matrix (Eigenvalue for Modulation matrix)

function D_gamma = compute_scaling_gamma(gamma, isoline, c_gamma)
    % Initialize variable
    D_gamma = zeros(2,2); 
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%    
end