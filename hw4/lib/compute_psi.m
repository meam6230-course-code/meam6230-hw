%% %%%%%%%%%%%%%%%
%% Compute PSI  %%
%%%%%%%%%%%%%%%%%%
% Solve for \psi(n) and \psi'(e) to correct gradient ascent directions, 
% where n and e are the normal and tangent gamma vector

% Reference: 
% C. K. Fourie, N. Figueroa and J. A. Shah, "On-Manifold Strategies for 
% Reactive Dynamical System Modulation With Nonconvex Obstacles," in 
% IEEE Transactions on Robotics, vol. 40, pp. 2390-2409, 2024 

% Input Shape: 
%   ds              : 2x1 nominal dynamical system vector field f(x)
%   gamma           : scalar distance value \Gamma(x)
%   gradient_gamma  : 2x1 gradient of \Gamma (normal direction of obstacle)
%   tangent_gamma   : 2x1 tangent vector along obstacle boundary
%   isoline         : scalar \gamma value defining the obstacle boundary


% Output Shape: 
%      psi_combined : 2x1 vector consisting of [\psi(n); \psi'(e)]

function psi_combined = compute_psi(ds, gamma, gradient_gamma, tangent_gamma, isoline)
    % Initialize variable
    psi_combined = zeros(2,1); 
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
end