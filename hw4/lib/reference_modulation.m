%% %%%%%%%%%%%%%%%%%%%%%%%%
%% Reference Modulation  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implementation of the reference-based modulation approach; i.e., using a
% reference direction r(x) pointing inside to obstacle to construct E

% Input Shape: 
%      x:                  scalar (position of x)
%      y:                  scalar (position of y)
%      reference_point:    2x1 matrix
%      gamma:              scalar     (\Gamma(x) boundary function)
%      gradient_gamma:     2x1 vector (\nabla\Gamma(x)


% Output Shape: 
%      D:                  2x2 matrix (Eigenvalue for Modulation matrix)
%      E:                  2x2 matrix (Eigenbasis for Modulation matrix)
%      M:                  2x2 matrix (Modulation matrix)

function [D, E, M] = reference_modulation(x, y, reference_point, gamma, gradient_gamma)
    D = eye(2); E = eye(2); M = eye(2); %<== output variables
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
end