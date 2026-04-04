%% %%%%%%%%%%%%%%%%%%%%%%
%%  Normal Modulation  %%
%%%%%%%%%%%%%%%%%%%%%%%%%
% Implementation of the normal-based modulation approach; i.e., using the
% normal of the obstacle surface n(x)=\nabla\Gamma(x) to construct E

% Input Shape: 
%      gamma:              scalar     (\Gamma(x) boundary function)
%      gradient_gamma:     2x1 vector (\nabla\Gamma(x)


% Output Shape: 
%      D:                  2x2 matrix (Eigenvalue for Modulation matrix)
%      E:                  2x2 matrix (Eigenbasis for Modulation matrix)
%      M:                  2x2 matrix (Modulation matrix)

function [D, E, M] = normal_modulation(gamma, gradient_gamma)
    D = eye(2); E = eye(2); M = eye(2); %<== output variables
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
end

