%% %%%%%%%%%%%%%%%%%%%%%%%
%%  Spurious Attractor  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implementation of a spurious attractor with radius of influence r and
% strength of attraction p

% Input Shape: 
%      local_vec:     2x1 (a vector that points from target 
%                           spurious attractor to current position)
%      r:             scalar (influence radius)
%      p:             scalar (strength of spurious attractor)

% Output Shape: 
%      L:             2x2 matrix (Eigenvalue for Modulation matrix)
%      E:             2x2 matrix (Eigenbasis for Modulation matrix)
%      M:             2x2 matrix (Modulation matrix)

function [L, E, M] = spurious_modulation(local_vec, r, p)
    [N,M] = size(local_vec);
    L = eye(N); E = eye(N); M = eye(N); %<== output variables

    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
end