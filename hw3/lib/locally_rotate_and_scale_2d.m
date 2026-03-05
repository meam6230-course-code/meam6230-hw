%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  locally_rotate_and_scale_2d %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Batch implementation of locally rotating and scale modulation 
% given \phi(x) and \kappa(x) as inputs.
%
% Input Shape:
%      xd:                 2xM matrix (Array of M 2dimensional vectors f(x))
%      phi:                1xM matrix (Array of M rotation angles phi)
%      kappa:              1xM matrix (Array of M kappa-scalings kappa)
%
% Output Shape: 
%      v:                  2xM matrix
function v = locally_rotate_and_scale_2d(xd, phi, kappa)
[N,M] = size(xd);
v     = zeros(2,M);
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
end
