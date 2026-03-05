%% %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  multi_locally_rotate_2d %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Implementation of Locally rotating modulation with a desired angle 
% of rotation theta at modulation center point c exponentially decaying 
% with a region of influence ls resulting in \dot{x} = M(\phi(x))f(x)
% \phi(x) = h(x)\theta and h(x) = exp(-1/(2*ls^2)||x-c||^2)

% Input Shape:
%      x:                  2x1 vector (2-dimensional state vector x)
%      xd:                 2x1 vector (2-dimensional vectors f(x))
%      c_k:                2xK matrix (coordinates of K modulators)
%      ls_k:               1xK vector (K length-scales for effect of each modulator)
%      theta_k:            1xK vector (K angles of rotation in radian)
%      multi_strategy:     scalar     (modulation strategy 
                                      % 0: Naive product of modulations
                                      % 1: Your custom strategy)
%
% Output Shape: 
%      v:                  2x1 vector (final modulated velocity)
%      h_x:                scalar     (locally active function value)

function [h_x, v] = multi_locally_rotate_2d(x, xd, theta_k, ls_k, c_k, multi_strategy)
 
    [N,K] = size(c_k);
    h_x = 0; v = [0;0];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%

end


