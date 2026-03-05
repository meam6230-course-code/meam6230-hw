%% %%%%%%%%%%%%%%%%%%%%%
%%  locally_rotate_2d %%
%%%%%%%%%%%%%%%%%%%%%%%%

% Implementation of Locally rotating modulation with a desired angle 
% of rotation theta at modulation center point c exponentially decaying 
% with a region of influence ls resulting in \dot{x} = M(\phi(x))f(x)
% \phi(x) = h(x)\theta and h(x) = exp(-1/(2*ls^2)||x-c||^2)

% Input Shape:
%      x:                  2x1 vector (2-dimensional state vector x)
%      xd:                 2x1 vector (2-dimensional vectors f(x))
%      c:                  2x1 vector (coordinate of modulator)
%      ls:                 scalar     (length-scale for effect of modulator)
%      theta:              scalar     (angle of rotation in radian)          
%
% Output Shape: 
%      v:                  2x1 vector ( value of \dot{x} = M(\phi(x))f(x) )
%      h:                  scalar     ( value of h(x) )

function [h, v] = locally_rotate_2d(x, xd, theta, ls, c)
    h = 0; v = [0;0];
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%

end

