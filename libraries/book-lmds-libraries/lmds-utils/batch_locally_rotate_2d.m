%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  batch_locally_rotate_2d %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Batch implementation of Locally rotating modulation with a desired angle 
% of rotation theta at modulation center point c exponentially decaying 
% with a region of influence ls resulting in \dot{x} = M(\phi(x))f(x)
% \phi(x) = h(x)\theta and h(x) = exp(-1/(2*ls^2)||x-c||^2)
%
% Input Shape:
%      x:                  2xM matrix (Array of M 2dimensional state vectors x)
%      xd:                 2xM matrix (Array of M 2dimensional vectors f(x))
%      c:                  2x1 matrix (coordinate of modulator)
%      ls:                 scalar (length-scale for effect of modulator)
%      theta:              scalar (angle of rotation in radian)          
%
% Output Shape: 
%      v:                  2xM matrix
%      h_x:                1xM matrix


function [v, h_x] = batch_locally_rotate_2d(x, xd, theta, ls, c)
[N,M] = size(x); % rows x columns (vector dim x samples)
v   = zeros(N,M); % <= this is the variable that has to be returned
h_x = zeros(1,M); % <= this is the variable that has to be returned

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --> Locally Decayng  Rotation 
% M(x) = [cos(phi(x)) -sin(phi(x)); 
%         sin(phi(x)) cos(phi(x))];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for j=1:M
    [~, v(:,j)] = locally_rotate_2d(x(:,j), xd(:,j), theta, ls, c);
end

end