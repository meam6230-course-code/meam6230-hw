%% %%%%%%%%%%%%%%%
%%  exp_loc_act %%
%%%%%%%%%%%%%%%%%%

% Exponentially decaying function for effect of local modulation
% Batch implementation of the RBF that for locally active regions
% h(x) = exp(-1/(2*ls^2)||x-c||^2)

% Input Shape: 
%      c:                  2x1 matrix (coordinate of modulator)
%      ls:                 scalar (length-scale for effect of modulator)
%      x:                  2xM matrix (Array of M 2dimensional vectors x)
%
% Output Shape: 
%      h:                  1xM matrix (Array of M 1dimensional scalars h(x))
%
function [h] = exp_loc_act(ls, c, x)

[N,M] = size(x); % rows x columns
c_s = repmat(c,[1 M]);
diff_norm = vecnorm(x-c_s);
b = 1/(2*ls^2);
h = exp(-b.*diff_norm.^2);  
%

end

