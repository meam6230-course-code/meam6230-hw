%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  external_stable_limit_cycle %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Batch implementation of an externally modulate stable limit cycle
% Batch evaluation of K query points (for simulation) at time s.

% Input Shape: 
%      x:                  2xK matrix (Array of K 2dimensional state vectors x)
%      s:                  scalar (current simulation time - in Seconds)
%      ds_fun:             function handle (use to valuate the nominal DS
%                                           f(x) as xd = ds_fun(x))
%      x0:                 2x1 vector (coordinate of modulator)
%      sigma:              scalar (Kernel width for effect of modulator)
%      radius:             scalar (radius of desired limit cycle)
%      theta:              scalar (in radian)
%      T_start:            scalar (time at which you want to start the 
%                                  de-activation of rotation - in Seconds)
%      decay_rate:         scalar (decay rate of the expontial function)

% Output Shape: 
%      v:                  2xK matrix (Array of K 2dimensional externally modulated dynamics)

function [v] = external_stable_limit_cycle(x, s, ds_fun, x0, sigma, radius, theta, T_start, decay_rate)
    xd  = ds_fun(x);    % Batch querying the nominal linear DS f(x)
    [N,K] = size(xd);
    v = zeros(N,K);     % Output variable batch evaluation of K query points x
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
end