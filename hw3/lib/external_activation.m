%% %%%%%%%%%%%%%%%%%%%%%%%
%%  external_activation %%
%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implementation of an external activation funtion given s=t.
%
% Input Shape:
%      s:                  scalar (current simulation time - in Seconds)
%      T_start:            scalar (time at which you want to start the 
%                                  de-activation of rotation - in Seconds)
%      decay_rate:         scalar (decay rate of the expontial function)
%
% Output Shape: 
%      h_s:                scalar (value between 0-1)
function [h_s] = external_activation(s, T_start, decay_rate)
    h_s = 0;
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
end