function [vel] = lmds_2d_external(original_dynamics, exp_funct, exter_funct, query_pos, phi_c)

% Compute original dynamics at query points
vel     = feval(original_dynamics, query_pos);
dim     = size(vel,1);
samples = size(vel,2);

% Compute Activation Values
h_x     = feval(exp_funct, query_pos);
size(h_x)

% Computed externally modulated function
h_s    = feval(exter_funct, query_pos);
size(h_s)

display('Modulating Dynamics through Local Rotation');
% and modulate
vel = locally_rotate_2d_external(vel, h_s', h_x, phi_c);

end