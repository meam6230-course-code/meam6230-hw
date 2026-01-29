function [vel] = lmds_2d(query_pos, original_dynamics, theta, ls, c)

% Batch compute of original dynamics f(x) at query points
vel     = feval(original_dynamics, query_pos);

display('Modulating Dynamics through Local Rotation on 2D Space.');

% Batch modulate give batch of original dynamics f(x)
vel = batch_locally_rotate_2d(query_pos, vel, theta, ls, c);

end