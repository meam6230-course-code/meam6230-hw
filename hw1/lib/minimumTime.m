% Task 2a: Minimum time 
% This function minimizes the time scaling parameter Tf to minimize
% trajectory time

% Input Shape: 
%      Please check the MATLAB Workspace after running it once

% Output Shape: 
%      cost: float
%

% The robot arm is modeled with the following state space representation:
% - X(i, 1:4): the ith time step state vector (joint angles) in a trajectory
% - U(i, 1:4): the ith time step input vector (joint velocities) in a trajectory
% - U(any index, 5): final time of the trajectory (constant for all timesteps), at 
% which the robot arm should reach a specified target
function cost = minimumTime(X, U, e, data, robot, target)

    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
end