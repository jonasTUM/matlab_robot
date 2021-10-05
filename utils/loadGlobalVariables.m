function [globalVariables] = loadGlobalVariables()
%LOADGLOBALVARIABLES Set up the global parameters of the planning problem.
    % Home configuration in [rad].
    globalVariables.desiredHomeConfiguration =  [pi/8 -pi/8 -pi/4 -pi/2]';
    % Maximum length of a motion allowed in the tree, specified as a scalar.
    globalVariables.connectionDistance = 0.5;
    % Lower and Upper joint limits of the robot in [¶ad].
    globalVariables.qMin(1:size(globalVariables.desiredHomeConfiguration,1),1) = -pi;
    globalVariables.qMax(1:size(globalVariables.desiredHomeConfiguration,1),1) = pi;
    % Name of the link that is defined as end effector.
    globalVariables.endEffector = 'endEffector';
    globalVariables.isExhaustiveChecking = true;
    % Resolution of the collision checking between roadmap waypoints in
    % [rad].
    globalVariables.maxCollisionFreeConfigStep = 0.05;
    % Dimension of the configuration space.
    globalVariables.dimensionConfig = size(globalVariables.desiredHomeConfiguration,1);
    % Time step sizes between the starting and ending of a trajectory.
    globalVariables.timeStep = 0.05; % [s]
    % Activation threshold for collision avoidance cost.
    globalVariables.activationThreshold = 0.1;
    globalVariables.goalBias = 0.1;
    globalVariables.motionDuration = 1.0;
end




