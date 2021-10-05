function [collisionFreeMotion] = checkPtpMotion(planProb, start, goal)
%CHECKROBOTMOTION Checks a point to point robot motion. start is always
%formulated as a joint configuration so that the determination of the
%initial guess can be done. The goal can either be formulated in task space
%or joint space.
    % Get the joint configurations to check collisions for.
    if size(goal,2) == 1 % Config space.
        distanceNorm = abs(goal-start);
        numOfSteps = floor(max(distanceNorm/planProb.globVar.maxCollisionFreeConfigStep));
        qDelta = (goal - start) / numOfSteps;
        configurations = zeros(planProb.globVar.dimensionConfig,numOfSteps);
        for ii = 1:numOfSteps
            configurations(:,ii) = start + ii * qDelta;
        end
    else % Task space.
        configurations = getConfigurationsFromTaskSpace(planProb,start,goal);
    end
    % Perform the collision checks in joint space.
    collisionFreeMotion = true;
    for ii = 1:size(configurations,2)
        [isCollision,~,~,~,~] = checkRobotConfiguration(planProb,configurations(:,ii));
        if(isCollision)
            collisionFreeMotion = false;
            return;
        end
    end
end

function [configurations] = getConfigurationsFromTaskSpace(planProb,start,goal)
    startPose = getTransform(planProb.rob,start,'endEffector');    
    % The tranformations will be interpolated via a generated trajectory.
    [waypoints,~,~] = transformtraj(startPose,goal,[0 10],0:0.1:10);
    % Compute the inverse kinematics for the waypoints.
    initialGuess = start;    
    configurations = zeros(planProb.globVar.dimensionConfig,size(waypoints,3));
    for ii = 1 : size(waypoints,3)
        configWaypoint = invKinPositionLevel(planProb,initialGuess, waypoints(:,:,ii));
        initialGuess = configWaypoint;
        configurations(:,ii) = configWaypoint;
    end
end