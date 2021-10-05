function [optimizedPath] = pathLengthOptimization(planningProblem,initialPath)
%POSTPROCESSINGPATHLENGTH Optimizes the resulting path length in joint
%space.
    start = initialPath(:,1);
    goal = initialPath(:,end);
    initialGuess = reshape(initialPath,[size(initialPath,2)*planningProblem.globVar.dimensionConfig 1]);
    lb = repmat(planningProblem.globVar.qMin,size(initialPath,2),1);
    ub = repmat(planningProblem.globVar.qMax,size(initialPath,2),1);
    options = optimoptions('fmincon', ...
        'FunValCheck', 'on', ...
        'Display', 'iter-detailed', ... 
        'Algorithm', 'sqp', ...
        'MaxFunEvals', 1e5, ...
        'MaxSQPIter', 1e6, ...
        'MaxIter', 10, ...
        'TolCon',1e-6, ...
        'SpecifyObjectiveGradient', true, ...
        'SpecifyConstraintGradient', true, ...
        'FiniteDifferenceType', 'forward', ...
        'FiniteDifferenceStepSize', 1e-6, ...
        'ScaleProblem', true, ...
        'CheckGradients', false ...
    );
    [optimizedPath, ~] = fmincon(@(x)costPathLength(x,planningProblem.globVar.dimensionConfig), ...
                        initialGuess, ... 
                        [],[],[],[], ...
                        lb,ub, ...
                        @(x)constraints(x,planningProblem,start,goal), ...
                        options);   
    optimizedPath = reshape(optimizedPath,size(initialPath));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% Optimization Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Optimization Objective - Path Length in Configuration Space.
function [l,grad] = costPathLength(x, numDof)
    steps = length(x) / numDof;
    l = 0;
    grad = zeros(size(x));
    % Cost.
    for ii = 1:steps-1
        index = (ii-1)*numDof+1:ii*numDof;
        indexPlus1 = index + numDof;
        diff = x(index)-x(indexPlus1);
        c = 0.5*(diff'*diff);
        l = l + c;
    end
    % Gradient.
    for ii = 1:steps
        index = (ii-1)*numDof+1:ii*numDof;
        indexPlus1 = index + numDof;
        indexMinus1 = index - numDof;
        if ii == 1
            grad(index) = (x(index)-x(indexPlus1));
        elseif ii == steps
            grad(index) = -(x(indexMinus1)-x(index));
        else
            grad(index) = (x(index)-x(indexPlus1)) - (x(indexMinus1)-x(index));
        end
    end
end

%% Constraints - Start, Goal, Discretization, Collision.
function [ineq, eq, gradIneq, gradEq] = constraints(x, pP, start, goal)
    numJoints = pP.globVar.dimensionConfig;
    steps = length(x) / numJoints;
    
    %% Start and Goal Constraint - Equality Constraints.
    numStartGoalConstraints = numJoints * 2;
    eq = [x(1:numJoints) - start; ...
          x(end-numJoints+1:end) - goal];
    jacobianEq = zeros(numStartGoalConstraints, length(x));
    jacobianEq(1:numJoints,1:numJoints) = eye(numJoints);
    jacobianEq(end-numJoints+1:end,end-numJoints+1:end) = eye(numJoints);
    gradEq = jacobianEq';
    
    %% Discretization Constraints - Equality Constraints.
    sizeWaypointConstraint = (steps-2);
    eqWaypoints = zeros(sizeWaypointConstraint, 1);
    jacobianEqWaypoints = zeros(sizeWaypointConstraint, numJoints*steps);
    for ii = 1:steps-2
        index = (ii-1)*numJoints+1 : ii*numJoints;
        indexPlus1 = index + numJoints;
        indexPlus2 = index + 2*numJoints;
        diff1 = x(indexPlus1)-x(index);
        diff2 = x(indexPlus2)-x(indexPlus1);
        halfDistanceSquared1 = 0.5*(diff1'*diff1);
        halfDistanceSquared2 = 0.5*(diff2'*diff2);
        eqWaypoints(ii) = halfDistanceSquared1 - halfDistanceSquared2;
        % Gradient.
        jacobianEqWaypoints(ii, index) = diff1' * (-eye(numJoints));
        jacobianEqWaypoints(ii, indexPlus1) = diff1' * eye(numJoints)+ diff2' * eye(numJoints);
        jacobianEqWaypoints(ii, indexPlus2) = diff2' * (-eye(numJoints));

    end
    eq = [eq; eqWaypoints];
    gradEq = [gradEq jacobianEqWaypoints'];  

    %% Collision Constraints - Inequality Constraints.
    numBodies = pP.rob.NumBodies - 1;
    numObstacles = numel(pP.collWorld);
    sizeCollisionConstraint = steps * numBodies * numObstacles;
    ineqColl = zeros(sizeCollisionConstraint,1);
    jacobianIneqColl = zeros(sizeCollisionConstraint, steps*numJoints);
    % inequality constraint: collision avoidance
    % cineq = threshold - dist <= 0
    % Loop through the steps.
    for ii=1:steps
        robotConfig = x((ii-1)*numJoints+1:ii*numJoints);
        [~, ~, ~,~,~,distances, allWntPts] = checkRobotConfiguration(pP, robotConfig);
        % The distances are stored within a multidim. matrix of the form
        % distances[1][robotArmIndex][worldObjectIndex]. E.g. the distance
        % between the second robot arm and first collision object in the
        % world can be obtained via distances(1,2,1).
        % Concerning the inequality constraint order, we will first
        % iterate the corresponding robot arm with all collision world
        % objects.
        for j = 1 : numBodies
            for k = 1 : numObstacles
                if distances(1,j,k)
                    index = (ii-1)*numObstacles*numBodies+(j-1)*numObstacles+k;
                    distance = distances(1,j,k);
                    % Gradient
                    normal = (allWntPts(:,1,j,k)-allWntPts(:,2,j,k));
                    bodyJacobian = geometricJacobian(pP.rob,robotConfig, pP.rob.BodyNames{j});
                    linkPoint = getTransform(pP.rob, robotConfig, pP.rob.BodyNames{j});
                    pointOnRobot = allWntPts(:,1,j,k)-linkPoint(1:3,4);
                    currentCollisionGradient = (1/distance)*(transpose(bodyJacobian(4:6,:)-vec2skew(pointOnRobot)*bodyJacobian(1:3,:))*normal);
                    ineqColl(index) = pP.globVar.activationThreshold - distance;
                    jacobianIneqColl(index, (ii-1)*numJoints+1:ii*numJoints)= -currentCollisionGradient';
                elseif distances(1,j,k) == 0
                    distance = 1e-4;
                    bodyJacobian = geometricJacobian(pP.rob,robotConfig, pP.rob.BodyNames{j});
                    linkPoint = getTransform(pP.rob, robotConfig, pP.rob.BodyNames{j});
                    pointOnRobot = allWntPts(:,1,j,k)-linkPoint(1:3,4);
                    normal = (allWntPts(:,1,j,k)-allWntPts(:,2,j,k));
                    currentCollisionGradient = (1/distance)*(transpose(bodyJacobian(4:6,:)-vec2skew(pointOnRobot)*bodyJacobian(1:3,:))*normal);                    
                    ineqColl(index) = pP.globVar.activationThreshold - distance;
                    jacobianIneqColl(index, (ii-1)*numJoints+1:ii*numJoints)= -currentCollisionGradient';
                end
            end
        end
    end
    ineq = ineqColl;
    gradIneq = jacobianIneqColl';  
end
