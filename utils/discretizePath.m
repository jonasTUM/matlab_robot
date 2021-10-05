function [discretizedPath] = discretizePath(path,planningProblem)
%DISCRETIZEPATH Samples intermediate points if distance between two
% waypoints is bigger than maxCollisionFreeConfigStep.
    discretizedPath = [];
    for ii = 1:size(path,2)-1
        distanceNorm = norm(path(:,ii)-path(:,ii+1));
        numOfSteps = floor(distanceNorm/planningProblem.globVar.maxCollisionFreeConfigStep);
        numOfSteps = max(numOfSteps,1);
        qDelta = (path(:,ii+1)-path(:,ii)) / numOfSteps;
        configurations = zeros(planningProblem.globVar.dimensionConfig,numOfSteps);
        for jj = 1:numOfSteps
            configurations(:,jj) = path(:,ii) + (jj-1) * qDelta;
        end
        discretizedPath = [discretizedPath configurations];
    end
    discretizedPath = [discretizedPath path(:,end)];
end
