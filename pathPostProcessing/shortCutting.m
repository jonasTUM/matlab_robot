function [path] = shortCutting(path,planningProblem)
%SHORTCUTTING If PTP motion [i, i+2] is collision free, delete i+1.
    numOfIterations = 10;
    for j = 1:numOfIterations
        numWaypoints = size(path,2);
        randomIndex = sort(randi(numWaypoints-1,2,1));
        collisionFreeMotion = checkPtpMotion(planningProblem, path(:,randomIndex(1)), path(:,randomIndex(2)));
        if collisionFreeMotion
        	path = [path(:,1:randomIndex(1)) path(:,randomIndex(2)+1:end)];
        end
    end
end
