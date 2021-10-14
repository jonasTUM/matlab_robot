function [path, bubbles] = elasticBands(path,planningProblem)
    % Loop through waypoints to generate bubbles.
    numWaypoints = size(path,2);
    bubbles = cell(2,numWaypoints);
    repulsiveF = zeros(4, numWaypoints);
    contractionF = zeros(4, numWaypoints);

    for ii = 1 : numWaypoints
        %% Compute Bubble.
        [~, ~, ~,~,~,distances, allWntPts] = checkRobotConfiguration(planningProblem, path(:,ii));
        
        if any(isnan(distances))
            continue;
        end
        
        [minDist, minIndex] = min(distances);
        closestPair = allWntPts(:,:,minIndex);
        if ~isnan(minDist)
            bubbles{1,ii} = minDist;
        end
        bubbles{2,ii} = closestPair(:,1);
        %% Compute repuslive force.
        bodyJacobian = geometricJacobian(planningProblem.rob,path(:,ii), planningProblem.rob.BodyNames{minIndex});
        linkPoint = getTransform(planningProblem.rob, path(:,ii), planningProblem.rob.BodyNames{minIndex});
        pointOnRobot = closestPair(:,1)-linkPoint(1:3,4);
        normal = closestPair(:,1)-closestPair(:,2);
        if minDist < 0.23 && ii ~= 1 && ii ~= numWaypoints
            repulsiveF(:,ii) = (1/minDist)*(transpose(bodyJacobian(4:6,:)-vec2skew(pointOnRobot)*bodyJacobian(1:3,:))*normal);            
            if norm(repulsiveF(:,ii)) > 0.001
                repulsiveF(:,ii) = repulsiveF(:,ii)/norm(repulsiveF(:,ii));
            end
        end
        if any(isnan(repulsiveF(:,ii)))
            repulsiveF(:,ii) = zeros(4,1);
        end
        %% Compute tension force.
        if ii ~= 1 && ii ~= numWaypoints
            b = path(:,ii);
            bMinus1 = path(:,ii-1);
            bPlus1 = path(:,ii+1);
            diff1 = bMinus1 - b;
            diff1 = diff1 / norm(diff1);
            diff2 = bPlus1 - b;
            diff2 = diff2 / norm(diff2);
            contractionF(:,ii) = diff1 + diff2;
        end
    end
    
    %% Apply forces on the bubble.
    for ii = 1 : numWaypoints
        if bubbles{1,ii} > 0.3
            alpha = 0.01;
            beta = 0.1;
        else
            alpha = 0.1;
            beta = 0.01;
        end
        path(:,ii) = path(:,ii) + alpha*repulsiveF(:,ii) + beta *contractionF(:,ii);
    end
    
end