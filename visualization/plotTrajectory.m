function plotTrajectory(planProb, configurations)
%PLOTTRAJECTORY Visualizes a sequence of configuration space waypoints in 
% the current figure.
    for ii = 1:size(configurations,2)
        axisHandles = plotCollisionRobot(planProb.rob, planProb.collRob, configurations(:,ii),gca);
        pause(0.1);
        if ii < size(configurations,2)
            for jj = 1 : numel(axisHandles)
                delete(axisHandles{jj});
            end
        end
    end
end
