function  [position] = config2position(x,planningProblem)
% CONFIG2POSE Extracts the end effector positions for the waypoints of a
% configurations space path.
    position = zeros(size(x,1),3);
    for i = 1:size(x,1)
        if isnan(x(i,1))
            continue
        else
            H = getTransform(planningProblem.rob, x(i,:)', planningProblem.globVar.endEffector);
            position(i,:) = tform2trvec(H);
        end
    end
end