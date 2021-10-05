function [environment] = loadEnvironment()
%LOADENVIRONMENT Sets up the collision models for the enviornment.
    % Used colors.
    blue = [0 0.396 0.741 1.0]; %TUMBlue
    box{1} = collisionBox(0.1, 0.5, 0.5);
    box{1}.Pose = trvec2tform([0.2 -0.6 0.0]);
    box{2} = collisionBox(0.1, 0.5, 0.5);
    box{2}.Pose = trvec2tform([0.7 -0.6 0.0]);
    for ii = 1:length(box)
        box{ii} = {box{ii}, blue};
    end
    environment = box(:)'; 
end
