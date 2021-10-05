function [collisionRobot] = loadCollisionRobot(robot)
%LOADCOLLISIONROBOT Generates the collision mesh for a robot.
%   Reads in the STL files and generate a collision model based on those.
%   Might by replaced with a collision model based on simpliefied
%   geometries. See
%   https://de.mathworks.com/help/robotics/examples/create-collision-objects-for-manipulator-collision-checks.html
%   for further information.
    % Assumption on the radius of the cylinder elements.
    radius = 0.02;
    dimensionArray = [...
        radius 0; ... % Base
        radius, robot.Bodies{2}.Joint.JointToParentTransform(1,4); % link1
        radius, robot.Bodies{3}.Joint.JointToParentTransform(1,4); % link2
        radius, robot.Bodies{4}.Joint.JointToParentTransform(1,4); % link3
        radius, robot.Bodies{5}.Joint.JointToParentTransform(1,4); % link4       
        radius 0]; % endEffector
    collisionRobot = { ...
        [] eye(4); ... % Base
        collisionCylinder(dimensionArray(2,1), dimensionArray(2,2)) axang2tform([0 1 0 pi/2])*trvec2tform([0 0 dimensionArray(2,2)/2]); ... % link1
        collisionCylinder(dimensionArray(3,1), dimensionArray(3,2)) axang2tform([0 1 0 pi/2])*trvec2tform([0 0 dimensionArray(3,2)/2]); ... % link2
        collisionCylinder(dimensionArray(4,1), dimensionArray(4,2)) axang2tform([0 1 0 pi/2])*trvec2tform([0 0 dimensionArray(4,2)/2]); ... % link3
        collisionCylinder(dimensionArray(5,1), dimensionArray(5,2)) axang2tform([0 1 0 pi/2])*trvec2tform([0 0 dimensionArray(5,2)/2]); ... % link4    
        [] eye(4); ... % endEffector
    };
end
