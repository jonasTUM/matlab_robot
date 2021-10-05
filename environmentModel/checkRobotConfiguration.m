function [isInCollision, selfCollisionPairIdx, worldCollisionPairIdx, globalSepdist, globalWitnessPts, allSepDist, allWitnessPts] = checkRobotConfiguration(planProb, config)
% This function is based on MATLAB's exampleHelperManipCheckCollisions.
% Furhter functionality is included like the allowed collision matrix and
% the distance informatio during the checkCollisions() call.

%exampleHelperManipCheckSelfCollisions Check for collisions between robot bodies
%   This example accepts a rigidBodyTree object, TREE, and COLLISIONARRAY,
%   a cell array of collision meshes and the relationship of their origin
%   to the robot. For a given robot configuration, CONFIG, the function
%   checks whether the robot is in collision with itself. If
%   planProb.globVar.isExhaustiveChecking is set to FALSE, the function will exit as soon as
%   a collision is found. The elements of COLLISIONARRAY are ordered so
%   that the their indices correspond to the indices of the rigid bodies in
%   the vector [tree.Base tree.Bodies], where the first row corresponds to
%   the base of the rigidBodyTree, the second element corresponds to
%   planProb.rob.Bodies{1}, the third to planProb.rob.Bodies{2}, etc. The function outputs
%   a flag indicating whether collision is detected, ISINCOLLISION, and an
%   Mx2 matrix of collision body indices that indicate which objects in the
%   COLLISIONARRAY are in collision with each other.

%   Copyright 2019 The MathWorks, Inc.

    % Basic validation
    narginchk(2,2);
    validateattributes(planProb.rob, {'robotics.RigidBodyTree'}, {'nonempty'}, 'exampleHelperManipSelfCollisions', 'planProb.rob');
    validateattributes(planProb.collRob, {'cell'}, {'nonempty','nrows',planProb.rob.NumBodies+1, 'ncols', 2}, 'exampleHelperManipSelfCollisions', 'planProb.rob');
    validateattributes(config, {'double'}, {'nonempty','vector','nrows',length(planProb.rob.homeConfiguration)}, 'exampleHelperManipSelfCollisions', 'config');
    validateattributes(planProb.globVar.isExhaustiveChecking, {'logical'}, {'nonempty','scalar'}, 'exampleHelperManipSelfCollisions', 'planProb.globVar.isExhaustiveChecking');

    % Initialize the outputs
    isInCollision = false;
    % If the robot is in collision the collsion pair indices are returned.
    selfCollisionPairIdx = [];
    worldCollisionPairIdx = [];
    % Initialize key parameters
    planProb.rob.DataFormat = 'column';
    robotBodies = [{planProb.rob.Base} planProb.rob.Bodies];
    % If the robot is not in collision the closest points and distance are
    % returned.
    globalSepdist = inf;
    globalWitnessPts = zeros(3,2);
    numRobotBodies = numel(robotBodies) - 2; % No Base; No Link
    numCollBodies = numel(planProb.collWorld);
    allSepDist = inf(1, numRobotBodies, numCollBodies);
    allWitnessPts = zeros(3, 2, numRobotBodies, numCollBodies);
    
    % Rather than calling getTransform at each loop, populate a transform
    % tree, which is a cell array of all body transforms with respect to
    % the base frame
    transformTree = cell(numel(robotBodies),1); 
    
    % For the base, this is the identity
    transformTree{1} = eye(4);
    for i = 1:numel(robotBodies)
        transformTree{i} = getTransform(planProb.rob, config, robotBodies{i}.Name);
    end
    
    % Iterate over all bodies and check self collisions and collision with
    % the environment.
    for j = 1:numel(robotBodies)
        % Since collisionPairIdx order doesn't matter, only check every
        % pair of bodies once
        for k = j:numel(robotBodies)
            % Ensure that both bodies have associated collision objects and
            % that they are not the same object.
            if j ~= k && ~isempty(planProb.collRob{j,1}) && ~isempty(planProb.collRob{k,1})
                % Get the collision object pose from the associated
                % rigid body tree. The updated pose is the product of
                % the associated rigid body tree pose and transform
                % that relates the collision object origin to the rigid
                % body position (measured from the parent joint).
                planProb.collRob{j,1}.Pose = transformTree{j}*planProb.collRob{j,2};
                planProb.collRob{k,1}.Pose = transformTree{k}*planProb.collRob{k,2};
                % Don't check collision with neighbors
                if j ~= k+1 && j ~= k-1 % not checking with self and neighbors
                    % Check for local collision and update the overall
                    % collision status flag
                    localCollisionStatus = checkCollision(planProb.collRob{j}, planProb.collRob{k});
                    isInCollision = isInCollision || localCollisionStatus;
                    % If a collision is detected, update the matrix of bodies
                    % in collision
                    if localCollisionStatus
                        selfCollisionPairIdx = [selfCollisionPairIdx; [j k]]; %#ok<AGROW>
                        if ~planProb.globVar.isExhaustiveChecking
                            return;
                        end
                    end
                end
            end
        end
        
        % Check collisions with all the world collision objects
        for m = 1:numel(planProb.collWorld)
            if ~isempty(planProb.collRob{j,1})
                % The body poses have already been updated in the previous
                % for-loop, and the world collision objects don't move, so
                % there's no need to update poses

                % Check for local collision and update the overall
                % collision status flag
                [localCollisionStatus,sepdist,witnesspts] = checkCollision(planProb.collRob{j}, planProb.collWorld{m}{1});
                % If a collision occured, do not store the distance
                % information.
                if localCollisionStatus
                    allSepDist(:,j-1,m) = 0; % Here, we use j-1, as we assume that the 1st Robotarm is not valid/base.
                    sepdist = 0;
                else
                    allWitnessPts(:,:,j-1,m) = witnesspts; 
                    allSepDist(:,j-1,m) = sepdist;
                end
                if sepdist < globalSepdist && sepdist > 0.0
                    globalSepdist = sepdist;
                    globalWitnessPts = witnesspts;
                end

                isInCollision = isInCollision || localCollisionStatus;
                % If a collision is detected, update the matrix of bodies
                % in collision
                if localCollisionStatus
                    worldCollisionPairIdx = [worldCollisionPairIdx; [j m]];%#ok<AGROW>
                    if ~planProb.globVar.isExhaustiveChecking
                        return;
                    end
                end
            end
        end
    end
end
