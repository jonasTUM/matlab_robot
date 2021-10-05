function [axisHandles] = plotCollisionRobot(rigidBodyTree, rigidBodyCollisionArray, config, axisHandle)
%PLOTCOLLISIONROBOT Plot collision objects using pose from
%rigidbodytree configuration.
    rbt = copy(rigidBodyTree);
    rbt.DataFormat = 'column';
    %rbt.show(config);
    bodies = [{rbt.Base} rbt.Bodies];
    axisHandles = cell(0,1);
    for i = 1:numel(bodies)
        % Get tree pose
        TForm = getTransform(rbt, config, bodies{i}.Name, rbt.Base.Name);
        % Get collision object information
        collisionObject = rigidBodyCollisionArray{i,1};
        collisionObjectPosition = rigidBodyCollisionArray{i,2};
        % Collision object position is a combination of the joint
        % position and the relative pose of the object to the
        % joint.
        if ~isempty(collisionObject)
            collisionObject.Pose = TForm*collisionObjectPosition;
            [~, axisHandles{end+1}]= collisionObject.show('Parent',axisHandle);
        end
    end
end

