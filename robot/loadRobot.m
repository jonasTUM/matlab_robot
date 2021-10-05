function [robot] = loadRobot()
%LOADROBOT Loads the robot model.
    % Length of the links in [m].
    L1 = 0.5;
    L2 = 0.5;
    L3 = 0.5;
    L4 = 0.5;    
    % Set up 4 DoF robot.
    robot = rigidBodyTree('DataFormat','column','MaxNumBodies',5);
    
    body = rigidBody('link1');
    joint = rigidBodyJoint('joint1', 'revolute');
    setFixedTransform(joint,trvec2tform([0 0 0]));
    joint.JointAxis = [0 0 1];
    body.Joint = joint;
    addBody(robot, body, 'base');
    
    body = rigidBody('link2');
    joint = rigidBodyJoint('joint2','revolute');
    setFixedTransform(joint, trvec2tform([L1,0,0]));
    joint.JointAxis = [0 0 1];
    body.Joint = joint;
    addBody(robot, body, 'link1');
    
    body = rigidBody('link3');
    joint = rigidBodyJoint('joint3','revolute');
    setFixedTransform(joint, trvec2tform([L2,0,0]));
    joint.JointAxis = [0 0 1];
    body.Joint = joint;
    addBody(robot, body, 'link2');
    
    body = rigidBody('link4');
    joint = rigidBodyJoint('joint4','revolute');
    setFixedTransform(joint, trvec2tform([L3,0,0]));
    joint.JointAxis = [0 0 1];
    body.Joint = joint;
    addBody(robot, body, 'link3');    

    body = rigidBody('endEffector');
    body.Mass = 0.0;
    joint = rigidBodyJoint('fix1','fixed');
    setFixedTransform(joint, trvec2tform([L4, 0, 0]));
    body.Joint = joint;
    addBody(robot, body, 'link4');
end
