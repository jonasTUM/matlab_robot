close all;
clear;
clc;
addpath(genpath('./'));

%% Setup the planning problem.
planningProblem.globVar = loadGlobalVariables();
planningProblem.collWorld = loadEnvironment();
planningProblem.rob = loadRobot();
planningProblem.collRob = loadCollisionRobot(planningProblem.rob);

%% Define motion query, i.e. the start configuration and the goal pose.
startConfig = planningProblem.globVar.desiredHomeConfiguration;
goalPose = trvec2tform([0.4, -0.5, 0.0])*axang2tform([0 0 1 -pi/2]);
ik = inverseKinematics('RigidBodyTree',planningProblem.rob);
[goalConfig,~] = ik(planningProblem.globVar.endEffector,goalPose,[1 1 1 1 1 1],startConfig);
    
%% Setup the RRT path planner and solve the motion query.
disp('main.m: Global Planner...');
% State space of the planning domain.
ss = CustomConfigurationSpace(planningProblem.globVar.qMin, planningProblem.globVar.qMax);
% State validator for collision checks.
sv = CustomStateValidator(ss);
sv.planProb = planningProblem;
sv.ValidationDistance = planningProblem.globVar.maxCollisionFreeConfigStep;  
% Setup RRT planner.
planner = plannerRRT(ss,sv);
planner.GoalBias = planningProblem.globVar.goalBias;  
planner.GoalReachedFcn = @(planner, goalState, newState) (planner.StateSpace.distance(newState, goalState) < 0.01);
planner.MaxConnectionDistance = planningProblem.globVar.connectionDistance;
rng(0,'twister'); % For repeatable result.

%% Solve/Load motion query.
%[pthObj,solnInfo] = planner.plan(startConfig',goalConfig');
%path = pthObj.States';
load('PlanningDomain.mat');

%% Path Postprocessor.
numIteration = 60;
pathContainer = cell(numIteration,1);
disp('main.m: Path Postprocessor...');
discretizedPath = discretizePath(path, planningProblem);
%[optimizedPath] = shortCutting(discretizedPath,planningProblem);
%[optimizedPath] = pathLengthOptimization(planningProblem, optimizedPath);
[pathContainer{1,1}, bubbles] = elasticBands(path, planningProblem);
optimizedPath = pathContainer{1,1};
for ii = 2 : numIteration
    [pathContainer{ii,1}, bubbles] = elasticBands(optimizedPath, planningProblem);
    optimizedPath = pathContainer{ii,1};
end

%% Trajectory Generator based on cubic splines.
disp('main.m: Trajecotry Generator...');
t = linspace(0,planningProblem.globVar.motionDuration,size(optimizedPath,2));
tSamples = 0 : planningProblem.globVar.timeStep : planningProblem.globVar.motionDuration;        
[q,~,~] = cubicpolytraj(optimizedPath,t,tSamples);

%% Visualization.
disp('main.m: Display Motion...');
robotFigureHandle = generateRobotFigure(planningProblem);
% Plot path waypoints.
pathPosition = config2position(path', planningProblem);
plot3(pathPosition(:,1), pathPosition(:,2), pathPosition(:,3),'x', 'Linewidth',1, 'Color', [0 0.396 0.741]);
plot3(pathPosition(:,1), pathPosition(:,2), pathPosition(:,3),'-', 'Linewidth',1, 'Color', [0 0.396 0.741]);
pathPosition = config2position(optimizedPath', planningProblem);
plot3(pathPosition(:,1), pathPosition(:,2), pathPosition(:,3),'x', 'Linewidth',1, 'Color', [0.635 0.678 0]);
plot3(pathPosition(:,1), pathPosition(:,2), pathPosition(:,3),'-', 'Linewidth',1, 'Color', [0.635 0.678 0]);
for ii = 1 : numIteration
    currentPath = pathContainer{ii,1};
    pathPosition = config2position(currentPath', planningProblem);
    plot3(pathPosition(:,1), pathPosition(:,2), pathPosition(:,3),'x', 'Linewidth',1, 'Color', [0.635 0.678 0]);
    plot3(pathPosition(:,1), pathPosition(:,2), pathPosition(:,3),'-', 'Linewidth',1, 'Color', [0.635 0.678 0]);
end
% Plot Bubbles.
for ii = 1 : size(bubbles,2)
    r = bubbles{1,ii};
    center = bubbles{2,ii};
    circle(center(1),center(2),r);
end
disp('main.m: Done.');

%% -------------------------------------------------------------------
% Plot circle.
% -------------------------------------------------------------------
function h = circle(x,y,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit);
    hold off
end
