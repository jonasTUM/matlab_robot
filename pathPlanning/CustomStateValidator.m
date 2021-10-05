%   This class defines a template for creating a custom state validation definition
%   that can be used by the sampling-based path planners like plannerRRT and
%   plannerRRTStar. The state validator allows you to validate states and
%   motions between states.
%
%   To access documentation for how to define a state space, enter the following
%   at the MATLAB command prompt:
%
%    >> doc nav.StateValidator
%
%   For a concrete implementation of the same interface, see the following
%   nav.StateValidator class:
%
%    >> edit validatorOccupancyMap
%
%
%   To use this custom state validator for path planning, follow the steps
%   outlined below and complete the class definition. Then, save this
%   file somewhere on the MATLAB path. You can add a folder to the
%   path using the ADDPATH function.

%   Copyright 2019 The MathWorks, Inc.


classdef CustomStateValidator < nav.StateValidator & ...
        matlabshared.planning.internal.EnforceScalarHandle
    
    %---------------------------------------------------------------------
    % Step 1: Define properties to be used by your state space. These are
    % user-defined properties.
    properties
        % ValidationDistance, defines the longest valid motion distance
        % without collision checks.
        ValidationDistance
        
        % Planning Problem.
        planProb       
    end
    
    %----------------------------------------------------------------------
    % Step 2: Define functions used for managing your state validator.
    methods
        % a) Use the constructor to set the name of the state space, the
        %    number of state variables, and to define its boundaries.
        %
        function obj = CustomStateValidator(space)
            
            narginchk(0,1)
            
            if nargin == 0
                space = stateSpaceSE2;
            end
            
            % The state space object is validated in the StateValidator base class
            obj@nav.StateValidator(space);
            
            %--------------------------------------------------------------
            % Place your code here
            %--------------------------------------------------------------
        end
        
        % b) Define how the object is being copied (a new object is created
        %    from the current one). You have to customize this function
        %    if you define your own properties or special constructor.
        %
        %    For more help, see
        %    >> doc nav.StateValidator/copy
        %
        function copyObj = copy(obj)
            
            % Default behavior: Create a new object of the same type with no arguments.
            copyObj = feval(class(obj), obj.StateSpace);
            
            %--------------------------------------------------------------
            % Place your code here
            %--------------------------------------------------------------
        end
        
        % c) Define how a given state is validated. The STATE input can be a
        %    single state (row) or multiple states (one state per row).
        %    You have to customize this function if you want to have
        %    special validation behavior, for example to check for
        %    collisions with obstacles.
        %
        %    For more help, see
        %    >> doc nav.StateValidator/isStateValid
        %
        function isValid = isStateValid(obj, state)
            
            narginchk(2,2);
            
            nav.internal.validation.validateStateMatrix(state, nan, obj.StateSpace.NumStateVariables, ...
                "isStateValid", "state");
            
            % Default behavior: Verify that state is within bounds
            bounds = obj.StateSpace.StateBounds';
            state = round(state,5);
            inBounds = state >= bounds(1,:) & state <= bounds(2,:);
            isValid = all(inBounds, 2);
            if(~isValid)
                return;
            end
            
            % Verify that state is not in collision.
            [isInCollision, ~, ~, ~, ~] = checkRobotConfiguration(obj.planProb, state');
            if(isInCollision)
                isValid = false;
                return;
            else
                isValid = true;
            end
        end
        
        % d) Define how a motion between states is validated.
        %
        %    For more help, see
        %    >> doc nav.StateValidator/isMotionValid
        %
        function [isValid, lastValid] = isMotionValid(obj, state1, state2)
            dist = obj.StateSpace.distance(state1, state2);
            interval = obj.ValidationDistance/dist;
            interpStates = obj.StateSpace.interpolate(state1, state2, [0:interval:1 1]);
            isValid = true;
            for i = 1: size(interpStates,1)
                interpSt = interpStates(i,:);
                if ~obj.isStateValid(interpSt)
                    isValid = false; 
                    break;
                end
            end
            lastValid = inf;
        end
        
    end
end
