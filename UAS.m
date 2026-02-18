classdef UAS < handle
    properties
        speed
        target
        mode
        position
        targetUnitVector
        range
        active
        obstacles
        destroyedAssets
        totalAssets
        tempSpeed
        altitude

        % Hybrid A* properties
        planner
        pathPoints
        pathHeadings
        heading
        tickOffset
    end

    methods
        function obj = UAS(speed, entrance, target, mode, altitude)
            arguments
                speed
                entrance
                target
                mode
                altitude = 0   % default altitude; not used in HybridAStar mode
            end
            obj.speed = speed;
            obj.altitude = altitude;
            obj.position = [entrance(1), entrance(2), altitude];
            obj.target = target;
            obj.mode = mode;
            obj.tempSpeed = speed;
            dir2D = obj.target(1:2) - obj.position(1:2);
            obj.targetUnitVector = [dir2D/norm(dir2D), 0]; % Z-component is 0
            obj.active = true; % Default to active

            % Initialize Hybrid A* state
            obj.planner      = [];
            obj.pathPoints   = [];
            obj.pathHeadings = [];
            obj.heading      = atan2(dir2D(2), dir2D(1));
            obj.tickOffset   = 0;
        end

        function linearMotion(obj, time)
            if obj.active
                obj.position = obj.position + obj.speed*time*obj.targetUnitVector;
            end
        end

        function hybridAStarMotion(obj, time, tick, turnRadius, costMap)
            if ~obj.active
                return;
            end

            % Build planner on first call
            if isempty(obj.planner)
                ss = stateSpaceSE2;
                ss.StateBounds = [costMap.XWorldLimits; costMap.YWorldLimits; -pi pi];
                sv = validatorOccupancyMap(ss);
                sv.Map = costMap;
                
                % Set MotionPrimitiveLength to a safe value within valid range
                % Must be > sqrt(2)*CellSize and <= 1.41372
                % For CellSize=0.1 (resolution=10): range is (0.1414, 1.41372]
                % Choose turnRadius as primitive length if in range, otherwise use safe default
                cellSize = 1 / costMap.Resolution;
                minPrimLength = sqrt(2) * cellSize;
                maxPrimLength = 1.41;  % Use 1.41 for safety margin below upper limit of 1.41372
                
                if turnRadius > minPrimLength && turnRadius <= maxPrimLength
                    motionPrimLength = turnRadius;
                else
                    % Use small margin above minimum if turnRadius is out of range
                    margin = 0.01;
                    motionPrimLength = max(minPrimLength + margin, min(turnRadius, maxPrimLength));
                end
                
                obj.planner = plannerHybridAStar(sv, ...
                    'MinTurningRadius', turnRadius, ...
                    'MotionPrimitiveLength', motionPrimLength, ...
                    'InterpolationDistance', obj.speed * time);

                % Plan initial path: start=[x y heading], goal=[tx ty heading_to_target]
                goalHeading = atan2(obj.target(2) - obj.position(2), ...
                                    obj.target(1) - obj.position(1));
                refPath = plan(obj.planner, ...
                    [obj.position(1:2), obj.heading], ...
                    [obj.target(1:2),   goalHeading]);
                obj.pathPoints   = refPath.States(:, 1:2);
                obj.pathHeadings = refPath.States(:, 3);
                obj.tickOffset   = tick - 1; % so that tick-tickOffset starts at 1
            end

            idx = tick - obj.tickOffset;

            if idx >= 1 && idx <= size(obj.pathPoints, 1)
                % Follow pre-planned path
                pose = obj.pathPoints(idx, :);
                obj.position = [pose(1), pose(2), obj.position(3)];
                obj.heading  = obj.pathHeadings(idx);
                % Keep targetUnitVector consistent for any code that reads it
                obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];

            elseif idx > size(obj.pathPoints, 1)
                % Reached end of planned path — replan toward nearest map boundary (escape)
                posXY = obj.position(1:2);
                posEsc = [costMap.XWorldLimits(1), posXY(2);  % left edge
                          posXY(1), costMap.YWorldLimits(1);  % bottom edge
                          costMap.XWorldLimits(2), posXY(2);  % right edge
                          posXY(1), costMap.YWorldLimits(2)]; % top edge
                [~, Iesc] = min(sum((posEsc - posXY).^2, 2));
                obj.target = posEsc(Iesc, :);

                goalHeading = atan2(obj.target(2) - posXY(2), ...
                                    obj.target(1) - posXY(1));
                refPath = plan(obj.planner, ...
                    [posXY, obj.heading], ...
                    [obj.target, goalHeading]);
                obj.pathPoints   = refPath.States(:, 1:2);
                obj.pathHeadings = refPath.States(:, 3);
                obj.tickOffset   = tick - 1;

                % Take first step of new path immediately
                pose = obj.pathPoints(1, :);
                obj.position = [pose(1), pose(2), obj.position(3)];
                obj.heading  = obj.pathHeadings(1);
                obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];
            end
        end

        function searchMotion(obj, time, assets, destroyedAssets, NFZs)
            if ~obj.active
                return;
            end
            
            obj.range = 20;
            obj.obstacles.NFZs = NFZs;
            obj.obstacles.assets = assets;
            
            % avoidNFZ logic needs to be fixed to actually turn the UAS
            obj.avoidNFZ(); 

            obj.totalAssets = length(obj.obstacles.assets);
            obj.destroyedAssets = destroyedAssets;

            currentAssets = obj.obstacles.assets;
            if ~isempty(obj.destroyedAssets)
                currentAssets(obj.destroyedAssets) = [];
            end

            if ~isempty(currentAssets)
                assetDistance = zeros(1, length(currentAssets));
                for n = 1:length(currentAssets)
                    dist2D = norm(obj.position(1:2) - currentAssets(n).location(1:2));
                    assetDistance(n) = dist2D;
                end
                [minDist, assetNumber] = min(assetDistance);

                if minDist <= obj.range
                    obj.assetFound(minDist, assetNumber, time, currentAssets);
                else
                    obj.position = obj.position + obj.speed*time*obj.targetUnitVector;
                end
            end
        end

        function assetFound(obj, assetDistance, assetNumber, time, currentAssets)
            turnRadius = assetDistance/2;
            
            assetLocation = [currentAssets(assetNumber).location, 0] - [obj.position(1:2), 0];
            
            tuv = [obj.targetUnitVector(1:2), 0];
            
            turnAngle = acos(dot(tuv, assetLocation)/(norm(tuv)*norm(assetLocation)));
            rotDir = cross(tuv, assetLocation);
            
            angleVelo = (sin(turnAngle)*obj.speed)/turnRadius;
            angle = angleVelo*time;

            if abs(turnAngle) > 0.1
                obj.turnMotion(angle, rotDir, time);
            else
                obj.position = obj.position + obj.tempSpeed*time*obj.targetUnitVector;
            end
        end

        function turnMotion(obj, angle, rotDir, time)
            if rotDir(3) < 0
                DCM = [cos(angle) -sin(angle); sin(angle) cos(angle)];
            elseif rotDir(3) > 0
                DCM = [cos(-angle) -sin(-angle); sin(-angle) cos(-angle)];
            else
                DCM = eye(2);
            end
            
            newVec2D = (DCM * obj.targetUnitVector(1:2)')';
            obj.targetUnitVector = [newVec2D, 0];
            
            obj.position = obj.position + obj.tempSpeed*time*obj.targetUnitVector;
        end

        function avoidNFZ(obj)
            % detects but does not yet turn
            if isinterior(obj.obstacles.NFZs, obj.position + obj.targetUnitVector*obj.range)
                angle = linspace(-pi/4, pi/4, 100);
                options = zeros(100, 2);
                for n = 1:length(angle)
                    check = obj.position' + [cos(-angle(n)) -sin(-angle(n)); sin(-angle(n)) cos(-angle(n))]*obj.targetUnitVector'*obj.range;
                    options(n, :) = check';
                end
                crash = find(isinterior(obj.obstacles.NFZs, options) == true);
            end
        end
    end
end