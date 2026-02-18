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
        turnRadius
    end

    methods
        function obj = UAS(speed, entrance, target, mode, options)
            arguments
                speed
                entrance
                target
                mode
                options.altitude   = 0
                options.turnRadius = 5  % metres
            end
            obj.speed      = speed;
            obj.altitude   = options.altitude;
            obj.position   = [entrance(1), entrance(2), options.altitude];
            obj.target     = target;
            obj.mode       = mode;
            obj.tempSpeed  = speed;
            obj.turnRadius = options.turnRadius;

            dir2D = obj.target(1:2) - obj.position(1:2);
            obj.targetUnitVector = [dir2D/norm(dir2D), 0];
            obj.active = true;

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

        function hybridAStarMotion(obj, time, tick, costMap)
            if ~obj.active; return; end

            if isempty(obj.planner)
                ss = stateSpaceSE2;
                ss.StateBounds = [costMap.XWorldLimits; costMap.YWorldLimits; -pi pi];
                sv = validatorOccupancyMap(ss);
                sv.Map = costMap;

                cellSize   = 1 / costMap.Resolution;
                primLen    = sqrt(2) * cellSize;      % minimum allowed value
                interpDist = primLen * 0.9;           % safely below MotionPrimitiveLength

                obj.planner = plannerHybridAStar(sv, ...
                    'MinTurningRadius',      obj.turnRadius, ...
                    'MotionPrimitiveLength', primLen, ...
                    'InterpolationDistance', interpDist);

                goalHeading = atan2(obj.target(2) - obj.position(2), ...
                                    obj.target(1) - obj.position(1));
                try
                    refPath = plan(obj.planner, ...
                        [obj.position(1:2), obj.heading], ...
                        [obj.target(1:2),   goalHeading]);
                    obj.pathPoints   = refPath.States(:, 1:2);
                    obj.pathHeadings = refPath.States(:, 3);
                catch
                    obj.pathPoints   = [];
                    obj.pathHeadings = [];
                end
                obj.tickOffset = tick - 1;
            end

            if isempty(obj.pathPoints)
                obj.position = obj.position + obj.speed * time * obj.targetUnitVector;
                return;
            end

            idx = tick - obj.tickOffset;

            if idx >= 1 && idx <= size(obj.pathPoints, 1)
                pose = obj.pathPoints(idx, :);
                obj.position = [pose(1), pose(2), obj.position(3)];
                obj.heading  = obj.pathHeadings(idx);
                obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];

            elseif idx > size(obj.pathPoints, 1)
                % Replan toward nearest map boundary, clamped 1 cell inward
                posXY  = obj.position(1:2);
                xl     = costMap.XWorldLimits;
                yl     = costMap.YWorldLimits;
                margin = 1 / costMap.Resolution;
                posEsc = [xl(1)+margin, posXY(2);
                          posXY(1),     yl(1)+margin;
                          xl(2)-margin, posXY(2);
                          posXY(1),     yl(2)-margin];
                [~, Iesc]  = min(sum((posEsc - posXY).^2, 2));
                obj.target = posEsc(Iesc, :);

                goalHeading = atan2(obj.target(2) - posXY(2), ...
                                    obj.target(1) - posXY(1));
                try
                    refPath = plan(obj.planner, ...
                        [posXY, obj.heading], ...
                        [obj.target, goalHeading]);
                    obj.pathPoints   = refPath.States(:, 1:2);
                    obj.pathHeadings = refPath.States(:, 3);
                catch
                    obj.pathPoints   = [];
                    obj.pathHeadings = [];
                    obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];
                    obj.position = obj.position + obj.speed * time * obj.targetUnitVector;
                    return;
                end
                obj.tickOffset = tick - 1;

                pose = obj.pathPoints(1, :);
                obj.position = [pose(1), pose(2), obj.position(3)];
                obj.heading  = obj.pathHeadings(1);
                obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];
            end
        end

        function searchMotion(obj, time, assets, destroyedAssets, NFZs)
            if ~obj.active; return; end

            obj.range = 20;
            obj.obstacles.NFZs = NFZs;
            obj.obstacles.assets = assets;
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
                    assetDistance(n) = norm(obj.position(1:2) - currentAssets(n).location(1:2));
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
            if isinterior(obj.obstacles.NFZs, obj.position + obj.targetUnitVector*obj.range)
                angle = linspace(-pi/4, pi/4, 100);
                options = zeros(100, 2);
                for n = 1:length(angle)
                    check = obj.position' + [cos(-angle(n)) -sin(-angle(n)); sin(-angle(n)) cos(-angle(n))]*obj.targetUnitVector'*obj.range;
                    options(n, :) = check';
                end
                crash = find(isinterior(obj.obstacles.NFZs, options) == true); %#ok<NASGU>
            end
        end
    end
end