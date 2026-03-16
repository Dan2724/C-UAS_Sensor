classdef UAS < handle
    properties
        speed
        target
        mode
        position
        targetUnitVector
        active
        altitude

        % Hybrid A* properties
        planner
        pathPoints
        pathHeadings
        heading
        tickOffset
        turnRadius
        
        % NEW: egress properties
        egressPoint
        headingToEgress
    end

    methods
        function obj = UAS(speed, entrance, target, mode, options)
            arguments
                speed
                entrance
                target
                mode
                options.altitude   = 0
                options.turnRadius = 5
                options.egressPoint = []  % NEW
            end
            obj.speed      = speed;
            obj.altitude   = options.altitude;
            obj.position   = [entrance(1), entrance(2), options.altitude];
            obj.target     = target;
            obj.mode       = mode;
            obj.turnRadius = options.turnRadius;
            obj.egressPoint = options.egressPoint;  % NEW
            obj.headingToEgress = false;  % NEW

            dir2D = obj.target(1:2) - obj.position(1:2);
            obj.targetUnitVector = [dir2D/norm(dir2D), 0];
            obj.active  = true;

            obj.planner      = [];
            obj.pathPoints   = [];
            obj.pathHeadings = [];
            obj.heading      = atan2(dir2D(2), dir2D(1));
            obj.tickOffset   = 0;
        end

        function hybridAStarMotion(obj, time, tick, costMap)
            if ~obj.active; return; end

            % Plan once on first call
            if isempty(obj.planner)
                ss = stateSpaceSE2;
                ss.StateBounds = [costMap.XWorldLimits; costMap.YWorldLimits; -pi pi];
                sv = validatorOccupancyMap(ss);
                sv.Map = costMap;

                sv.ValidationDistance = 0.25;

                cellSize = 1 / costMap.Resolution;
                minLen   = sqrt(2) * cellSize + 0.01;
                maxLen   = (pi/2) * obj.turnRadius;
                primLen  = max(minLen, 0.75 * maxLen);
                interpDist = min(0.25, primLen * 0.5);

                obj.planner = plannerHybridAStar(sv);

                obj.planner.MinTurningRadius      = obj.turnRadius;
                obj.planner.MotionPrimitiveLength  = primLen;
                obj.planner.InterpolationDistance  = interpDist;

                goalHeading = atan2(obj.target(2) - obj.position(2), ...
                                    obj.target(1) - obj.position(1));
                try
                    refPath = plan(obj.planner, ...
                        [obj.position(1:2), obj.heading], ...
                        [obj.target(1:2),   goalHeading]);
                    obj.pathPoints   = refPath.States(:, 1:2);
                    obj.pathHeadings = refPath.States(:, 3);
                catch ME
                    warning('HA* planning failed: %s', ME.message);
                    obj.pathPoints   = [];
                    obj.pathHeadings = [];
                end
                obj.tickOffset = tick - 1;
            end

            if isempty(obj.pathPoints)
                return;
            end

            idx = tick - obj.tickOffset;

            if idx >= 1 && idx <= size(obj.pathPoints, 1)
                pose = obj.pathPoints(idx, :);
                obj.position = [pose(1), pose(2), obj.position(3)];
                obj.heading  = obj.pathHeadings(idx);
                obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];

            elseif idx > size(obj.pathPoints, 1)
                % Path exhausted — replan to egress if available, else map boundary
                posXY = obj.position(1:2);
                
                if ~isempty(obj.egressPoint) && ~obj.headingToEgress
                    % Switch to egress point
                    obj.target = obj.egressPoint;
                    obj.headingToEgress = true;
                else
                    % Original behavior: go to nearest boundary
                    xl     = costMap.XWorldLimits;
                    yl     = costMap.YWorldLimits;
                    margin = 2.0;
                    posEsc = [xl(1)+margin, posXY(2);
                              posXY(1),     yl(1)+margin;
                              xl(2)-margin, posXY(2);
                              posXY(1),     yl(2)-margin];
                    [~, Iesc]  = min(sum((posEsc - posXY).^2, 2));
                    obj.target = posEsc(Iesc, :);
                end

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
                    return;
                end
                obj.tickOffset = tick - 1;

                pose = obj.pathPoints(1, :);
                obj.position = [pose(1), pose(2), obj.position(3)];
                obj.heading  = obj.pathHeadings(1);
                obj.targetUnitVector = [cos(obj.heading), sin(obj.heading), 0];
            end
        end
    end
end