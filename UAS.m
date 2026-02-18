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
            end
            obj.speed      = speed;
            obj.altitude   = options.altitude;
            obj.position   = [entrance(1), entrance(2), options.altitude];
            obj.target     = target;
            obj.mode       = mode;
            obj.turnRadius = options.turnRadius;

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

                % ValidationDistance: check every 0.25 m along each motion
                % primitive so no occupied cell can be skipped
                sv.ValidationDistance = 0.25;

                % MotionPrimitiveLength: 3 m gives the planner enough
                % resolution to thread through gaps while keeping search
                % tractable on a 100x100 m map.
                primLen    = 3.0;
                interpDist = 0.25;   % interpolation finer than validation distance

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
                catch ME
                    warning('HA* planning failed: %s', ME.message);
                    obj.pathPoints   = [];
                    obj.pathHeadings = [];
                end
                obj.tickOffset = tick - 1;
            end

            % If planning failed, hold position — do NOT fly through NFZs
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
                % Path exhausted — replan toward nearest map boundary
                posXY  = obj.position(1:2);
                xl     = costMap.XWorldLimits;
                yl     = costMap.YWorldLimits;
                margin = 2.0;   % stay 2 m inside world limits
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
                    % Hold position if replan also fails
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