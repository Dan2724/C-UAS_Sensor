classdef simulator
    %SIMULATOR for C-UAS Sensor Placement
    %   This is the main simulator class for the UAS-Sensor Placement. This
    %   is where time will march, and from here all map updates are called.

    properties
        map
        AOR
        UAS
        sensors
        assets
        time
        dt
        animate
        NFZs
    end

    methods
        function obj = simulator(map, aor, uas, sensors, assets, dt, animate, nfzs)
            %SIMULATOR
            arguments
                map
                aor
                uas
                sensors
                assets
                dt
                animate
                nfzs polyshape = polyshape.empty % Exclude input if not needed
            end
            obj.map = map;
            obj.AOR = aor;
            obj.UAS = uas;
            obj.sensors = sensors;
            obj.assets = assets;
            obj.time = 0;
            obj.dt = dt;
            obj.animate = animate;
            obj.NFZs = nfzs;
        end

        function result = runSim(obj)
            result = 0;
            eventExitBounds = 0;
            sensedCount = 0;
            sensed = [];
            UASPos = [obj.UAS.entrance(1), obj.UAS.entrance(2)]; % This matrix tracks all current and previous UAS positions
            UASTarget = [obj.UAS.target(1), obj.UAS.target(2)]; % This matrix tracks all UAS targets

            targetUnitVector = (UASTarget - UASPos(1,:)) / norm(UASTarget - UASPos(1,:));

            if obj.animate == true
                obj.startAnimation(obj);
                UASTrail = plot(NaN, NaN, 'Color', 'r', 'DisplayName', "UAS Trail");
                UASHead = plot(NaN, NaN, 'Color', 'r', 'Marker', '^', 'DisplayName', "UAS");
                UASSensed = plot(NaN, NaN, 'Color', 'y', 'Marker', 'square', 'LineStyle', 'none', 'DisplayName', "UAS Sensor Detection Point");
            end

            while eventExitBounds == 0
                if obj.animate
                    pause(obj.dt/5)
                end
                if obj.UAS.mode == 'Linear'
                    obj.UAS.linearMotion(UASPos(1, 1),UASPos(1, 2),UASPos(end, 1),UASPos(end, 2),UASTarget(1),UASTarget(2),obj.time,targetUnitVector);
                elseif obj.UAS.mode == 'Search'
                    obj.UAS.searchMotion(UASPos(1, 1),UASPos(1, 2),UASPos(end, 1),UASPos(end, 2),UASTarget(1),UASTarget(2),obj.time,targetUnitVector,obj.assets);
                end

                UASPos = cat(1, UASPos, [obj.UAS.position.xPos, obj.UAS.position.yPos]);

                % Determine state based on object collision
                [eventSensor] = obj.checkSensorCollision(UASPos(end, :));
                [eventAsset,  asset] = obj.checkAssetCollision(UASPos(end, :), obj.UAS.speed*obj.dt);
                [eventNFZ] = obj.checkNFZCollision(UASPos(end, :));
                [eventExitBounds] = obj.checkOutOfBounds(UASPos(end, :), obj.map.size);

                if eventSensor == 1 % UAV sensed
                    sensed = cat(1, sensed, UASPos(end, :));
                    sensedCount = sensedCount + 1;
                    if obj.animate
                        obj.updateAnimation(UASTrail, UASHead, UASPos)
                        obj.updateSensedLocations(sensed, UASSensed)
                    end
                    if sensedCount == 10 % Determines how many tiems the UAS needs to be sensed to be "killed"
                        result = 1;
                        if obj.animate
                            obj.animateUAVDestroyed(UASPos(end, :))
                        end
                        break
                    end
                elseif eventAsset == 1 % Asset attacked
                    UASPos = cat(1, UASPos, obj.assets(asset).location);
                    if obj.animate
                        obj.animateAssetDestroyed(obj.assets(asset).location);
                        obj.updateAnimation(UASTrail, UASHead, UASPos)
                    end
                    result = 0;
                    break
                elseif eventNFZ == 1 % UAV entered NFZ
                    if obj.animate
                        obj.animateUAVDestroyed(UASPos(end, :))
                        obj.updateAnimation(UASTrail, UASHead, UASPos)
                    end
                    result = 1;
                    break
                else % UAV safe
                    sensedCount = 0;
                    if obj.animate
                        obj.updateAnimation(UASTrail, UASHead, UASPos)
                    end
                end

                obj.time = obj.time + obj.dt; % Progress time
            end
        end

        function [event, sensor] = checkSensorCollision(obj, pos)
            % Sensor collision detection
            % Inputs:
            %   pos - A 2-element vector representing the position to check
            % Outputs:
            %   event - 1 if a collision is detected, 0 otherwise
            %   sensor - Index of the sensor that detected the collision, or 0 if none

            event = 0; % Initialize event to no collision
            sensor = 0; % Initialize sensor index

            for i = 1:length(obj.sensors)
                r = [pos(1), pos(2)] - obj.sensors(i).location;
                if norm(r) <= obj.sensors(i).range
                    event = 1; % Collision detected
                    sensor = i; % Store the index of the colliding sensor
                    return; % Exit the function early
                end
            end
        end

        function [event, asset] = checkAssetCollision(obj, pos, deltaPos)
            % Asset collision
            for i = 1:length(obj.assets)
                deltaAssetPos = norm(obj.assets(i).location - [pos(1), pos(2)]);
                if deltaAssetPos <= deltaPos
                    event = 1;
                    asset = i;
                    return
                else
                    event = 0; asset = 0;
                end
            end
        end

        function [event, NFZ] = checkNFZCollision(obj, pos)
            % NFZ collision
            event = 0; NFZ = 0;
            if isempty(obj.NFZs) == 0
                for i = 1:length(obj.NFZs)
                    if isinterior(obj.NFZs(i), pos(1), pos(2)) == 1
                        event = 1;
                        NFZ = i;
                        return
                    end
                end
            end
        end

        function [event] = checkOutOfBounds(~, pos, size)
            if pos(1) < 0 || pos(1) > size.vert
                event = 1;
            elseif pos(2) < 0 || pos(2) > size.horiz
                event = 1;
            else
                event = 0;
            end
        end
    end

    methods(Static)
        % Initialize animation
        function startAnimation(obj)
            obj.map.displayMap

            % Determine if plot has already been initialized
            axesChildren = get(gca, 'Children');
            axesMatch = findobj(axesChildren, 'DisplayName', "AOR");


            if isempty(axesMatch)
                % Plot AOR
                plot(obj.AOR, 'FaceColor', 'white', 'FaceAlpha', 0.05, 'DisplayName', "AOR");

                % Plot assets
                for i = 1:length(obj.assets)
                    plot(obj.assets(i).location(1), obj.assets(i).location(2), 'Marker', 'square', 'Color', 'g', 'MarkerSize', 10, 'LineWidth', 2, 'LineStyle','none' , 'DisplayName', "Asset " + i);
                end



                % Plot NFZs
                if isempty(obj.NFZs) == 0
                    for i = 1:length(obj.NFZs)
                        plot(obj.NFZs(i), 'FaceColor', 'y', 'FaceAlpha', 0.2, 'EdgeColor', 'y', 'DisplayName', "NFZ " + i);
                    end
                end

                % Plot sensors
                for i = 1:length(obj.sensors)
                    x = obj.sensors(i).location(1);
                    y = obj.sensors(i).location(2);
                    r = obj.sensors(i).range;

                    rectangle('Position',[x-r, y-r, 2*r, 2*r], ...
                        'Curvature', [1 1], ...
                        'FaceColor', 'b', ...
                        'EdgeColor', 'b', ...
                        'FaceAlpha', 0.05)
                    plot(x, y, 'o', 'Color', 'b', 'DisplayName', "Sensor " + i)
                end

                %
            end
            xlim([0,obj.map.size.horiz])
            ylim([0,obj.map.size.vert])
        end

        % Main animation update (rename to updateUAVAnimation?)
        function updateAnimation(UASTrail, UASHead, UASPos)
            set(UASTrail, 'XData', UASPos(:, 1), 'YData', UASPos(:, 2))
            set(UASHead, 'XData', UASPos(end, 1), 'YData', UASPos(end, 2))
        end

        function updateSensedLocations(sensedPos, UASSensed)
            set(UASSensed, 'XData', sensedPos(:, 1), 'YData', sensedPos(:, 2))
        end

        function animateAssetDestroyed(target)
            plot(target(1), target(2), 'Marker', 'x', 'Color', 'r', 'MarkerSize', 20, 'LineWidth', 2, 'DisplayName', "Asset Destroyed")
        end

        function animateUAVDestroyed(position)
            plot(position(1), position(2), 'Marker', 'x', 'Color', 'g', 'MarkerSize', 12)
        end

    end
end