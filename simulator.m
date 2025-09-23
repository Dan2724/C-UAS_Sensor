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
            sensed = [];
            UASPos = [obj.UAS.entrance(1), obj.UAS.entrance(2)]; % This matrix tracks all current and previous UAS positions
            UASTarget = [obj.UAS.target(1), obj.UAS.target(2)]; % This matrix tracks all UAS targets

            if obj.animate == true
                obj.map.startAnimation(obj.AOR, obj.assets, obj.NFZs, obj.sensors);
            end

            while eventExitBounds == 0
                if obj.animate
                    pause(obj.dt/5)
                end
                if obj.UAS.mode == 'Linear'
                    obj.UAS.linearMotion(UASPos(1, 1),UASPos(1, 2),UASPos(end, 1),UASPos(end, 2),UASTarget(1),UASTarget(2),obj.dt);
                elseif obj.UAS.mode == 'Search'
                    obj.UAS.searchMotion(UASPos(1, 1),UASPos(1, 2),UASPos(end, 1),UASPos(end, 2),UASTarget(1),UASTarget(2),obj.dt,obj.assets);
                end

                UASPos = cat(1, UASPos, [obj.UAS.position.xPos, obj.UAS.position.yPos]);

                % Determine state based on object collision
                [eventSensor] = obj.checkSensorCollision(UASPos(end, :));
                [eventAsset,  asset] = obj.checkAssetCollision(UASPos(end, :), obj.UAS.speed*obj.dt);
                [eventNFZ] = obj.checkNFZCollision(UASPos(end, :));
                [eventExitBounds] = obj.checkOutOfBounds(UASPos(end, :), obj.map.size);

                if eventSensor == 1 % UAV sensed
                    sensed = cat(1, sensed, UASPos(end, :));
                    if obj.animate
                        obj.map.updateUASAnimation(UASPos)
                        obj.map.updateSensedLocations(sensed)
                    end
                    result = 0;
                elseif eventAsset == 1 % Asset attacked
                    if obj.animate
                        obj.map.animateAssetDestroyed(obj.assets(asset).location);
                        obj.map.updateUASAnimation(UASPos)
                    end
                    result = 1;
                elseif eventNFZ == 1 % UAV entered NFZ
                    if obj.animate
                        obj.map.animateUASDestroyed(UASPos(end, :))
                        obj.map.updateUASAnimation(UASPos)
                    end
                    result = 2;
                    break
                else % UAV safe
                    if obj.animate
                        obj.map.updateUASAnimation(UASPos)
                    end
                    result = 0;
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
end