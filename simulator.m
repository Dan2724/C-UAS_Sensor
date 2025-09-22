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

            xPos0 = obj.UAS.entrance(1);
            yPos0 = obj.UAS.entrance(2);

            xtarget = obj.UAS.target(1);
            ytarget = obj.UAS.target(2);

            xPos = xPos0;
            yPos = yPos0;
            targetUnitVector = (obj.UAS.target - obj.UAS.entrance) / norm(obj.UAS.target - obj.UAS.entrance);

            if obj.animate == true
                obj.map.displayMap

                % Plot AOR
                plot(obj.AOR, 'FaceColor', 'white', 'FaceAlpha', 0.05)

                % Plot assets
                for i = 1:length(obj.assets)
                    plot(obj.assets(i).location(1), obj.assets(i).location(2), 'Marker', 'square', 'Color', 'b', 'MarkerSize', 10, 'LineWidth', 5)
                end

                % Plot NFZs
                if isempty(obj.NFZs) == 0
                    for i = 1:length(obj.NFZs)
                        plot(obj.NFZs(i), 'FaceColor', 'm', 'FaceAlpha', 0.2)
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
                    plot(x, y, 'o', 'Color', 'b')
                end

                %
                xlim([0,obj.map.size.horiz])
                ylim([0,obj.map.size.vert])

                % Set up animation
                UAVTrail = plot(NaN, NaN, 'Color', 'r');
                UAVHead = plot(NaN, NaN, 'Color', 'r', 'Marker', '^');
                UAVx = []; UAVy = [];
            end



            while xPos <= obj.map.size.horiz && yPos <= obj.map.size.vert

                if obj.UAS.mode == 'Linear'
                    obj.UAS.linearMotion(xPos0,yPos0,xPos,yPos,xtarget,ytarget,obj.time,targetUnitVector);
                elseif obj.UAS.mode == 'Search'
                    obj.UAS.searchMotion(xPos, yPos, xtarget, ytarget, obj.time);

                end
                xPos = obj.UAS.xPos;
                yPos = obj.UAS.yPos;
                
                % Determine state based on object collision
                pos = [xPos, yPos];
                [eventSensor] = obj.checkSensorCollision(pos);
                [eventAsset,  asset] = obj.checkAssetCollision(xPos, yPos, obj.UAS.speed*obj.dt);
                [eventNFZ] = obj.checkNFZCollision(xPos, yPos);
                
                if obj.animate
                    UAVx(end+1) = xPos;
                    UAVy(end+1) = yPos;
                end

                if eventSensor == 1 % UAV sensed
                    if obj.animate
                        obj.animateUAVDestroyed([xPos, yPos])
                        obj.updateAnimation(UAVTrail, UAVHead, UAVx, UAVy)
                    end
                    result = 1;
                    break
                elseif eventAsset == 1 % Asset attacked
                    if obj.animate
                        obj.animateAssetDestroyed([obj.assets(asset).location(1), obj.assets(asset).location(2)])
                        UAVx(end+1) = xtarget;
                        UAVy(end+1) = ytarget;
                        obj.updateAnimation(UAVTrail, UAVHead, UAVx, UAVy)
                    end
                    result = 0;
                    break
                elseif eventNFZ == 1 % UAV entered NFZ
                    if obj.animate
                        obj.animateUAVDestroyed([xPos, yPos])
                        obj.updateAnimation(UAVTrail, UAVHead, UAVx, UAVy)
                    end
                    result = 1;
                    break
                else % UAV safe
                    if obj.animate
                        pause(obj.dt)
                        obj.updateAnimation(UAVTrail, UAVHead, UAVx, UAVy)
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

        function [event, asset] = checkAssetCollision(obj, xPos, yPos, speed)
            % Asset collision
            for i = 1:length(obj.assets)
                deltaAsset = norm(obj.assets(i).location - [xPos, yPos]);
                if deltaAsset <= speed
                    event = 1;
                    asset = i;
                    return
                else
                    event = 0; asset = 0;
                end
            end
        end

        function [event, NFZ] = checkNFZCollision(obj, xPos, yPos)
            % NFZ collision
            event = 0; NFZ = 0;
            if isempty(obj.NFZs) == 0
                for i = 1:length(obj.NFZs)
                    if isinterior(obj.NFZs(i), xPos, yPos) == 1
                        event = 1;
                        NFZ = i;
                        return
                    end
                end
            end
        end
    end

    methods(Static)
        % Main animation update (rename to updateUAVAnimation?)
        function updateAnimation(UAVTrail, UAVHead, UAVx, UAVy)
            set(UAVTrail, 'XData', UAVx, 'YData', UAVy)
            set(UAVHead, 'XData', UAVx(end), 'YData', UAVy(end))
            drawnow;
        end
        
        function animateAssetDestroyed(target)
            plot(target(1), target(2), 'Marker', 'x', 'Color', 'r', 'MarkerSize', 12)
            drawnow;
        end

        function animateUAVDestroyed(position)
            plot(position(1), position(2), 'Marker', 'x', 'Color', 'g', 'MarkerSize', 12)
            drawnow;
        end
    end
end