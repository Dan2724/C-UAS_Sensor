classdef map < handle
    properties
        size
        UASTrail
        UASHead
        UASSensed
        UASDestroyed
        assetDestroyed
        assets
        NFZs
        timeBox
    end
    methods
        function obj = map(vertical, horizontal)
            obj.size.vert  = vertical;
            obj.size.horiz = horizontal;
        end

        function displayMap(obj)
            hold on
            xlim([0, obj.size.horiz])
            ylim([0, obj.size.vert])
            grid on
            axis equal
            title("UAS Simulation")
            xlabel("X (m)")
            ylabel("Y (m)")
        end

        function startAnimation(obj, AOR, assets, NFZs, sensors, P, hideClock)
            obj.displayMap

            obj.UASTrail      = plot(NaN, NaN, 'Color', 'r', 'DisplayName', "UAS Trail");
            obj.UASHead       = plot(NaN, NaN, 'Color', 'r', 'Marker', '^', 'DisplayName', "UAS");
            obj.UASSensed     = plot(NaN, NaN, 'Color', 'y', 'Marker', 'square', 'LineStyle', 'none', 'DisplayName', "UAS Sensor Detection Point");
            obj.UASDestroyed  = plot(NaN, NaN, 'Marker', 'x', 'Color', 'g', 'MarkerSize', 12);
            obj.assetDestroyed = plot(NaN, NaN, 'Marker', 'x', 'Color', 'r', 'MarkerSize', 20, 'LineWidth', 2, 'DisplayName', "Asset Destroyed");

            axesChildren = get(gca, 'Children');
            axesMatch    = findobj(axesChildren, 'DisplayName', "AOR");

            if isempty(axesMatch)

                xVec = 0 : 1 : obj.size.horiz;
                yVec = 0 : 1 : obj.size.vert;

                % --- Draw each sensor's individual attenuated contour ---
                % Each sensor gets its own filled contour at the P=0.5 level
                % so you can see exactly how interference has shrunk its
                % effective footprint compared to an isolated sensor.
                colours = lines(length(sensors));
                for i = 1:length(sensors)
                    Pi = P_perSensor(sensors, i, obj.size);   % see local helper below
                    contourf(xVec, yVec, Pi, [0.5 0.5], ...
                        'FaceAlpha', 0.12, ...
                        'EdgeColor', colours(i,:), ...
                        'LineWidth', 1.2, ...
                        'DisplayName', sprintf('Sensor %d (p=0.5)', i));
                    % Sensor location dot
                    plot(sensors(i).location(1), sensors(i).location(2), '.', ...
                        'Color', colours(i,:), ...
                        'MarkerSize', 20, ...
                        'DisplayName', sprintf('Sensor %d', i));
                end

                % --- Combined probability field (all sensors summed) ---
                contourf(xVec, yVec, P, 0.1:0.05:0.9, ...
                    'FaceAlpha', 0.07, 'LineStyle', 'none');
                colorbar;

                % --- AOR ---
                plot(AOR, 'FaceColor', 'white', 'FaceAlpha', 0.05, 'DisplayName', "AOR");

                if hideClock == false
                    obj.timeBox = text(0.05*obj.size.vert, 0.95*obj.size.vert, ...
                        't: 0s', 'ColorMode', 'auto', 'EdgeColor', 'k');
                end

                % --- Assets ---
                for i = 1:length(assets)
                    obj.assets = plot(assets(i).location(1), assets(i).location(2), ...
                        'Marker', 'square', 'Color', 'g', 'MarkerSize', 10, ...
                        'LineWidth', 2, 'LineStyle', 'none', 'DisplayName', "Asset " + i);
                end

                % --- NFZs ---
                if ~isempty(NFZs)
                    for i = 1:length(NFZs)
                        obj.NFZs = plot(NFZs(i), 'FaceColor', 'y', 'FaceAlpha', 0.2, ...
                            'EdgeColor', 'y', 'DisplayName', "NFZ " + i);
                    end
                end
            end

            xlim([0, obj.size.horiz])
            ylim([0, obj.size.vert])
        end

        function updateUASAnimation(obj, UASPos)
            set(obj.UASTrail, 'XData', UASPos(:,1), 'YData', UASPos(:,2))
            set(obj.UASHead,  'XData', UASPos(end,1), 'YData', UASPos(end,2))
        end

        function updateSensedLocations(obj, sensedPos)
            set(obj.UASSensed, 'XData', sensedPos(:,1), 'YData', sensedPos(:,2))
        end

        function animateDestroyedAssets(obj, assets, destroyedAssets)
            XData = [];  YData = [];
            for i = 1:length(destroyedAssets)
                XData(1,i) = assets(destroyedAssets(i)).location(1);
                YData(1,i) = assets(destroyedAssets(i)).location(2);
            end
            set(obj.assetDestroyed, 'XData', XData, 'YData', YData)
        end

        function animateUASDestroyed(obj, position)
            set(obj.UASDestroyed, 'XData', position(1), 'YData', position(2))
        end

        function updateClock(obj, time)
            set(obj.timeBox, 'String', ['t: ', sprintf('%.2f', time), 's']);
        end

        function cleanAnimation(obj)
            obj.timeBox = [];
        end

        function wipeAnimation(obj)
            clf
            obj.timeBox = [];
        end
    end
end

% -------------------------------------------------------------------------
% Local helper: recompute the probability grid for a single sensor,
% passing the full sensor array so interference is included.
% This is only called during animation setup, not during the sim loop.
% -------------------------------------------------------------------------
function Pi = P_perSensor(sensors, idx, mapSize)
    [~, ~, Pi] = sensors(idx).createSensorContours(mapSize, polyshape.empty, sensors);
end