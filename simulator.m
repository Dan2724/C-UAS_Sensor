classdef simulator
    properties
        map
        AOR
        UAS
        UASPos_all
        sensors
        assets

        tick
        dt
        tps
        animate
        NFZs
        resetGraphics
        animationMultiplier
        hideClock
    end

    methods
        function obj = simulator(map, aor, uas, sensors, assets, options)
            arguments
                map, aor, uas, sensors, assets
                options.tps                 = 20
                options.animate             = true
                options.nfzs                = polyshape.empty
                options.resetGraphics       = true
                options.animationMultiplier = 1
                options.hideClock           = false
            end
            obj.map     = map;
            obj.AOR     = aor;
            obj.UAS     = uas;
            obj.sensors = sensors;
            obj.assets  = assets;

            obj.tick                 = 0;
            obj.tps                  = options.tps;
            obj.dt                   = 1 / obj.tps;
            obj.animate              = options.animate;
            obj.NFZs                 = options.nfzs;
            obj.resetGraphics        = options.resetGraphics;
            obj.animationMultiplier  = options.animationMultiplier;
            obj.hideClock            = options.hideClock;

            obj.UASPos_all = cell(1, length(obj.UAS));
            for i = 1:length(obj.UAS)
                obj.UASPos_all{i} = obj.UAS(i).position;
            end
        end

        function results = runSim(obj)
            dt_local = obj.dt;

            % --- Sensor setup ---
            hasSensors = ~isempty(obj.sensors);
            if hasSensors
                sensorD50  = arrayfun(@(s) s.params.d50, obj.sensors);
                sensorK    = arrayfun(@(s) s.params.k,   obj.sensors);
                sensorLocs = reshape([obj.sensors.location], 2, [])';
            end

            % --- Asset setup ---
            hasAssets = ~isempty(obj.assets);
            if hasAssets
                assetLocs = reshape([obj.assets.location], 2, [])';
            end

            numUAS     = length(obj.UAS);
            uas_active = true(numUAS, 1);

            % Detection score: running sum of per-tick detection probabilities
            % for each UAS. Accumulated across all ticks and all sensors.
            detectionScore = zeros(numUAS, 1);

            destroyedAssets = [];
            outcomeLog      = strings(0);

            % --- Build occupancy map for Hybrid A* ---
            xLimits  = [0, obj.map.size.horiz];
            yLimits  = [0, obj.map.size.vert];

            % Use 0.5 m/cell resolution. Finer cells mean NFZ edges are
            % accurately represented without needing a separate inflate step.
            cellSize = 0.5;
            costMap  = binaryOccupancyMap(yLimits(2), xLimits(2), 1/cellSize);
            costMap.GridOriginInLocal = [xLimits(1), yLimits(1)];

            if ~isempty(obj.NFZs)
                % Sample every 0.25 m — well below cell size — so no edge
                % cell is ever missed, including thin polygon boundaries.
                sampleStep = cellSize / 2;
                xs = xLimits(1) : sampleStep : xLimits(2);
                ys = yLimits(1) : sampleStep : yLimits(2);
                [Xg, Yg] = meshgrid(xs, ys);
                pts   = [Xg(:), Yg(:)];
                inNFZ = false(size(pts, 1), 1);
                for nfzIdx = 1:length(obj.NFZs)
                    inNFZ = inNFZ | isinterior(obj.NFZs(nfzIdx), pts);
                end
                if any(inNFZ)
                    setOccupancy(costMap, pts(inNFZ, :), 1);
                end
            end

            % Ensure UAS start and goal cells are free
            for k = 1:length(obj.UAS)
                setOccupancy(costMap, obj.UAS(k).position(1:2), 0);
                setOccupancy(costMap, obj.UAS(k).target(1:2),   0);
            end

            % --- Initialize Graphics ---
            animate_on = obj.animate;
            if animate_on
                if obj.resetGraphics
                    obj.map.wipeAnimation();
                end
                P = zeros(obj.map.size.vert + 1, obj.map.size.horiz + 1);
                for s = 1:length(obj.sensors)
                    [~, ~, Ps] = obj.sensors(s).createSensorContours(obj.map.size);
                    P = P + Ps;
                end
                obj.map.startAnimation(obj.AOR, obj.assets, obj.NFZs, obj.sensors, P, obj.hideClock);
            end

            simComplete = false;
            tick_count  = 0;

            while ~simComplete
                simComplete = true;
                tick_count  = tick_count + 1;
                currentTime = tick_count * dt_local;

                for i = 1:numUAS
                    if ~uas_active(i); continue; end
                    simComplete = false;

                    % 1. MOVE
                    uasObj = obj.UAS(i);
                    uasObj.hybridAStarMotion(dt_local, tick_count, costMap);
                    pos = uasObj.position;

                    if animate_on
                        obj.UASPos_all{i} = cat(1, obj.UASPos_all{i}, pos);
                    end

                    % 2. SENSOR DETECTION — accumulate detection score
                    % For each sensor, compute the logistic detection
                    % probability dp (0 < dp < 1) and add it to the running
                    % total. No stochastic sampling is performed here; the
                    % score represents the cumulative expected detections
                    % over the entire flight.
                    if hasSensors
                        d_sens = sqrt((sensorLocs(:,1) - pos(1)).^2 + ...
                                      (sensorLocs(:,2) - pos(2)).^2);
                        % dp: column vector, one probability per sensor
                        dp = 1 ./ (1 + exp((d_sens - sensorD50') ./ sensorK'));
                        detectionScore(i) = detectionScore(i) + sum(dp);
                    end

                    % 3. ASSET HIT CHECK
                    eventAsset = false; hitAssetID = 0;
                    if hasAssets
                        d_asset = sqrt((assetLocs(:,1) - pos(1)).^2 + ...
                                       (assetLocs(:,2) - pos(2)).^2);
                        hitIdx  = find(d_asset <= (uasObj.speed * dt_local));
                        if ~isempty(hitIdx)
                            eventAsset = true;
                            hitAssetID = hitIdx(1);
                        end
                    end

                    eventExit = (pos(1) < 0 || pos(1) > obj.map.size.horiz || ...
                                 pos(2) < 0 || pos(2) > obj.map.size.vert);

                    if eventExit
                        outcomeLog(end+1) = "Escaped";
                        uasObj.active     = false;
                        uas_active(i)     = false;

                    elseif eventAsset
                        if ~any(destroyedAssets == hitAssetID)
                            destroyedAssets(end+1) = hitAssetID;
                            outcomeLog(end+1)      = "AssetHit";
                            if animate_on
                                obj.map.animateDestroyedAssets(obj.assets, destroyedAssets);
                            end
                            simComplete = true;
                        end
                    end
                end

                % 4. UPDATE ANIMATION
                if animate_on
                    pause(dt_local / obj.animationMultiplier);
                    obj.map.updateUASAnimation(obj.UASPos_all{1});
                    if ~obj.hideClock; obj.map.updateClock(currentTime); end
                end
            end

            % --- Output results ---
            results.UASPos_all      = obj.UASPos_all;
            results.detectionScore  = detectionScore;  % cumulative sum of dp per UAS
            results.destroyedAssets = destroyedAssets;
            results.outcomeLog      = outcomeLog;
            results.tick            = tick_count;
        end
    end
end