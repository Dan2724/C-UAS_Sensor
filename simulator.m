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

            obj.tick                = 0;
            obj.tps                 = options.tps;
            obj.dt                  = 1 / obj.tps;
            obj.animate             = options.animate;
            obj.NFZs                = options.nfzs;
            obj.resetGraphics       = options.resetGraphics;
            obj.animationMultiplier = options.animationMultiplier;
            obj.hideClock           = options.hideClock;

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
                numSensors = length(obj.sensors);
                sensorD50  = arrayfun(@(s) s.params.d50, obj.sensors);
                sensorK    = arrayfun(@(s) s.params.k,   obj.sensors);
                sensorLocs = reshape([obj.sensors.location], 2, [])';
                k_int      = 3.0;
            end

            % --- Asset setup ---
            hasAssets = ~isempty(obj.assets);
            if hasAssets
                assetLocs = reshape([obj.assets.location], 2, [])';
            end

            numUAS     = length(obj.UAS);
            uas_active = true(numUAS, 1);

            % Raw weighted score accumulator and tick counter per UAS.
            weightedScoreSum = zeros(numUAS, 1);
            tickCount        = zeros(numUAS, 1);

            destroyedAssets = [];
            outcomeLog      = strings(0);

            % --- Build occupancy map for Hybrid A* ---
            xLimits = [0, obj.map.size.horiz];
            yLimits = [0, obj.map.size.vert];

            cellSize = 0.5;
            costMap  = binaryOccupancyMap(yLimits(2), xLimits(2), 1/cellSize);
            costMap.GridOriginInLocal = [xLimits(1), yLimits(1)];

            if ~isempty(obj.NFZs)
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
                    [~, ~, Ps] = obj.sensors(s).createSensorContours( ...
                        obj.map.size, obj.NFZs, obj.sensors);
                    P = P + Ps;
                end
                obj.map.startAnimation(obj.AOR, obj.assets, obj.NFZs, obj.sensors, P, obj.hideClock);
            end

            simComplete = false;
            tick_count  = 0;
            maxTicks    = 10000; % Safety limit to prevent infinite loops

            while ~simComplete && tick_count < maxTicks
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

                    % 2. SENSOR DETECTION
                    if hasSensors
                        d_sens = sqrt((sensorLocs(:,1) - pos(1)).^2 + ...
                                      (sensorLocs(:,2) - pos(2)).^2);
                        dp = 1 ./ (1 + exp((d_sens - sensorD50') ./ sensorK'));

                        % Spatially-varying co-channel interference
                        for si = 1:numSensors
                            neighbourInterference = 0;
                            for sj = 1:numSensors
                                if si == sj; continue; end
                                neighbourInterference = neighbourInterference + dp(sj);
                            end
                            dp(si) = dp(si) / (1 + k_int * neighbourInterference);
                        end

                        % NFZ LOS attenuation
                        if ~isempty(obj.NFZs)
                            for si = 1:numSensors
                                if losBlockedByNFZ(pos(1:2), sensorLocs(si,:), obj.NFZs)
                                    dp(si) = dp(si) * 0.1;
                                end
                            end
                        end

                        if hasAssets
                            d_to_asset = min(sqrt( ...
                                (assetLocs(:,1) - pos(1)).^2 + ...
                                (assetLocs(:,2) - pos(2)).^2));
                        else
                            d_to_asset = 1;
                        end

                        weightedScoreSum(i) = weightedScoreSum(i) + sum(dp) * d_to_asset;
                        tickCount(i)        = tickCount(i) + 1;
                    end

                    % 3. COLLISION/EVENT CHECKS
                    eventAsset = false; hitAssetID = 0;
                    if hasAssets && ~uasObj.headingToEgress
                        d_asset = sqrt((assetLocs(:,1) - pos(1)).^2 + ...
                                       (assetLocs(:,2) - pos(2)).^2);
                        hitIdx  = find(d_asset <= (uasObj.speed * dt_local));
                        if ~isempty(hitIdx)
                            eventAsset = true;
                            hitAssetID = hitIdx(1);
                        end
                    end

                    % Check if UAS exited map
                    eventExit = (pos(1) < 0 || pos(1) > obj.map.size.horiz || ...
                                 pos(2) < 0 || pos(2) > obj.map.size.vert);

                    % Check if UAS reached egress (close to map boundary while heading to egress)
                    eventEgress = false;
                    if uasObj.headingToEgress && ~isempty(uasObj.egressPoint)
                        d_to_egress = sqrt((pos(1) - uasObj.egressPoint(1))^2 + ...
                                          (pos(2) - uasObj.egressPoint(2))^2);
                        if d_to_egress < 5.0  % Within 5m of egress
                            eventEgress = true;
                        end
                    end

                    % Handle events
                    if eventExit || eventEgress
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
                            % UAS continues to egress
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

            % --- Normalise detection scores ---
            detectionScore = zeros(numUAS, 1);
            for i = 1:numUAS
                if tickCount(i) > 0
                    detectionScore(i) = weightedScoreSum(i) / tickCount(i);
                end
            end

            results.UASPos_all      = obj.UASPos_all;
            results.detectionScore  = detectionScore;
            results.destroyedAssets = destroyedAssets;
            results.outcomeLog      = outcomeLog;
            results.tick            = tick_count;
        end
    end
end

% -------------------------------------------------------------------------
function blocked = losBlockedByNFZ(A, B, nfzs)
    blocked = false;
    for n = 1:length(nfzs)
        vx   = nfzs(n).Vertices(:, 1);
        vy   = nfzs(n).Vertices(:, 2);
        numV = length(vx);
        for j = 1:numV
            j2  = mod(j, numV) + 1;
            ex1 = vx(j);  ey1 = vy(j);
            ex2 = vx(j2); ey2 = vy(j2);

            dgx = B(1) - A(1);  dgy = B(2) - A(2);
            dex = ex2 - ex1;    dey = ey2 - ey1;

            denom = dgx * dey - dgy * dex;
            if abs(denom) < 1e-10; continue; end

            t = ((ex1 - A(1)) * dey - (ey1 - A(2)) * dex) / denom;
            u = ((ex1 - A(1)) * dgy - (ey1 - A(2)) * dgx) / denom;

            if t > 1e-6 && t < (1 - 1e-6) && u >= 0 && u <= 1
                blocked = true;
                return;
            end
        end
    end
end