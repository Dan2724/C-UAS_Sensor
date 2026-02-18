classdef simulator
    properties
        map
        AOR
        UAS
        UASPos_all
        effectors
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
        fadePings
        costConfig
        effectors3D 
    end

    methods
        function obj = simulator(map, aor, uas, effectors, sensors, assets, options)
            arguments
                map, aor, uas, effectors, sensors, assets
                options.tps = 20
                options.animate = true
                options.nfzs = polyshape.empty
                options.resetGraphics = true
                options.animationMultiplier = 1
                options.hideClock = false
                options.fadePings = false
                options.costConfig = struct('effector', 100, 'asset', 2000, 'leak', 250)
            end
            obj.map = map; obj.AOR = aor; obj.UAS = uas; obj.effectors = effectors; obj.sensors = sensors; obj.assets = assets;
            obj.tick = 0; obj.tps = options.tps; obj.dt = 1 / obj.tps;
            obj.animate = options.animate; obj.NFZs = options.nfzs; obj.resetGraphics = options.resetGraphics;
            obj.animationMultiplier = options.animationMultiplier; obj.hideClock = options.hideClock; obj.fadePings = options.fadePings; 
            obj.costConfig = options.costConfig;
            
            % Initialize history for N UAS
            obj.UASPos_all = cell(1, length(obj.UAS));
            for i = 1:length(obj.UAS)
                obj.UASPos_all{i} = obj.UAS(i).position;
            end
        
            if ~isempty(obj.effectors)
                numEff = length(obj.effectors);
                obj.effectors3D = zeros(numEff, 3);
                for k = 1:numEff
                    loc = obj.effectors(k).location;
                    z = obj.map.getElevation(loc(1), loc(2));
                    obj.effectors3D(k, :) = [loc(1), loc(2), z];
                end
            end
        end

        function results = runSim(obj)
            dt_local = obj.dt;
            cost_eff = obj.costConfig.effector; cost_leak = obj.costConfig.leak; cost_asset = obj.costConfig.asset;
            
            hasSensors = ~isempty(obj.sensors);
            if hasSensors
                % sensor class objects: read d50 and k directly from params
                sensorD50 = arrayfun(@(s) s.params.d50, obj.sensors);
                sensorK   = arrayfun(@(s) s.params.k,   obj.sensors);
                sensorLocs = reshape([obj.sensors.location], 2, [])';
                scan_rate = dt_local; % scan every tick (sensor class has no scanRate field)
            else
                scan_rate = dt_local;
            end

            hasEffectors = ~isempty(obj.effectors3D);
            if hasEffectors
                effLocs = obj.effectors3D;
                effRanges = [obj.effectors.range]';
            end
            
            hasAssets = ~isempty(obj.assets);
            if hasAssets
                assetLocs = reshape([obj.assets.location], 2, [])'; 
            end
            
            terrainProxy = obj.map.terrainProxy;
            numUAS = length(obj.UAS);
            uas_active = true(numUAS, 1);

            destroyedAssets = []; cost = 0; UASkilled = 0; outcomeLog = strings(0);
            UASSensed = zeros(0, 4); % [time, x, y, z]

            % -------------------------------------------------------
            % Build occupancy costMap for Hybrid A* (built once here)
            % -------------------------------------------------------
            hasHybridAStar = any(arrayfun(@(u) u.mode == "HybridAStar", obj.UAS));
            if hasHybridAStar
                mapRes  = obj.map.resolution;
                xLimits = [0, obj.map.size.horiz];
                yLimits = [0, obj.map.size.vert];
                costMap = occupancyMap(obj.map.size.vert, obj.map.size.horiz, 1/mapRes);
                costMap.GridOriginInLocal = [xLimits(1), yLimits(1)];

                % Mark NFZ cells as occupied
                if ~isempty(obj.NFZs) && obj.NFZs.NumRegions > 0
                    xv = xLimits(1):mapRes:xLimits(2);
                    yv = yLimits(1):mapRes:yLimits(2);
                    [Xg, Yg] = meshgrid(xv, yv);
                    pts = [Xg(:), Yg(:)];
                    inNFZ = isinterior(obj.NFZs, pts);
                    if any(inNFZ)
                        setOccupancy(costMap, pts(inNFZ, :), 1);
                    end
                end

                % Use smallest turning radius across all HybridAStar UAS
                haModes = arrayfun(@(u) u.mode == "HybridAStar", obj.UAS);
                hybridSpeeds = arrayfun(@(u) u.speed, obj.UAS(haModes));
                hybridTurnRadius = min(hybridSpeeds) * dt_local;
            end
            % -------------------------------------------------------
            
            % Initialize Graphics
            animate_on = obj.animate;
            if animate_on
                if obj.resetGraphics
                    obj.map.wipeAnimation();
                end
                obj.map.startAnimation(obj.AOR, obj.assets, obj.effectors, obj.sensors, numUAS, obj.hideClock);
                UASsensedPos = [];
            end
            
            simComplete = false; tick_count = 0;
            
            while ~simComplete
                simComplete = true; 
                tick_count = tick_count + 1;
                currentTime = tick_count * dt_local;
                
                for i = 1:numUAS
                    if ~uas_active(i)
                        continue;
                    end
                    simComplete = false;
                    
                    % 1. MOVE UAS
                    uasObj = obj.UAS(i);
                    if uasObj.mode == "Linear"
                        uasObj.linearMotion(dt_local);
                    elseif uasObj.mode == "Search"
                        uasObj.searchMotion(dt_local, obj.assets, destroyedAssets, obj.NFZs);
                    elseif uasObj.mode == "HybridAStar"
                        uasObj.hybridAStarMotion(dt_local, tick_count, hybridTurnRadius, costMap);
                    end
                    pos = uasObj.position;
                    
                    if animate_on
                        obj.UASPos_all{i} = cat(1, obj.UASPos_all{i}, pos);
                    end
                    
                    % 2. CHECK SENSOR DETECTION
                    % Each sensor fires every tick using its logistic model
                    isPinged = false;
                    if hasSensors
                        d_sens = sqrt((sensorLocs(:,1) - pos(1)).^2 + (sensorLocs(:,2) - pos(2)).^2);
                        probs  = 1 ./ (1 + exp((d_sens - sensorD50') ./ sensorK'));
                        if any(probs >= rand(size(probs)))
                            isPinged = true;
                        end
                    end

                    if isPinged
                        UASSensed(end+1, :) = [currentTime, pos];
                        if animate_on
                            UASsensedPos = cat(1, UASsensedPos, [currentTime, pos]);
                            obj.map.animateUASsensed(UASsensedPos);
                        end
                    end
                    
                    % 3. CHECK COLLISIONS
                    eventEffector = false;
                    if hasEffectors
                        d_eff = sqrt(sum((effLocs - pos).^2, 2));
                        if any(d_eff <= effRanges); eventEffector = true; end
                    end
                    
                    eventAsset = false; hitAssetID = 0;
                    if hasAssets
                        d_asset = sqrt((assetLocs(:,1) - pos(1)).^2 + (assetLocs(:,2) - pos(2)).^2);
                        hitIdx = find(d_asset <= (uasObj.speed * dt_local)); 
                        if ~isempty(hitIdx); eventAsset = true; hitAssetID = hitIdx(1); end
                    end
                    
                    z_terr = terrainProxy(pos(2), pos(1)); 
                    eventCrash = (pos(3) <= z_terr);
                    
                    eventExit = (pos(1) < 0 || pos(1) > obj.map.size.horiz || pos(2) < 0 || pos(2) > obj.map.size.vert);

                    if eventEffector
                        cost = cost + cost_eff; outcomeLog(end+1) = "Intercept";
                        uasObj.active = false; uas_active(i) = false; UASkilled = UASkilled + 1;
                        if animate_on; obj.map.animateUASkilled(pos); end
                        
                    elseif eventCrash
                        cost = cost + cost_leak; outcomeLog(end+1) = "TerrainCrash";
                        uasObj.active = false; uas_active(i) = false;
                        if animate_on; obj.map.animateUAScrashed(pos); end
                        
                    elseif eventExit
                        cost = cost + cost_leak; outcomeLog(end+1) = "Escaped";
                        uasObj.active = false; uas_active(i) = false;
                        
                    elseif eventAsset
                        if ~any(destroyedAssets == hitAssetID)
                            destroyedAssets(end+1) = hitAssetID;
                            cost = cost + cost_asset; outcomeLog(end+1) = "AssetHit";
                            if animate_on; obj.map.animateDestroyedAssets(obj.assets, destroyedAssets); end
                            simComplete = true; 
                        end
                    end
                end 
                
                % 4. UPDATE ANIMATION
                if animate_on
                    pause(dt_local/obj.animationMultiplier);
                    obj.map.updateUASAnimation(obj.UASPos_all);
                    if ~obj.hideClock; obj.map.updateClock(currentTime); end
                end
            end
            
            results.UASPos_all = obj.UASPos_all;
            results.UASSensed = UASSensed;       % [time, x, y, z] — used by TestingInterface line 42
            results.destroyedAssets = destroyedAssets; 
            results.cost = cost; 
            results.UASkilled = UASkilled;
            results.outcomeLog = outcomeLog;
            results.tick = tick_count;
        end
    end
end