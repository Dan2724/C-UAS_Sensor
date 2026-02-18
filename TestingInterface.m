clc
clear
close all

% =========================================================================
%  Configuration
% =========================================================================
N          = 10;   % number of sensor configurations to try
M          = 100;    % number of UAS trials per sensor configuration

MAP_W      = 100;
MAP_H      = 100;

params.d50 = 10;
params.k   = 10;
turnRadius = 5;

% --- No-Fly Zones ---
NFZ1 = polyshape([8,  25, 42, 44, 25], [91, 96, 89, 66, 87]);
NFZ2 = polyshape([71, 84, 82, 68],     [31, 22,  6, 10]);
NFZ3 = polyshape([30, 50, 52, 32],     [55, 58, 40, 38]);
NFZ4 = polyshape([60, 78, 80, 62],     [70, 72, 55, 52]);
NFZ5 = polyshape([10, 28, 26,  8],     [35, 38, 18, 15]);
NFZ6 = polyshape([45, 65, 67, 47],     [88, 90, 75, 73]);
allNFZs = [NFZ1, NFZ2, NFZ3, NFZ4, NFZ5, NFZ6];

AOR    = polyshape([15, 85, 85, 15], [85, 85, 15, 15]);
asset1 = asset([55, 40]);

% =========================================================================
%  Pre-generate all random sensor configs and UAS departures
%  (must be done outside parfor — rng is not thread-safe)
% =========================================================================
fprintf('Pre-generating %d sensor configurations x %d UAS departures...\n', N, M);

sensorLocs = zeros(N, 3, 2);   % (config, sensor, [x y])
depLocs    = zeros(N, M, 2);   % (config, trial,  [x y])

for i = 1:N
    for s = 1:3
        sensorLocs(i, s, :) = randPosOutsideNFZs(allNFZs, MAP_W, MAP_H);
    end
    for j = 1:M
        side = randi(4);
        switch side
            case 1; dep = [rand()*MAP_W, 0];
            case 2; dep = [rand()*MAP_W, MAP_H];
            case 3; dep = [0,            rand()*MAP_H];
            case 4; dep = [MAP_W,        rand()*MAP_H];
        end
        depLocs(i, j, :) = dep;
    end
end

% =========================================================================
%  Parallel outer loop: sensor configurations
%  Inner parfor: UAS trials per config
% =========================================================================
fprintf('Running %d configs x %d UAS trials on parallel pool...\n', N, M);

meanScores = zeros(N, 1);   % mean detection score per sensor config

% Broadcast variables (read-only inside parfor)
assetLoc   = asset1.location;
nfzList    = allNFZs;
aorShape   = AOR;

parfor i = 1:N
    loc1 = squeeze(sensorLocs(i, 1, :))';
    loc2 = squeeze(sensorLocs(i, 2, :))';
    loc3 = squeeze(sensorLocs(i, 3, :))';

    s1 = sensor(loc1, params.d50, "logistic", params, 1, 0, 360);
    s2 = sensor(loc2, params.d50, "logistic", params, 1, 0, 360);
    s3 = sensor(loc3, params.d50, "logistic", params, 1, 0, 360);
    sensorArray = [s1, s2, s3];

    trialScores = zeros(M, 1);

    % Inner loop — UAS trials for this sensor config.
    % parfor here would spawn nested pools which MATLAB doesn't support,
    % so this is a plain for loop (still runs in parallel across configs).
    for j = 1:M
        dep  = squeeze(depLocs(i, j, :))';
        uObj = UAS(18, dep, assetLoc, 'HybridAStar', turnRadius=turnRadius);

        % map is a handle class and is not safe to share across workers —
        % create a lightweight throwaway instance per trial.
        trialMap = map(MAP_H, MAP_W);

        sim = simulator(trialMap, aorShape, uObj, sensorArray, [asset(assetLoc)], ...
            tps=20, animate=false, nfzs=nfzList);

        res = sim.runSim();
        trialScores(j) = res.detectionScore(1);
    end

    meanScores(i) = mean(trialScores);
end

fprintf('Done.\n');

% =========================================================================
%  Results
% =========================================================================
[bestScore, bestIdx] = max(meanScores);
bestLoc1 = squeeze(sensorLocs(bestIdx, 1, :))';
bestLoc2 = squeeze(sensorLocs(bestIdx, 2, :))';
bestLoc3 = squeeze(sensorLocs(bestIdx, 3, :))';

fprintf('\n=== Best Configuration (config %d / %d) ===\n', bestIdx, N);
fprintf('  Mean detection score : %.4f\n', bestScore);
fprintf('  Sensor 1             : [%.2f, %.2f]\n', bestLoc1(1), bestLoc1(2));
fprintf('  Sensor 2             : [%.2f, %.2f]\n', bestLoc2(1), bestLoc2(2));
fprintf('  Sensor 3             : [%.2f, %.2f]\n', bestLoc3(1), bestLoc3(2));

% =========================================================================
%  Heat map: average detection score by sensor placement bin
% =========================================================================
nBins    = 20;
binEdges = linspace(0, MAP_W, nBins + 1);
heatSum  = zeros(nBins, nBins);
heatCnt  = zeros(nBins, nBins);

for i = 1:N
    for s = 1:3
        sx = sensorLocs(i, s, 1);
        sy = sensorLocs(i, s, 2);
        bx = find(sx >= binEdges(1:end-1) & sx < binEdges(2:end), 1);
        by = find(sy >= binEdges(1:end-1) & sy < binEdges(2:end), 1);
        if ~isempty(bx) && ~isempty(by)
            heatSum(by, bx) = heatSum(by, bx) + meanScores(i);
            heatCnt(by, bx) = heatCnt(by, bx) + 1;
        end
    end
end

heatAvg          = zeros(nBins, nBins);
visited          = heatCnt > 0;
heatAvg(visited) = heatSum(visited) ./ heatCnt(visited);

figure;
binCentres = binEdges(1:end-1) + diff(binEdges)/2;
imagesc(binCentres, binCentres, heatAvg);
set(gca, 'YDir', 'normal');
colormap(hot); colorbar; hold on;

for k = 1:length(allNFZs)
    plot(allNFZs(k), 'FaceColor', 'none', 'EdgeColor', 'c', 'LineWidth', 1.5);
end
plot(asset1.location(1), asset1.location(2), 'gs', ...
    'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Asset');
plot(bestLoc1(1), bestLoc1(2), 'b^', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Best S1');
plot(bestLoc2(1), bestLoc2(2), 'b^', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Best S2');
plot(bestLoc3(1), bestLoc3(2), 'b^', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Best S3');
xlim([0 MAP_W]); ylim([0 MAP_H]);
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('Avg Detection Score by Sensor Placement  (N=%d configs, M=%d UAS trials each)', N, M));
legend('show', 'Location', 'northeastoutside');

% =========================================================================
%  Re-run best configuration with animation
% =========================================================================
fprintf('\nRe-running best configuration with animation...\n');
theMap   = map(MAP_H, MAP_W);
bSensor1 = sensor(bestLoc1, params.d50, "logistic", params, 1, 0, 360);
bSensor2 = sensor(bestLoc2, params.d50, "logistic", params, 1, 0, 360);
bSensor3 = sensor(bestLoc3, params.d50, "logistic", params, 1, 0, 360);

% Use the median departure from this config's trials as a representative run
medDep = squeeze(depLocs(bestIdx, round(M/2), :))';
bUAS   = UAS(18, medDep, asset1.location, 'HybridAStar', turnRadius=turnRadius);

simBest = simulator(theMap, AOR, bUAS, [bSensor1, bSensor2, bSensor3], [asset1], ...
    tps=20, animate=true, nfzs=allNFZs, animationMultiplier=10, hideClock=false);
simBest.runSim();
title(sprintf('Best Configuration  |  Mean Score: %.4f', bestScore));

% =========================================================================
%  Local helpers
% =========================================================================
function pos = randPosOutsideNFZs(nfzs, w, h)
    while true
        pos = [rand()*w, rand()*h];
        inside = false;
        for k = 1:length(nfzs)
            if isinterior(nfzs(k), pos(1), pos(2))
                inside = true;
                break;
            end
        end
        if ~inside; return; end
    end
end