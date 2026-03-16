clc
clear
close all

% =========================================================================
%  Force a process-based parallel pool.
% =========================================================================
existingPool = gcp('nocreate');
if ~isempty(existingPool) && ~isa(existingPool, 'parallel.ProcessPool')
    delete(existingPool);
    existingPool = [];
end
if isempty(existingPool)
    parpool('Processes');
end

% =========================================================================
%  Configuration
% =========================================================================
N          = 1;
M          = 100;

MAP_W      = 160;
MAP_H      = 160;

params.d50 = 10;
params.k   = 10;
turnRadius = 10;
uasRadius  = 3;
uasSpeed = 300;

% --- No-Fly Zones: SEPARATE for UAS path planning vs sensors ---
NFZ1_base = polyshape([20,  60, 40], [100, 100, 140]);
NFZ2_base = polyshape([20, 50, 20],     [20, 40,  60]);
NFZ3_base = polyshape([80, 120, 150, 110],     [20, 30, 50, 40]);
NFZ4_base = polyshape([100, 130, 110, 80],     [100, 120, 150, 130]);
allNFZs_base = [NFZ1_base, NFZ2_base, NFZ3_base, NFZ4_base];

% Inflated NFZs for UAS path planning only
NFZ1_inflated = polybuffer(NFZ1_base, uasRadius);
NFZ2_inflated = polybuffer(NFZ2_base, uasRadius);
NFZ3_inflated = polybuffer(NFZ3_base, uasRadius);
NFZ4_inflated = polybuffer(NFZ4_base, uasRadius);
allNFZs_inflated = [NFZ1_inflated, NFZ2_inflated, NFZ3_inflated, NFZ4_inflated];

AOR    = polyshape([15, 85, 85, 15], [85, 85, 15, 15]);
asset1 = asset([80, 80]);

% =========================================================================
%  Pre-generate all random sensor configs and UAS departures + egresses
% =========================================================================
fprintf('Pre-generating %d sensor configurations x %d UAS paths...\n', N, M);

sensorLocs = zeros(N, 3, 2);
depLocs    = zeros(N, M, 2);
egressLocs = zeros(N, M, 2);

for i = 1:N
    for s = 1:3
        sensorLocs(i, s, :) = randPosOutsideNFZs(allNFZs_base, MAP_W, MAP_H);
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
        
        egressSide = randi(4);
        switch egressSide
            case 1; egr = [rand()*MAP_W, 0];
            case 2; egr = [rand()*MAP_W, MAP_H];
            case 3; egr = [0,            rand()*MAP_H];
            case 4; egr = [MAP_W,        rand()*MAP_H];
        end
        egressLocs(i, j, :) = egr;
    end
end

save('UAS_scenarios_100.mat', 'depLocs', 'egressLocs', 'sensorLocs');
fprintf('Saved scenarios to UAS_scenarios_100.mat\n\n');

% =========================================================================
%  Parallel outer loop: sensor configurations
% =========================================================================
fprintf('Running %d configs x %d UAS trials on process-based pool...\n', N, M);

meanScores = zeros(N, 1);
assetLoc   = asset1.location;
nfzList_inflated = allNFZs_inflated;  % For UAS path planning
nfzList_base     = allNFZs_base;      % For sensor LOS
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

    for j = 1:M
        dep  = squeeze(depLocs(i, j, :))';
        egr  = squeeze(egressLocs(i, j, :))';
        
        uObj = UAS(uasSpeed, dep, assetLoc, 'HybridAStar', turnRadius=turnRadius, egressPoint=egr);

        trialMap = map(MAP_H, MAP_W);

        % Pass BOTH NFZ sets: inflated for costmap, base for sensors
        sim = simulator(trialMap, aorShape, uObj, sensorArray, [asset(assetLoc)], ...
            tps=20, animate=false, nfzs_inflated=nfzList_inflated, nfzs_base=nfzList_base);

        res = sim.runSim();
        trialScores(j) = res.detectionScore(1);
    end

    meanScores(i) = mean(trialScores);
end

fprintf('Done.\n');

% =========================================================================
%  Export UAS Paths to Separate CSV Files
% =========================================================================
fprintf('Exporting UAS paths from best configuration to separate CSV files...\n');

% Find best configuration first
[bestScore, bestIdx] = max(meanScores);

% Create output directory
outputDir = 'UAS_Paths_CSV';
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% Re-run best configuration to get all paths
bestLoc1 = squeeze(sensorLocs(bestIdx, 1, :))';
bestLoc2 = squeeze(sensorLocs(bestIdx, 2, :))';
bestLoc3 = squeeze(sensorLocs(bestIdx, 3, :))';

s1_best = sensor(bestLoc1, params.d50, "logistic", params, 1, 0, 360);
s2_best = sensor(bestLoc2, params.d50, "logistic", params, 1, 0, 360);
s3_best = sensor(bestLoc3, params.d50, "logistic", params, 1, 0, 360);

allPaths = cell(M, 1);

for j = 1:M
    dep  = squeeze(depLocs(bestIdx, j, :))';
    egr  = squeeze(egressLocs(bestIdx, j, :))';
    
    uObj = UAS(uasSpeed, dep, assetLoc, 'HybridAStar', turnRadius=turnRadius, egressPoint=egr);
    trialMap = map(MAP_H, MAP_W);
    
    % animate=false for speed, but paths are still captured
    sim = simulator(trialMap, aorShape, uObj, [s1_best, s2_best, s3_best], [asset(assetLoc)], ...
        tps=20, animate=false, nfzs_inflated=nfzList_inflated, nfzs_base=nfzList_base);
    
    res = sim.runSim();
    path = res.UASPos_all{1};
    allPaths{j} = path;
    
    % Export this path to its own CSV
    numPoints = size(path, 1);
    pointIndices = (1:numPoints)';
    
    % Create table for this path
    T = table(pointIndices, path(:,1), path(:,2), path(:,3), ...
        'VariableNames', {'PointIndex', 'X', 'Y', 'Z'});
    
    % Write to individual CSV file
    filename = fullfile(outputDir, sprintf('UAS_Path_%03d.csv', j));
    writetable(T, filename);
    
    if mod(j, 10) == 0
        fprintf('  Exported path %d/%d\n', j, M);
    end
end

fprintf('Exported %d paths to folder: %s\n', M, outputDir);

% Also create a summary CSV with metadata
summaryData = zeros(M, 6);
for j = 1:M
    path = allPaths{j};
    summaryData(j, :) = [j, ...
                         depLocs(bestIdx, j, 1), depLocs(bestIdx, j, 2), ...
                         egressLocs(bestIdx, j, 1), egressLocs(bestIdx, j, 2), ...
                         size(path, 1)];
end

summaryTable = array2table(summaryData, ...
    'VariableNames', {'PathID', 'IngressX', 'IngressY', 'EgressX', 'EgressY', 'NumWaypoints'});
writetable(summaryTable, fullfile(outputDir, 'Path_Summary.csv'));

fprintf('Created Path_Summary.csv with metadata\n');

% Save MAT file
save('UAS_paths_best_config.mat', 'allPaths', 'bestIdx');
fprintf('Also saved to UAS_paths_best_config.mat\n\n');

save('UAS_scenarios_100.mat', 'depLocs', 'egressLocs', 'sensorLocs', 'meanScores');

% =========================================================================
%  Results
% =========================================================================
bestLoc1 = squeeze(sensorLocs(bestIdx, 1, :))';
bestLoc2 = squeeze(sensorLocs(bestIdx, 2, :))';
bestLoc3 = squeeze(sensorLocs(bestIdx, 3, :))';

fprintf('\n=== Best Configuration (config %d / %d) ===\n', bestIdx, N);
fprintf('  Mean detection score : %.4f\n', bestScore);
fprintf('  Sensor 1             : [%.2f, %.2f]\n', bestLoc1(1), bestLoc1(2));
fprintf('  Sensor 2             : [%.2f, %.2f]\n', bestLoc2(1), bestLoc2(2));
fprintf('  Sensor 3             : [%.2f, %.2f]\n', bestLoc3(1), bestLoc3(2));

% =========================================================================
%  Re-run best configuration with animation
% =========================================================================
fprintf('\nRe-running best configuration with animation...\n');

figAnim = figure('Name', 'Best Configuration Animation');

theMap   = map(MAP_H, MAP_W);
bSensor1 = sensor(bestLoc1, params.d50, "logistic", params, 1, 0, 360);
bSensor2 = sensor(bestLoc2, params.d50, "logistic", params, 1, 0, 360);
bSensor3 = sensor(bestLoc3, params.d50, "logistic", params, 1, 0, 360);

medDep = squeeze(depLocs(bestIdx, round(M/2), :))';
medEgr = squeeze(egressLocs(bestIdx, round(M/2), :))';
bUAS   = UAS(uasSpeed, medDep, asset1.location, 'HybridAStar', turnRadius=turnRadius, egressPoint=medEgr);

simBest = simulator(theMap, AOR, bUAS, [bSensor1, bSensor2, bSensor3], [asset1], ...
    tps=20, animate=true, nfzs_inflated=allNFZs_inflated, nfzs_base=allNFZs_base, ...
    animationMultiplier=10, hideClock=false);
simBest.runSim();

figure(figAnim);
title(sprintf('Best Configuration  |  Mean Score: %.4f', bestScore));

% =========================================================================
%  Heat map
% =========================================================================
sigma    = 8;
res      = 1;
xVec     = 0 : res : MAP_W;
yVec     = 0 : res : MAP_H;
[Xg, Yg] = meshgrid(xVec, yVec);

weightSum = zeros(size(Xg));
kernelSum = zeros(size(Xg));

for i = 1:N
    for s = 1:3
        sx = sensorLocs(i, s, 1);
        sy = sensorLocs(i, s, 2);
        G = exp(-((Xg - sx).^2 + (Yg - sy).^2) / (2 * sigma^2));
        weightSum = weightSum + meanScores(i) .* G;
        kernelSum = kernelSum + G;
    end
end

heatMap = zeros(size(Xg));
valid = kernelSum > 1e-10;
heatMap(valid) = weightSum(valid) ./ kernelSum(valid);

figHeat = figure('Name', 'Sensor Placement Heat Map');
ax = axes(figHeat);
imagesc(ax, xVec, yVec, heatMap);
set(ax, 'YDir', 'normal');
colormap(ax, hot);
colorbar(ax);
hold(ax, 'on');

for k = 1:length(allNFZs_base)
    plot(ax, allNFZs_base(k), 'FaceColor', 'none', 'EdgeColor', 'c', 'LineWidth', 1.5);
end
plot(ax, asset1.location(1), asset1.location(2), 'gs', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Asset');
plot(ax, bestLoc1(1), bestLoc1(2), 'b^', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Best S1');
plot(ax, bestLoc2(1), bestLoc2(2), 'b^', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Best S2');
plot(ax, bestLoc3(1), bestLoc3(2), 'b^', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Best S3');
xlim(ax, [0 MAP_W]);
ylim(ax, [0 MAP_H]);
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
title(ax, sprintf('Avg Detection Score by Sensor Placement  (N=%d configs, M=%d UAS trials each)', N, M));
legend(ax, 'show', 'Location', 'northeastoutside');

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