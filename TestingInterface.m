clc
clear
close all

% =========================================================================
%  Force a process-based parallel pool.
%  The Navigation Toolbox (Hybrid A* / DPGrid) is not supported on
%  thread-based workers, so we must use 'Processes'.
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
N          = 10;
M          = 100;  % Changed from 10 to 100

MAP_W      = 180;
MAP_H      = 180;

params.d50 = 10;
params.k   = 10;
turnRadius = 10;
uasRadius  = 3;  % UAS physical radius for NFZ inflation

% --- No-Fly Zones (inflated by UAS radius) ---
NFZ1_base = polyshape([20,  60, 40], [100, 100, 140]);
NFZ2_base = polyshape([20, 50, 20],     [20, 40,  60]);
NFZ3_base = polyshape([80, 120, 150, 110],     [20, 30, 50, 40]);
NFZ4_base = polyshape([100, 130, 110, 80],     [100, 120, 150, 130]);

% Inflate NFZs by UAS radius
NFZ1 = polybuffer(NFZ1_base, uasRadius);
NFZ2 = polybuffer(NFZ2_base, uasRadius);
NFZ3 = polybuffer(NFZ3_base, uasRadius);
NFZ4 = polybuffer(NFZ4_base, uasRadius);
allNFZs = [NFZ1, NFZ2, NFZ3, NFZ4];

AOR    = polyshape([15, 85, 85, 15], [85, 85, 15, 15]);
asset1 = asset([80, 80]);

% =========================================================================
%  Pre-generate all random sensor configs and UAS departures + egresses
% =========================================================================
fprintf('Pre-generating %d sensor configurations x %d UAS paths...\n', N, M);

sensorLocs = zeros(N, 3, 2);
depLocs    = zeros(N, M, 2);
egressLocs = zeros(N, M, 2);  % NEW: egress points

for i = 1:N
    for s = 1:3
        sensorLocs(i, s, :) = randPosOutsideNFZs(allNFZs, MAP_W, MAP_H);
    end
    for j = 1:M
        % Ingress point
        side = randi(4);
        switch side
            case 1; dep = [rand()*MAP_W, 0];
            case 2; dep = [rand()*MAP_W, MAP_H];
            case 3; dep = [0,            rand()*MAP_H];
            case 4; dep = [MAP_W,        rand()*MAP_H];
        end
        depLocs(i, j, :) = dep;
        
        % Egress point (different random edge)
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

% Save scenarios to file
save('UAS_scenarios_100.mat', 'depLocs', 'egressLocs', 'sensorLocs');
fprintf('Saved scenarios to UAS_scenarios_100.mat\n\n');

% =========================================================================
%  Parallel outer loop: sensor configurations
% =========================================================================
fprintf('Running %d configs x %d UAS trials on process-based pool...\n', N, M);

meanScores = zeros(N, 1);
assetLoc   = asset1.location;
nfzList    = allNFZs;
aorShape   = AOR;
pathStore = cell(N, 1);

parfor i = 1:N
    loc1 = squeeze(sensorLocs(i, 1, :))';
    loc2 = squeeze(sensorLocs(i, 2, :))';
    loc3 = squeeze(sensorLocs(i, 3, :))';

    s1 = sensor(loc1, params.d50, "logistic", params, 1, 0, 360);
    s2 = sensor(loc2, params.d50, "logistic", params, 1, 0, 360);
    s3 = sensor(loc3, params.d50, "logistic", params, 1, 0, 360);
    sensorArray = [s1, s2, s3];

    trialScores = zeros(M, 1);
    trialPaths = cell(M, 1);  % NEW: Store actual paths

    for j = 1:M
        dep  = squeeze(depLocs(i, j, :))';
        egr  = squeeze(egressLocs(i, j, :))';
        
        uObj = UAS(18, dep, assetLoc, 'HybridAStar', turnRadius=turnRadius, egressPoint=egr);

        trialMap = map(MAP_H, MAP_W);

        sim = simulator(trialMap, aorShape, uObj, sensorArray, [asset(assetLoc)], ...
            tps=20, animate=false, nfzs=nfzList);

        res = sim.runSim();
        trialScores(j) = res.detectionScore(1);
        trialPaths{j} = res.UASPos_all{1};  % NEW: Save full path
    end

    meanScores(i) = mean(trialScores);
    pathStore{i} = trialPaths;  % NEW: Store paths for this config
end

fprintf('Done.\n');

% =========================================================================
%  Save all UAS paths to file
% =========================================================================
fprintf('Saving UAS path data...\n');
allUASPaths = cell(N, M);
for i = 1:N
    % Note: We need to re-run to get full paths since parfor doesn't save them
    % Or we could modify the parfor to save paths during execution
    fprintf('  Config %d/%d\n', i, N);
end
% For now, paths are already stored in the .mat file via depLocs/egressLocs
fprintf('Path coordinates saved in UAS_scenarios_100.mat\n\n');
save('UAS_scenarios_100.mat', 'depLocs', 'egressLocs', 'sensorLocs', 'pathStore', 'meanScores');

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
%  Re-run best configuration with animation (Figure 1)
% =========================================================================
fprintf('\nRe-running best configuration with animation...\n');

figAnim = figure('Name', 'Best Configuration Animation');

theMap   = map(MAP_H, MAP_W);
bSensor1 = sensor(bestLoc1, params.d50, "logistic", params, 1, 0, 360);
bSensor2 = sensor(bestLoc2, params.d50, "logistic", params, 1, 0, 360);
bSensor3 = sensor(bestLoc3, params.d50, "logistic", params, 1, 0, 360);

medDep = squeeze(depLocs(bestIdx, round(M/2), :))';
medEgr = squeeze(egressLocs(bestIdx, round(M/2), :))';  % NEW
bUAS   = UAS(18, medDep, asset1.location, 'HybridAStar', turnRadius=turnRadius, egressPoint=medEgr);

simBest = simulator(theMap, AOR, bUAS, [bSensor1, bSensor2, bSensor3], [asset1], ...
    tps=20, animate=true, nfzs=allNFZs, animationMultiplier=10, hideClock=false);
simBest.runSim();

figure(figAnim);
title(sprintf('Best Configuration  |  Mean Score: %.4f', bestScore));

% =========================================================================
%  Heat map (Figure 2) — smooth Gaussian kernel density estimate
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

for k = 1:length(allNFZs)
    plot(ax, allNFZs(k), 'FaceColor', 'none', 'EdgeColor', 'c', 'LineWidth', 1.5);
end
plot(ax, asset1.location(1), asset1.location(2), 'gs', ...
    'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Asset');
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