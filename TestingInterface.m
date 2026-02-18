clc
clear
close all

theMap = map(100, 100);

asset1 = asset([55, 40]);

N = 1000;

% --- 6 No-Fly Zones ---
NFZ1 = polyshape([8,  25, 42, 44, 25], [91, 96, 89, 66, 87]);
NFZ2 = polyshape([71, 84, 82, 68],     [31, 22,  6, 10]);
NFZ3 = polyshape([30, 50, 52, 32],     [55, 58, 40, 38]);
NFZ4 = polyshape([60, 78, 80, 62],     [70, 72, 55, 52]);
NFZ5 = polyshape([10, 28, 26,  8],     [35, 38, 18, 15]);
NFZ6 = polyshape([45, 65, 67, 47],     [88, 90, 75, 73]);

allNFZs = [NFZ1, NFZ2, NFZ3, NFZ4, NFZ5, NFZ6];

AOR = polyshape([15, 85, 85, 15], [85, 85, 15, 15]);

params.d50 = 10;
params.k   = 10;

turnRadius = 5;

% --- Storage ---
detectionScores = zeros(N, 1);
sensorLocs      = zeros(N, 3, 2);  % (trial, sensorIdx, [x y])
depLocs         = zeros(N, 2);

% --- Heat map accumulators: 20x20 bins over the 100x100 map ---
nBins    = 20;
binEdges = linspace(0, 100, nBins + 1);
heatSum  = zeros(nBins, nBins);   % accumulated score per bin
heatCnt  = zeros(nBins, nBins);   % visit count per bin

fprintf('Running %d trials (no animation)...\n', N);

for i = 1:N
    loc1 = randPosOutsideNFZs(allNFZs);
    loc2 = randPosOutsideNFZs(allNFZs);
    loc3 = randPosOutsideNFZs(allNFZs);

    sensor1 = sensor(loc1, params.d50, "logistic", params, 1, 0, 360);
    sensor2 = sensor(loc2, params.d50, "logistic", params, 1, 0, 360);
    sensor3 = sensor(loc3, params.d50, "logistic", params, 1, 0, 360);

    side = randi(4);
    switch side
        case 1; dep = [rand()*100, 0];
        case 2; dep = [rand()*100, 100];
        case 3; dep = [0,          rand()*100];
        case 4; dep = [100,        rand()*100];
    end

    UAS1 = UAS(18, dep, asset1.location, 'HybridAStar', turnRadius=turnRadius);

    sim = simulator(theMap, AOR, UAS1, [sensor1, sensor2, sensor3], [asset1], ...
        tps=20, animate=false, ...
        nfzs=allNFZs);

    results = sim.runSim();
    score   = results.detectionScore(1);

    detectionScores(i)    = score;
    sensorLocs(i, 1, :)   = loc1;
    sensorLocs(i, 2, :)   = loc2;
    sensorLocs(i, 3, :)   = loc3;
    depLocs(i, :)         = dep;

    % Accumulate into heat map: add this trial's score to each sensor's bin
    for s = 1:3
        sx = sensorLocs(i, s, 1);
        sy = sensorLocs(i, s, 2);
        bx = find(sx >= binEdges(1:end-1) & sx < binEdges(2:end), 1);
        by = find(sy >= binEdges(1:end-1) & sy < binEdges(2:end), 1);
        if ~isempty(bx) && ~isempty(by)
            heatSum(by, bx) = heatSum(by, bx) + score;
            heatCnt(by, bx) = heatCnt(by, bx) + 1;
        end
    end

    if mod(i, 100) == 0
        fprintf('  %d / %d complete\n', i, N);
    end
end

% --- Best configuration ---
[bestScore, bestIdx] = max(detectionScores);
bestLoc1 = squeeze(sensorLocs(bestIdx, 1, :))';
bestLoc2 = squeeze(sensorLocs(bestIdx, 2, :))';
bestLoc3 = squeeze(sensorLocs(bestIdx, 3, :))';
bestDep  = depLocs(bestIdx, :);

fprintf('\n=== Best Configuration (trial %d) ===\n', bestIdx);
fprintf('  Detection score : %.4f\n', bestScore);
fprintf('  Sensor 1        : [%.2f, %.2f]\n', bestLoc1(1), bestLoc1(2));
fprintf('  Sensor 2        : [%.2f, %.2f]\n', bestLoc2(1), bestLoc2(2));
fprintf('  Sensor 3        : [%.2f, %.2f]\n', bestLoc3(1), bestLoc3(2));
fprintf('  UAS departure   : [%.2f, %.2f]\n', bestDep(1), bestDep(2));

% --- Re-run best configuration with animation ---
fprintf('\nRe-running best configuration with animation...\n');
bSensor1 = sensor(bestLoc1, params.d50, "logistic", params, 1, 0, 360);
bSensor2 = sensor(bestLoc2, params.d50, "logistic", params, 1, 0, 360);
bSensor3 = sensor(bestLoc3, params.d50, "logistic", params, 1, 0, 360);
bUAS1    = UAS(18, bestDep, asset1.location, 'HybridAStar', turnRadius=turnRadius);

simBest = simulator(theMap, AOR, bUAS1, [bSensor1, bSensor2, bSensor3], [asset1], ...
    tps=20, animate=true, ...
    nfzs=allNFZs, ...
    animationMultiplier=10, hideClock=false);
simBest.runSim();
title(sprintf('Best Configuration  |  Score: %.4f', bestScore));

% --- Heat map: average detection score per sensor placement bin ---
heatAvg = zeros(nBins, nBins);
visited = heatCnt > 0;
heatAvg(visited) = heatSum(visited) ./ heatCnt(visited);

figure;
binCentres = binEdges(1:end-1) + diff(binEdges)/2;
imagesc(binCentres, binCentres, heatAvg);
set(gca, 'YDir', 'normal');
colormap(hot);
colorbar;
hold on;

% Overlay NFZs
for k = 1:length(allNFZs)
    plot(allNFZs(k), 'FaceColor', 'none', 'EdgeColor', 'c', 'LineWidth', 1.5);
end

% Mark asset
plot(asset1.location(1), asset1.location(2), 'gs', ...
    'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Asset');

% Mark best sensor locations
plot(bestLoc1(1), bestLoc1(2), 'b^', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Best S1');
plot(bestLoc2(1), bestLoc2(2), 'b^', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Best S2');
plot(bestLoc3(1), bestLoc3(2), 'b^', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Best S3');

xlim([0 100]); ylim([0 100]);
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('Average Detection Score by Sensor Placement  (N=%d trials)', N));
legend('show', 'Location', 'northeastoutside');

% -------------------------------------------------------------------------
function pos = randPosOutsideNFZs(nfzs)
    while true
        pos = [rand()*100, rand()*100];
        inside = false;
        for k = 1:length(nfzs)
            if isinterior(nfzs(k), pos(1), pos(2))
                inside = true;
                break;
            end
        end
        if ~inside
            return;
        end
    end
end