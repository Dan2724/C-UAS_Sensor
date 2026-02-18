clc
clear
close all

theMap = map(100, 100);

asset1 = asset([55, 40]);

N = 100;

% --- 6 No-Fly Zones spread across the 100x100 map ---
NFZ1 = polyshape([8,  25, 42, 44, 25], [91, 96, 89, 66, 87]);   % top-left
NFZ2 = polyshape([71, 84, 82, 68],     [31, 22,  6, 10]);        % bottom-right
NFZ3 = polyshape([30, 50, 52, 32],     [55, 58, 40, 38]);        % centre
NFZ4 = polyshape([60, 78, 80, 62],     [70, 72, 55, 52]);        % top-right
NFZ5 = polyshape([10, 28, 26,  8],     [35, 38, 18, 15]);        % left-middle
NFZ6 = polyshape([45, 65, 67, 47],     [88, 90, 75, 73]);        % top-centre

allNFZs = [NFZ1, NFZ2, NFZ3, NFZ4, NFZ5, NFZ6];

AOR = polyshape([15, 85, 85, 15], [85, 85, 15, 15]);

params.d50 = 10;
params.k   = 10;

turnRadius = 5; % metres — configure UAS turn radius here

detectionScores = zeros(N, 1);

for i = 1:N
    sensor1 = sensor(randPosOutsideNFZs(allNFZs), params.d50, "logistic", params, 1, 0, 360);
    sensor2 = sensor(randPosOutsideNFZs(allNFZs), params.d50, "logistic", params, 1, 0, 360);
    sensor3 = sensor(randPosOutsideNFZs(allNFZs), params.d50, "logistic", params, 1, 0, 360);

    side = randi(4);
    switch side
        case 1; dep = [rand()*100, 0];
        case 2; dep = [rand()*100, 100];
        case 3; dep = [0,          rand()*100];
        case 4; dep = [100,        rand()*100];
    end

    UAS1 = UAS(18, dep, asset1.location, 'HybridAStar', turnRadius=turnRadius);

    sim = simulator(theMap, AOR, UAS1, [sensor1, sensor2, sensor3], [asset1], ...
        tps=20, animate=true, ...
        nfzs=allNFZs, ...
        animationMultiplier=10, hideClock=false);

    results = sim.runSim();

    detectionScores(i) = results.detectionScore(1);
    disp(results.detectionScore(1));
end

% -------------------------------------------------------------------------
% Returns a random [x, y] position on the 100x100 map that does not fall
% inside any of the provided NFZ polyshapes.
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