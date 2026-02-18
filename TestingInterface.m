clc
clear
close all

map = map(100, 100); % Define map and size (vertical, horizontal)

asset1 = asset([55,40]); % Define asset and location (x, y)

N = 100; % Max Iterations

NFZ1 = polyshape([8, 25, 42, 44, 25], [91, 96, 89, 66, 87]); % Define NFZs as a polyshape
NFZ2 = polyshape([71, 84, 82, 68], [31, 22, 6, 10]);

AOR = polyshape([15, 85, 85, 15], [85, 85, 15, 15]); % Define AOR as a polyshape

params.d50 = 10;
params.k = 10;

for i = 1:N
    sensor1 = sensor([rand() * 100, rand() * 100], params.d50, "logistic", params, 1, 0, 360); % Define sensor(s)
    sensor2 = sensor([rand() * 100, rand() * 100], params.d50, "logistic", params, 1, 0, 360);
    sensor3 = sensor([rand() * 100, rand() * 100], params.d50, "logistic", params, 1, 0, 360);

    side = randi(4);
    switch side
        case 1
            dep = [rand() * 100, 0];
        case 2
            dep = [rand() * 100, 100];
        case 3
            dep = [0, rand() * 100];
        case 4
            dep = [100, rand() * 100];
    end

    UAS1 = UAS(18, dep, asset1.location, 'HybridAStar'); % Define UAS

    sim = simulator(map, AOR, UAS1, [], [sensor1, sensor2, sensor3], [asset1], tps=20, animate=true, nfzs=[NFZ1, NFZ2], animationMultiplier=10, hideClock=false);

    results = sim.runSim(); % Runsim

    detectionProbability = sum(results.UASSensed(:, 4));
end