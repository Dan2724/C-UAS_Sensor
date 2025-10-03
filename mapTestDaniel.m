clc
clear
close all

map = map(100, 100); % Define map and size (vertical, horizontal)

asset1 = asset([55,40]); % Define asset and location (x, y)
asset2 = asset([60,45]);

params.d50 = 10;
params.k = 10;

sensor1 = sensor([25, 25], params.d50, "logistic", params, 1, 0, 360); % Define sensor(s)
sensor2 = sensor([75, 75], params.d50, "logistic", params, 1, 0, 360);
sensor3 = sensor([35, 40], params.d50, "logistic", params, 1, 0, 360);

UAS1 = UAS(18, [0, 0], asset2.location, 'Linear'); % Define UAS

NFZ1 = polyshape([8, 25, 42, 44, 25], [91, 96, 89, 66, 87]); % Define NFZs as a polyshape
NFZ2 = polyshape([71, 84, 82, 68], [31, 22, 6, 10]);

AOR = polyshape([15, 85, 85, 15], [85, 85, 15, 15]); % Define AOR as a polyshape

sim = simulator(map, AOR, UAS1, [sensor1, sensor2, sensor3], [asset1, asset2], tps=20, animate=true, nfzs=[NFZ1, NFZ2], animationMultiplier=10, hideClock=false);

results = sim.runSim(); % Runsim

detectionProbability = mean(results.UASSensed(:, 4))