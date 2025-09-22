clc
clear
close all

map = map(100, 100); % Define map and size (vertical, horizontal)

asset1 = asset([55,40]); % Define asset and location (x, y)

sensor1 = sensor([25, 25], 15); % Define sensor(s)
sensor2 = sensor([70, 60], 15);

UAS1 = UAS(15, [10, 0], asset1.location, 'Linear'); % Define UAS (speed, entrance, target, mode)

sim = simulator(map, UAS1, [sensor1, sensor2], asset1, 0.05, 1); % Define simulator (map, UASs, Assets, Dt, Animate T/F)

s = sim.runSim() % Runsim