clc
clear
close all
map = map(5, 5);

asset1 = asset([2.5,2.5]); % Define asset (location)

UAS1 = UAS(1, [5,5], asset1.location); % Define UAS (speed, entrance, target)

sim = simulator(map, UAS1, asset1, 0.05, 1); % Define simulator (map, UASs, Assets, Dt, Animate T/F)
s = sim.runSim() % Runsim