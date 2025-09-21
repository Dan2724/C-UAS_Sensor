clc
clear
close all
testMap = map(5, 5);

asset1 = asset([2.5,2.5]);

testUAS = UAS(1, [0,2], asset1.location);

testMap.pullUAS(testUAS) % Select UAS as "active"
testMap.displayMap % Display the UAS map

sim = simulator(testMap, testUAS, asset1, 0.1);
sim.runSim(0.1)