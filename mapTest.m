clc
clear
close all
testMap = map(10);

asset1 = asset([5,5]);

testUAS = UAS(1, [0,2], asset1.location);

testMap.pullUAS(testUAS) % Select UAS as "active"
testMap.displayMap % Display the UAS map
testMap.runSim(0.1)
