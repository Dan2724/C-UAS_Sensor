clc
clear
close all
testMap = map(10);
testUAS = UAS(1,[5,0],[5, 5]);
testMap.pullUAS(testUAS)
testMap.displayMap
testMap.runSim(0.1)
