clc
clear
close all
testMap = map(10);
% testMap.displayMap
testUAS = UAS(1,45,-45);
testMap.pullUAS(testUAS)
% testMap.displayMap
testMap.runSim(0.1)
