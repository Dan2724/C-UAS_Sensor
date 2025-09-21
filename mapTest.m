clc
clear
close all
testMap = map(10); % sets map and stores object in "testMap"
% testMap.displayMap
testUAS = UAS(1,45,-45); % creates UAS object in "testUAS"
testMap.pullUAS(testUAS) % pulls the UAS object into the map object
% testMap.displayMap
testMap.runSim(0.1) % runs sim function in map object function 
