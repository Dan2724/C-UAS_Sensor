clc
clear
close all

map = map(100, 100);                                                        % create map object in map.m:  (L, L)

asset1 = asset([55,40]);                                                    % create asset object(s) in asset.m:  (x, y)
asset2 = asset([60,45]);

sensor1 = sensor([25, 25], 15);                                             % create sensor object(s) in sensor.m:  ([x, y], range)
sensor2 = sensor([75, 75], 15);

entrances = ingressPosns(100, 100);                                         % use ingressPosns.m function to make entrances = [x1, y1; x2, y2; etc.]

%UAS1 = UAS(15, [80, 0], asset1.location, 'Linear');                        % example of creating UAS object in UAS.m:   (speed, entrance, target, mode)

NFZ1 = polyshape([8, 25, 42, 44, 12], [91, 72, 89, 66, 70]);                % create NFZ polygon using polyshape fxn: ([x1, x2, x3, x4, x5], [y1, y2, y3, y4, y5])

AOR = polyshape([15, 85, 85, 15], [85, 85, 15, 15]);                        % define AOR within greater map using polyshape fxn: ([x1, x2, x3, x4], [y1, y2, y3, y4])

%sim = simulator(map, AOR, UAS1, [sensor1], [asset1], 0.05, 0, NFZ1);       % example of creating sim object in simulator.m:  (Map, AOR, UASs, Sensors, Assets, tps=20, aniamte=true, nfzs=[], resetGraphics=true, animationMultiplier=100)

results = zeros(100, 1);                                                    % preallocate results vector (1 = kill, 0 = no kill, [2 = NFZ hit])

for i = 1:height(entrances)                                                 % run sim for every UAS entrance location and record kill/nokill/NFZincursion in results vector                                           
    sim = simulator(map, AOR, UAS(15, entrances(i, :), asset1.location, 'Search'), [sensor1, sensor2], [asset1, asset2], tps=20, animate=true, nfzs=NFZ1, resetGraphics=false, animationMultiplier=100);
    results = sim.runSim();
end
     
DP = (sum(results, 'all')/height(results))*100;                             % calculate and display detection probability
fprintf('Detection Probability = %d %%  ', DP)


