clc
clear
close all

map = map(100, 100); % Define map and size (vertical, horizontal)

asset1 = asset([55,40]); % Define asset and location (x, y)

sensor1 = sensor([25, 25], 15); % Define sensor(s)

entrances = ingressPosns(100, 100);

%UAS1 = UAS(15, [80, 0], asset1.location, 'Linear'); % Define UAS (speed, entrance, target, mode)

NFZ1 = polyshape([8, 25, 42, 44, 12], [91, 72, 89, 66, 70]);

AOR = polyshape([15, 85, 85, 15], [85, 85, 15, 15]);

%sim = simulator(map, AOR, UAS1, [sensor1], asset1, 0.05, 0, NFZ1); % Define simulator (map, UASs, Assets, Dt, Animate T/F, NFZs)

%s = sim.runSim() % Runsim

results = zeros(100, 1);

for i = 1:height(entrances)
    sim = simulator(map, AOR, UAS(15, entrances(i, :), asset1.location, 'Linear'), [sensor1], asset1, 0.05, 0, NFZ1);
    results(i,1) = sim.runSim();
end

assignin('base', 'results', results)
     
KP = (sum(results, 'all')/height(results))*100;
fprintf('Kill Probability = %d %%', KP)




%{
for x = 20:5:80
    for y = 20:5:80
        s = 0;
        for k = 1:100
            sensor1 = sensor([x, y], 15);
            UAS1 = UAS(15, [k, 0], asset1.location, 'Linear');
            sim = simulator(map, AOR, UAS1, [sensor1], asset1, 0.05, 0, NFZ1); % Define simulator (map, UASs, Assets, Dt, Animate T/F, NFZs) 
            s = sim.runSim() + s; % Run the simulation
        end
        % Store the results for each (x, y) position
        results(x/5 - 1, y/5 - 1) = s;
    end
end
%}