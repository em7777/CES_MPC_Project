close all; clear; clc
map = binaryOccupancyMap(10,10,10);
setOccupancy(map,[5 5], 1);
inflate(map,1);
ds = 15; % downsample factor
% Set the start and goal poses

start = [2,2, -pi];
goal = [8, 8, 0];

[pathStates] = genpath(map,start,goal);


% waypoints 

path_wps = downsample(pathStates, ds);
if pathStates(end,:) ~= path_wps(end,:)
    path_wps(end,:) = pathStates(end,:);
end

%plot the waypoints
plot(path_wps(:,1),path_wps(:,2),'o','Color','k');

%bubbles
ru = 1; rl = 0.2;

for i=2:length(path_wps)-1
    [ri,result,Ai] = GenerateBubble(path_wps(i,:),map,ru,rl);
end
