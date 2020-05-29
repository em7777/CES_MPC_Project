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

%Bubble Parameters
ru = 1; rl = 0.2;
n = size(path_wps(:,1),1);
A = zeros(n,2);
r = zeros(n,2);

%Generate Bubbles
for i=1:length(path_wps)-1
    [ri,result,Ai] = GenerateBubble(path_wps(i,:),map,ru,rl);
    A(i,:) = Ai;
    r(i,:) = ri;
end
%BE SURE TO CHECK VISUALLY IF BUBBLES OVERLAP%
%This is because there are a few more checks in the actual algo. We assume P = A = path_wps(:,1:2)

%CES Algorithm
P = path_wps(:,1:2);
for i=1:3
    P = CES(P,A,r);
end
plot(P(:,1), P(:,2),'g-o');
