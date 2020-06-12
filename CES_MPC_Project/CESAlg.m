function [convPath,vel,obs] = CESAlg(start,goal)
res = 10;
%Square obstacles
% map = binaryOccupancyMap(10,10,res);
% %obs 1
% bL = [4,4];
% inMtx = ones(2*res,2*res);
% setOccupancy(map,bL,inMtx);
% %obs 2
% bL = [8,7];
% inMtx = ones(1*res,2*res);
% setOccupancy(map,bL,inMtx);
% 
% %obs 3
% bL = [2,0];
% inMtx = ones(1*res,6*res);
% setOccupancy(map,bL,inMtx);

%round obs
xObsLoc = [5;3;9;3];
yObsLoc = [5;0;7;7.5];
obs = [xObsLoc,yObsLoc];
map = binaryOccupancyMap(10,10,res);
setOccupancy(map,obs,1);
inflate(map,1);
ds = 12; % downsample 
num_iters = 3; %num of iterations
% Set the start and goal poses

% start = [2,2, -pi];
% goal = [8, 8, 0];

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
convPath = path_wps(:,1:2);

tic
numsteps = 6;
for i=1:numsteps
    
    [convPath,vel] = CES(convPath,A,r);
end
toc
%

%generate state angles
angs = zeros(size(convPath,1),1);
lh = 1; %lookahead integer
for i=1:(size(convPath,1) - lh)
    angs(i,1) = atan2(convPath(i+lh,2)-convPath(i,2), convPath(i+lh,1)-convPath(i,1));
end
angs(end,1) = angs(end-1,1);
convPath = [convPath, angs];

% plot 
plot(convPath(:,1), convPath(:,2),'b-o','LineWidth',2);
xlim([-1,11])
ylim([-1,11])
% hold on;
% quiver(convPath(:,1),convPath(:,2),0.25*cos(convPath(:,3)),0.25*sin(convPath(:,3)))
hold off;