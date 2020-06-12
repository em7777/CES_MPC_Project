function c = CESAvoidanceFn(X,U,e,data)
xObsLoc = [5;3;9;3];
yObsLoc = [5;0;7;7.5];
obs = [xObsLoc,yObsLoc];
c = [];
rThresh = 1.1;
for i=1:size(X,1)
    for k=1:size(obs,1)    
        c = [c; rThresh - norm([xObsLoc(k)-X(i,1),yObsLoc(k)-X(i,2)],2)];
    end 
end

if max(c) > 0
    tt = 1;
end

