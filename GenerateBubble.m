function [ri,result,Ai] = GenerateBubble(Pi, map, ru, rl)

circlePts = 100;
incSize = 0.1;
angles = linspace(0,2*pi,circlePts);

xc = Pi(1);
yc = Pi(2);
Ai = [xc, yc];
ri = 0; % starting value

while ri < ru
    %discretize the circle
    xy = [(ri + incSize)*cos(angles) + xc; (ri + incSize)*sin(angles) + yc]';
    
    if max(checkOccupancy(map,xy)) == 1
        if ri < rl
            result = 0;
            return
        else 
            break;
        end
    end
    
    ri = ri + incSize;

end

result = 1;
xy = [(ri - incSize)*cos(angles) + xc; (ri - incSize)*sin(angles) + yc]';
plot(xy(:,1),xy(:,2),'.','Color','r');
hold on;
end

