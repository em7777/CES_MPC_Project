function CESPlotTracking(info,Ts,Psteps,Tsteps,Xcl,Ucl)
% CESPlotTracking displays the optimal trajectory of the
%unicycle robot

Xopt = info.Xopt;
tp = Ts*(0:Psteps);
tt = Ts*(0:Tsteps);
figure(4)
states = {'x1','x2','theta'};
for i = 1:3
    subplot(3,1,i)
    plot(tt,Xcl(:,i),'+',tp,Xopt(:,i),'-')
    legend('actual','plan','location','northwest')
    title(states{i})
end
figure(5)
for i = 1:2
    subplot(2,1,i)
    stairs(tt(1:end-1),Ucl(:,i),'LineWidth',3)
    title(sprintf('Thrust u(%i)', i));
    axis([0 tt(end) -0.1 2.1])
    hold on
    stairs(tp(1:end-1),info.MVopt(1:end-1,i))
    legend('actual','plan')
    hold off
end
figure(6);
hold on
%xopt
plot(Xopt(:,1),Xopt(:,2),'o');
r = 1; %sqrt((Xopt(:,1)-Xopt(:,2)).^2);
quiver(Xopt(:,1),Xopt(:,2),r.*cos(Xopt(:,3)),r.*sin(Xopt(:,3)));
hold on;
quiver(Xopt(end,1),Xopt(end,2),r.*cos(Xopt(end,3)),r.*sin(Xopt(end,3)),'LineWidth',3);
hold on
%xcl
plot(Xcl(:,1),Xcl(:,2),'o');
r = 1; %sqrt((Xcl(:,1)-Xcl(:,2)).^2);
quiver(Xcl(:,1),Xcl(:,2),r.*cos(Xcl(:,3)),r.*sin(Xcl(:,3)));
hold on;
quiver(Xcl(end,1),Xcl(end,2),r.*cos(Xcl(end,3)),r.*sin(Xcl(end,3)),'LineWidth',3);

xlabel('x')
ylabel('y')
title('Compare with Optimal Trajectory')
