function CESPlotTrackingNI(Xref,Ts,Psteps,Tsteps,Xcl,Ucl,obs)
% CESPlotTracking displays the optimal trajectory of the
%unicycle robot
set(0,'DefaultLegendAutoUpdate','off')

Xopt = Xref;
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
% figure(5)
% for i = 1:2
%     subplot(2,1,i)
%     stairs(tt(1:end-1),Ucl(:,i),'LineWidth',3)
%     title(sprintf('Thrust u(%i)', i));
%     axis([0 tt(end) -0.1 2.1])
%     hold on
%     stairs(tp(1:end-1),info.MVopt(1:end-1,i))
%     legend('actual','plan')
%     hold off
% end
figure(6);
hold on

%xopt
plot(Xopt(:,1),Xopt(:,2),'-','MarkerSize',4);
r = 1; %sqrt((Xopt(:,1)-Xopt(:,2)).^2);
% quiver(Xopt(:,1),Xopt(:,2),r.*cos(Xopt(:,3)),r.*sin(Xopt(:,3)));
hold on;

%xcl
plot(Xcl(:,1),Xcl(:,2),'-','MarkerSize',4);
r = 1; %sqrt((Xcl(:,1)-Xcl(:,2)).^2);
% quiver(Xcl(:,1),Xcl(:,2),r.*cos(Xcl(:,3)),r.*sin(Xcl(:,3)));
hold on;

quiver(Xopt(end,1),Xopt(end,2),r.*cos(Xopt(end,3)),r.*sin(Xopt(end,3)),'LineWidth',3);
hold on
quiver(Xcl(end,1),Xcl(end,2),r.*cos(Xcl(end,3)),r.*sin(Xcl(end,3)),'LineWidth',3);
grid on;
legend('CES Gen. Trajectory', 'Actual Trajectory','CES Gen. End Angle','Actual End Angle');
xlabel('x')
ylabel('y')
hold on;
for i=1:length(obs)
    angles = linspace(0,2*pi,100);
    xy = [(1)*cos(angles) + obs(i,1); (1)*sin(angles) + obs(i,2)]';
    plot(xy(:,1),xy(:,2),'.','Color','r','MarkerSize',1);
    hold on;
end

title('Informed Initialization using Optimal Trajectory')
