function CESPlanning(Info)
% CESPlanning displays the optimal trajectory of the flying
% robot. (generated)


Xopt = Info.Xopt;
MVopt = Info.MVopt;
fprintf('Optimal Trajectory Cost = %10.6f\n',Info.Cost)

%% Examine solution
t = Info.Topt;
figure(1)
states = {'x','y','theta'};
for i = 1:size(Xopt,2)
    subplot(3,1,i)
    plot(t,Xopt(:,i),'o-')
    title(states{i})
end
figure(2)
MVopt(end,:) = 0; % replace the last row u(k+p) with 0
for i = 1:2
    subplot(2,1,i)
    stairs(t,MVopt(:,i),'o-')
    %axis([0 max(t) -0.1 1.1])
    title(sprintf('Input u(%i)', i));
end
hold off;
figure(3);
plot(Xopt(:,1),Xopt(:,2),'o');
r = 1; %sqrt((Xopt(:,1)-Xopt(:,2)).^2);
quiver(Xopt(:,1),Xopt(:,2),r.*cos(Xopt(:,3)),r.*sin(Xopt(:,3)));
hold on;
quiver(Xopt(end,1),Xopt(end,2),r.*cos(Xopt(end,3)),r.*sin(Xopt(end,3)),'LineWidth',3,'Color','r');
xlabel('x')
ylabel('y')
grid on
title('Optimal Trajectory')
