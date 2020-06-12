function J = CESTerminalFn(X,U,e,data)
Qt = 5*eye(3);
J = (X(end,:)-data.References(end,:))*Qt*(X(end,:)-data.References(end,:))';