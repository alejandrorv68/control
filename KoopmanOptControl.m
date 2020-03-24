clear all; close all; clc
mu = -.1;
lambda = 1;
tspan = 0:.01:50;
x0 = [-5; 5];
% LQR on linearized system
A = [-.1 0; 0 1];
B = [0; 1];
Q = eye(2);
R = 1;
C = lqr(A,B,Q,R);
vf = @(t,x) A*x + [0; -lambda*x(1)^2] - B*C*x;
[~,xLQR] = ode45(vf,tspan,x0);
% Koopman operator optimal control (KOOC); i.e., LQR on Koopman operator
A2 = [mu 0 0; 0 lambda -lambda; 0 0 2*mu];
B2 = [0; 1; 0];
Q2 = [1 0 0; 0 1 0; 0 0 0];
R = 1;
C2 = lqr(A2,B2,Q2,R);
% note that controller is nonlinear in the state 'x'
vf2 = @(t,x) A*x + [0; -lambda*x(1)^2] - B*C2(1:2)*x + [0; -C2(3)*x(1)^2];
[t,xKOOC] = ode45(vf2,tspan,x0);
%% Plot
figure(1)
subplot(1,3,1)
plot(xLQR(:,1),xLQR(:,2),'k','LineWidth',1.2);
hold on, grid on
plot(xKOOC(1:50:end,1),xKOOC(1:50:end,2),'r--','LineWidth',1.2);
xlabel('x_1'), ylabel('x_2')
subplot(1,3,2)
plot(tspan,xLQR,'k','LineWidth',1.2);
hold on, grid on
plot(tspan,xKOOC,'r--','LineWidth',1.2);
xlabel('t'), ylabel('x_k')
xlim([0 50])
JLQR = cumsum(xLQR(:,1).^2 + xLQR(:,2).^2 + (C*xLQR')'.^2)';
JKOOC = cumsum(xKOOC(:,1).^2 + xKOOC(:,2).^2 + (C*xKOOC')'.^2)';
subplot(1,3,3)
plot(tspan,JLQR,'k','LineWidth',1.2);
hold on, grid on
plot(tspan,JKOOC,'r--','LineWidth',1.2);
xlabel('t'), ylabel('J')
axis([0 50 0 500000])
legend('LQR','Koopman optimal control')