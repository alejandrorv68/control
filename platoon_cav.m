% Modeling of a vehicle platoon using:
%   - Linear Consensus control


% Vehicle dynamics: time constant
tau = 0.8;

% Assume 3 cars



L_pf = [ 0  0  0 ;
        -1  1  0 ;
         0 -1  1 ];
P_pf = [1 0 0;
        0 0 0
        0 0 0];
    
k = [10,10,10];
B_i = [0;0;1/tau];
A_i = [0 1 0; 0 0 1; 0 0 -1/tau];



% Initial conditions pos. speed. acc.
initcond = [102 12 2.8 30 19 0.8 10 19 1.8 ];

% Solve consensus problem
[tm,st]=ode113( @(t,x) lcns_fun(t,x,L_pf,P_pf,A_i,B_i,k,tau),[0 101], initcond');

figure;
p1 = plot(tm,st(:,1));
hold on
p2 = plot(tm,st(:,4));
p3 = plot(tm,st(:,7));
xlabel('time')
ylabel('pos [m]')
legend('vehicle 1', 'vehicle 2','vehicle 3')

figure;
v1 = plot(tm,st(:,2));
hold on
v2 = plot(tm,st(:,5));
v3 = plot(tm,st(:,8));
xlabel('time')
ylabel('speed [m/s]')
legend('vehicle 1', 'vehicle 2','vehicle 3')

figure;
a1 = plot(tm,st(:,3));
hold on
a2 = plot(tm,st(:,6));
a3 = plot(tm,st(:,9));
xlabel('time')
ylabel('acc ')
legend('vehicle 1', 'vehicle 2','vehicle 3')




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% References:
% Dynamical modeling and distributed control of connected and automated
% vehicles: challenges and opportunities
% 
% Range Policy of Adaptive Cruise Control Vehicles for
% Improved Flow Stability and String Stability
% Jing Zhou and Huei Peng [used for time constant]
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 