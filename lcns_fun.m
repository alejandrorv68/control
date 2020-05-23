function deltax = lcns_fun(t,x,L_pf,P_pf,A_i,B_i,k,tau)

deltax = zeros(1,9)';
% big boss: LConsensus eq.15
deltax = (kron(eye(3),A_i)-kron((L_pf+P_pf),B_i*k))*x;