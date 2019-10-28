omega=2*pi*2;
s=tf('s');
filter=omega^2/(s^2+2*omega*0.7*s+omega^2);
filter_d=c2d(filter,Ts,'tustin');
[Af,Bf,Cf,Df]=ssdata(filter_d);
