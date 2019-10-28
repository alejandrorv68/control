function [Pulse,Direction,Pulse_count,Homeing] = fcn(error,Kp,trigger,freqh,freq,Ts)
persistent counter counter2 SIZE pulse_count pulse homeing direction

%% we will let the feedback controller work at 50Hz
%% we will assume a sample rate of 10kHz

if isempty(counter)                                                         %First call, initialize variables and go to home
    counter = 0;
    counter2 = 0;
    SIZE=0;
    pulse_count=0;
    pulse=0;
    homeing=0;
    direction = 0;
elseif homeing==0                                                           %Homing
    if trigger<-4
        homeing=1;
    else
        pulse=5*(sign(sin(2*pi*freqh*counter2*Ts))+1)/2;
        direction=0;
    end
    counter2=counter2+1;            
else                                                                        %Normal operation
    if counter==200 %samples between 50Hz 10kHz
        counter=0;
    end
    
    counter=counter+1;  % every sample+1
    
    if counter==1
        SIZE=Kp*error;
    end
    pulse2=5*(sign(sin(min(2*pi*abs(SIZE),freq*2*pi)*counter2*Ts))+1)/2;
    if pulse2>pulse
        pulse_count=pulse_count+1*sign(SIZE);
    end
    pulse=pulse2;
    direction=5*(sign(SIZE)+1)/2;
    counter2=counter2+1;
end
Pulse=pulse;
Pulse_count=pulse_count;
Direction = direction;
Homeing=homeing;
