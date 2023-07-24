clc;
clear;
close all;
 
frequency = 15;
time1 = 1.7;
time2 = 2;
time3 = 5;
time4 = 10;
velSource1 = 3;
velSource2 = 6;
velSource3 = 9;
velSource4 = 12;
velocity = 343;
distance = 500;
x=0:0.1:time4;
d1=(velocity * time1)/2
d2=(velocity * time2)/2
d3=(velocity * time3)/2
d4=(velocity * time4)/2
plot(time1, d1,'*'); 
hold on 
plot(time2, d2,'*'); 
plot(time3, d3,'*'); 
plot(time4, d4,'*'); 
plot(x,0.5*velocity*x,'--')
legend("Time = 1.7 sec, distance = 291.5","Time = 2 sec, distance = 343","Time = 5 sec, distance = 857.5","Time = 10 sec, distance = 1715", 'relationship of time difference and distanse');

%%
fprintf("\nSource velocity = 3 m/s, Doppler shift = %.2f\n", frequency*(velSource1/velocity));
fprintf("Source velocity = 6 m/s, Doppler shift = %.2f\n", frequency*(velSource2/velocity));
fprintf("Source velocity = 9 m/s, Doppler shift = %.2f\n", frequency*(velSource3/velocity));  
fprintf("Source velocity = 12 m/s, Doppler shift = %.2f\n", frequency*(velSource4/velocity));
d5=(distance/velocity)/2;
fprintf("\nTime delay = %.2f\n", (distance/velocity)/2);
 
delay1 = 1.7;
delay2 = 1.9;
 
x1 = 0;
y1 = 0;
 
x2 = 0;
y2 = 60;
 
distance1 = ((delay1*velocity)/2);
distance2 = ((delay2*velocity)/2);
 
y3 = distance2^2-distance1^2;
y3 = (y3/(y2*(-2)));
 
x3 = distance2^2;
x = (y3-y2)*(y3-y2);
x3 = x3-x;
x3 = sqrt (x3);
fprintf ("\nx3 = +-%2f, y3 = %2f",x3, y3);