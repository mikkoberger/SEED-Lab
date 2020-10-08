%%Our sigma and k determined from matching open loop
sigma = 11.45;
k = 1.62;

%Experimental arrays
position = [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0.00 0.00 0.16 0.48 0.83 1.18 1.49 1.77 2.01 2.23 2.42 2.58 2.72 2.85 2.95 3.05 3.13 3.20 3.26 3.31 3.35 3.39 3.42 3.44 3.46 3.47 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.48 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.47 3.46 3.44 3.42 3.40 3.38 3.36 3.34 3.33 3.32 3.32 3.30 3.29 3.28 3.27 3.26 3.25 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.24 3.23 3.20 3.17 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14 3.14];
time = [0: .050: 9.35]; 
L = length(position);
X = length(time);
%%Data from simulink simulation
out=sim('motorstepresponsepid');

%%Plotting the graph
figure(1)
plot(out.simout,'-','color','blue');
hold on
plot(time, position, '-','color','red');
ylim([0 8])
ylabel('Angular Position')
title('PID Implementation')
hold off

