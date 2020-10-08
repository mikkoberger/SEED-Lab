sigma = 11.45;
k = 1.62;
i = 16;
w =16;
time = .001*[1000.00 1050.00 1100.00 1150.00 1202.00 1252.00 1302.00 1352.00 1403.00 1454.00 1504.00 1554.00 1605.00 1655.00 1705.00 1756.00 1807.00 1857.00 1907.00 1957.00]; 
velocity = [1.57 3.65 4.75 5.34 5.65 5.81 5.89 5.93 5.93 5.97 5.93 5.97 5.97 5.93 5.97 5.97 5.93 5.97 5.97 5.97];

out=sim('motorstepresponse');


figure(1)
plot(out.simout,'-','color','blue');
hold on
plot(time,velocity,'-','color','red');
xlim([0 5])
ylim([0 8])
ylabel('Angular Velocity')
hold off

