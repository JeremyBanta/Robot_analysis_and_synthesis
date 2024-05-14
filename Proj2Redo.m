clear;
clc;

qs = [0; 0];
qf = [pi/2; pi/2];
O1i = [1; 0];
O2i = [2; 0];
O1f = [0; 1];
O2f = [-1; 1];
b = [2; .5];
xi1 = 10;
xi2 = 1;
eta = 1;
a1 = .1;
a2 = .1;
goal = [-1; 1];

theta1 = 0;
theta2 = 0;

L(1) = Link([0,0,1,0]);
L(2) = Link([0,1,1,0]);
bot = SerialLink([L(1) L(2)], 'name', 'bot');

H01=[cos(theta1) -sin(theta1) 0 cos(theta1); ...
    sin(theta1) cos(theta1) 0 sin(theta1); ...
    0 0 1 0; ...
    0 0 0 1];

A1_2=[cos(theta2) -sin(theta2) 0 cos(theta2); ...
    sin(theta2) cos(theta2) 0 sin(theta2); ...
    0 0 1 0; ...
    0 0 0 1];
H02=H01*A1_2;

count = 0;
dist = sqrt((goal(1)-O2i(1))^2+(goal(2)-O2i(2))^2);
i = 0;

for i=1:100
    O1i = H01(1:2,4);
    O2i = H02(1:2,4);
    repulO1 = 0;
    repulO2 = Repulsive_Force(eta, O2i, b);
    
    plot(O1i(1), O1i(2), '-o', ...
        O2i(1), O2i(2), 's', ...
        b(1), b(2), '*');
    axis([-2.5 2.5 -2.5 2.5]);

    
    hold on;
    
    attForO1 = Attractive_Force(xi1, O1i, O1f);
    attForO2 = Attractive_Force(xi2, O2i, O2f);
    
    F1total = attForO1+repulO1;
    F2total = attForO2+repulO2;
    
    J01 = [-sin(theta1) 0; cos(theta1) 0];
    J02 = [-sin(theta1)-sin(theta1+theta2) -sin(theta1+theta2); ...
        cos(theta1)+cos(theta1+theta2) cos(theta1+theta2)];
    
    tau = (J01'*F1total) + (J02'*F2total);
    
    thetaAnimation1(count+1) = theta1;
    thetaAnimation2(count+1) = theta2;
    
    count = count + 1;
    
    qs = qs+tau/norm(tau);
    theta1 = theta1+a1*tau(1);
    theta2 = theta2+a2*tau(2);
    
    H01=[cos(theta1) -sin(theta1) 0 cos(theta1); ...
        sin(theta1) cos(theta1) 0 sin(theta1); ...
        0 0 1 0; ...
        0 0 0 1];

    A1_2=[cos(theta2) -sin(theta2) 0 cos(theta2); ...
        sin(theta2) cos(theta2) 0 sin(theta2); ...
        0 0 1 0; ...
        0 0 0 1];
    H02=H01*A1_2;
    
    O1i = [1*cos(theta1); 1*sin(theta1)];
    O2i = [1*cos(theta1)+1*cos(theta2); 1*sin(theta1)+1*sin(theta2)];
    
end
for j = 1:count
    figure(1);
    bot.plot([thetaAnimation1(j) thetaAnimation2(j)]);
    hold on
end