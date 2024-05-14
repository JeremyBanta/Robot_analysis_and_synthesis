clear; clc;
qs = [0; 0]; qf = [pi/2; pi/2];
O1i = [1; 0]; O2i = [2; 0]; O1f = [0; 1];
O2f = [-1; 1]; b = [2; .5];
xi1 = 10; xi2 = 1;
eta = 1; a1 = .1;
a2 = .1; goal = [-1; 1];
theta1 = 0; theta2 = 0;

L(1) = Link([0,0,1,0]);
L(2) = Link([0,1,1,0]);
botP2 = SerialLink([L(1) L(2)], 'name', 'botP2');

H01=[cos(theta1) -sin(theta1) 0 cos(theta1); ...
    sin(theta1) cos(theta1) 0 sin(theta1); ...
    0 0 1 0; ...
    0 0 0 1];     %The homegenous transform between the first frame and the second fram

A1_2=[cos(theta2) -sin(theta2) 0 cos(theta2); ...
    sin(theta2) cos(theta2) 0 sin(theta2); ...
    0 0 1 0; ...
    0 0 0 1];
H02=H01*A1_2;   %The Homegenous transformation between the second and third fram

count = 0;     %Intial value of count
dist = sqrt((goal(1)-O2i(1))^2+(goal(2)-O2i(2))^2);   %Distance from current location to the goal position
%i = 0;
figure(1);
plot(b(1), b(2),'*');
hold on; 


for i=1:100
    O1i = H01(1:2,4);
    O2i = H02(1:2,4);
    repulO1 = 0;
    repulO2 = Repulsive_Force(eta, O2i, b);
    
    axis([-2.5 2.5 -2.5 2.5]);
    
    attForO1 = Attractive_Force(xi1, O1i, O1f);  %finds the attractive force via the potential field method by 
    attForO2 = Attractive_Force(xi2, O2i, O2f);  %calling the function which will calculate the desired force
    
    F1total = attForO1+repulO1;  %Total force on the first joint of the robot
    F2total = attForO2+repulO2;  %Total force acting on the second joint of the robot
    
    J01 = [-sin(theta1) 0; cos(theta1) 0];                             %Jacobian matrix of the first joint
    J02 = [-sin(theta1)-sin(theta1+theta2) -sin(theta1+theta2); ...    %Jacobian matrix of the second joint
        cos(theta1)+cos(theta1+theta2) cos(theta1+theta2)];
    
    tau = (J01'*F1total) + (J02'*F2total);
    
    thetaAnimation1(count+1) = theta1;                            %these two lines will find the 
    thetaAnimation2(count+1) = theta2;                            %vector of required angle values which will be displayed in the for loop
    
    count = count + 1;  %increases count by 1 each time
    
    qs = qs+tau/norm(tau); 
    theta1 = theta1+a1*tau(1);  %this will adjust theta1 based on the previous position and the torque multipled by a gain
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
    Position1(:,count)=H01(1:end-2,end);
    Position2(:,count)=H02(1:end-2,end);
    
    
    
    O1i = [1*cos(theta1); 1*sin(theta1)];
    O2i = [1*cos(theta1)+1*cos(theta2); 1*sin(theta1)+1*sin(theta2)];
    figure(1);
    if(i==100)
        
        plot(Position1(1,:),Position1(2,:),'blue','LineWidth',3);
        hold on;
        plot(Position2(1,:),Position2(2,:),'red','LineWidth',3);
        hold on; 
        
        for count=1:100
    botP2.plot([thetaAnimation1(count) thetaAnimation2(count)]);  %Plots the robot on the screen
    hold on;   %keeps the plot of the robot for the next time step on the screen
        end
    end
end

