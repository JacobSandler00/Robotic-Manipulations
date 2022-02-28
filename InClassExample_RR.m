clc ; clear all ; close all
%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
LA = 1
LB = LA * 1.5
LC = LA * 1.25
%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Links(1) = Revolute('d' , LA , 'a' , LB , 'alpha' , pi/2)
Links(2) = Revolute('d' ,  0 , 'a' , LC , 'alpha' ,    0)
robot = SerialLink(Links , 'name', 'Mr. Roboto')
%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% robot.teach()
numStep = 50
q1 = linspace(-pi,pi,numStep)
q2 = linspace(-pi,pi,numStep)
index = 1
EndPoint = nan(3 , numStep^2)
for i = 1:numStep
    for j = 1:numStep
        EndPoint(:,index) = robot.fkine([q1(i) , q2(j)]).t;
        index = index + 1;
    end
end

%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
plot3(EndPoint(1,:),EndPoint(2,:),EndPoint(3,:))
robot.teach()