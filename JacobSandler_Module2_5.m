clc ; clear all ; close all
% Jacob Sandler
% Robotic Manipulations
% 2.5 - Inverse Kinematics Assignment
%% Constants - Link Lengths
L1 = 1
L2 = 1
L3 = 1
crd = [0.6 , 0.2 , 0.5]
msk = [1 1 1 0 0 0]
T = SE3(crd)
%% RRR Robot
RRR_Link(1) = Revolute('a' , L1 , 'alpha' , pi/2)
RRR_Link(2) = Revolute('a' , L2 )
RRR_Link(3) = Revolute('a' , L3 )
Robot1 = SerialLink(RRR_Link , 'name' , 'Robot1')
% Creating two uniquie solutions
IK_RRR_Soln1 = Robot1.ikine(T , 'mask' , msk)
IK_RRR_Soln2 = Robot1.ikine(T , 'mask' , msk , 'q0' , [.3218 , pi , -pi])
% Checking if they are right
FK_RRR_Soln1 = Robot1.fkine(IK_RRR_Soln1).t
FK_RRR_Soln2 = Robot1.fkine(IK_RRR_Soln2).t

figure(1)
plot3(crd(1) , crd(2) , crd(3) ,'o')
Robot1.plot(IK_RRR_Soln1)
title('RRR Configuration 1')
%% PRR Robot
PRR_Link(1) = Link('prismatic', 'theta' , 0 , 'a' , L1 , 'alpha' , 0 , 'qlim' , [0 1])
PRR_Link(2) = Link('revolute' , 'a' , L2)
PRR_Link(3) = Link('revolute' , 'a' , L3)
Robot2 = SerialLink(PRR_Link , 'name' , 'Robot2')
% Creating two unique solutions
IK_PRR_Soln1 = Robot2.ikine(T , 'mask' , msk)
IK_PRR_Soln2 = Robot2.ikine(T , 'mask' , msk , 'q0' , [.6 , -pi/2 , -pi/6])
% Checking if they are right
FK_PRR_Soln1 = Robot2.fkine(IK_PRR_Soln1).t
FK_PRR_Soln2 = Robot2.fkine(IK_PRR_Soln2).t

figure(2)
Robot2.plot(IK_PRR_Soln1)
title('PRR Configuration 1')
%% RPP Robot
RPP_Link(1) = Link('revolute' , 'd' , 0 , 'a', 0 , 'alpha' , 0 , 'offset' , pi/2)
RPP_Link(2) = Link('prismatic' , 'theta' , 0 , 'a' , 0 , 'alpha' , pi/2 ,'qlim' , [0 1])
RPP_Link(3) = Link('prismatic' , 'theta' , 0 , 'a' , 0 , 'alpha' , 0 , 'qlim' , [0 1])
Robot3 = SerialLink(RPP_Link , 'name' , 'Robot3')
% Creating two unique solutions
IK_RPP_Soln1 = Robot3.ikine(T , 'mask' , msk)
IK_RPP_Soln2 = Robot3.ikine(T , 'mask' , msk , 'q0' , [6.5 , pi , -pi]) 
% Checking if they are right
FK_RPP_Soln1 = Robot3.fkine(IK_RPP_Soln1).t
FK_RPP_Soln2 = Robot3.fkine(IK_RPP_Soln2).t

figure(3)
Robot3.plot(IK_RPP_Soln1)
title('RPP Configuration 1')

