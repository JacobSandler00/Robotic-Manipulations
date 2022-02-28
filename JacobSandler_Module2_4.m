clc ; clear all ; close all
%% Jacob Sandler
%% Robotic Manipulation 
%% Module 2.4-Forward Kinematics Assignment
%% Constants - Link Lengths
L1 = 1
L2 = 1  
L3 = 1
%% RRR Robot
RRR_Link(1) = Revolute('a' , L1 , 'alpha' , pi/2)
RRR_Link(2) = Revolute('a' , L2 )
RRR_Link(3) = Revolute('a' , L3 )
Robot1 = SerialLink(RRR_Link , 'name' , 'Robot1')
%% PRR Robot
PRR_Link(1) = Link('prismatic', 'theta' , 0 , 'a' , L1 , 'alpha' , 0 , 'qlim' , [0 1])
PRR_Link(2) = Link('revolute' , 'a' , L2)
PRR_Link(3) = Link('revolute' , 'a' , L3)
Robot2 = SerialLink(PRR_Link , 'name' , 'Robot2')
%% RPP Robot
RPP_Link(1) = Link('revolute' , 'd' , 0 , 'a', 0 , 'alpha' , 0 , 'offset' , pi/2)
RPP_Link(2) = Link('prismatic' , 'theta' , 0 , 'a' , 0 , 'alpha' , pi/2 ,'qlim' , [0 1])
RPP_Link(3) = Link('prismatic' , 'theta' , 0 , 'a' , 0 , 'alpha' ,-0 , 'qlim' , [0 1])
Robot3 = SerialLink(RPP_Link , 'name' , 'Robot3')
%% Plots and Forward Kinematics
figure(1)
Robot1.teach()
figure(2)
Robot2.teach()
figure(3)
Robot3.teach()

figure(4)
int1 = [0 0 0]
Robot1Pose = Robot1.fkine(int1)
Robot1.plot(int1)
figure(5)
int2 = [1 0 0]
Robot2Pose = Robot2.fkine(int2)
Robot2.plot(int2)
figure(6)
int3 = [0 , 1 , 1]
Robot3Pose = Robot3.fkine(int3)
Robot3.plot(int3)