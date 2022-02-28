clc ; clear all ; close all ; 
% Jacob Sandler 
% Robotic Manipulations
% 2.3 - Getting Started with Toolbox Forward Kinematic Commands
%% Part 1 
import ETS3.*
a1 = 1; a2 = 1;
ETS3_Part1 = Rz('q1') * Tx(a1) * Rz('q2') * Tx(a2)
%% Part 2 
%% Planar RR
% Robot 1 - DH Convention
L = 1
Link(1) = Revolute('d' , 0 , 'a' , L , 'alpha' , 0)
Link(2) = Revolute('d' , 0 , 'a' , L , 'alpha' , 0)
DH_Robot1_RR= SerialLink(Link, 'name' , 'Robot 1 - DH Planar')
%figure(1)
% Robot 2 - ETS3
L = 1
ETS3_Robot2_RR = Rz('q1') * Tx(L) * Rz('q2') * Tx(L)
%% Non-Planar RR 
% Robot 3 - DH Convention
L = 1
Links(1) = Revolute('d' , L , 'a' , L , 'alpha' , pi/2)
Links(2) = Revolute('d' ,  0 , 'a' , L , 'alpha' ,    0)
DH_Robot3_RR = SerialLink(Links , 'name', 'Robot 3 - DH Non-Planar')
% Robot 4 - ETS3
a1 = 1; a2 = 1;
ETS3_Robot4_RR = Tz(L)*Rz('q1') * Tx(a1) *Ry('q2') * Tx(a2)*Rx(pi/2)
%% Plots 
% All ETS3 Robots plotted first
figure
ETS3_Part1.plot([0,0])
title('ETS3 - Part 1')

figure
ETS3_Robot2_RR.plot([0,0])
title('ETS3 - Part2 - Planar RR')

figure
ETS3_Robot4_RR.plot([0,0])
title('ETS3 - Part2 - NonPlanar RR')

% DH Robots plotted next
figure
DH_Robot1_RR.plot([0,0])
title('Std DH - Part 2 - Planar RR')

figure
DH_Robot3_RR.plot([0,0])
title('Std DH - Part 2 - NonPlanar RR')

