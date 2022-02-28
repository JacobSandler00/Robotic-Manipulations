%% 
% Student InClass Example - Puma 560
clc ; clear all ; close all
%% 
L1 = 1
L2 = 0.2
L3 = 1
puma_Link(1) = Revolute('d' , L1 , 'alpha' , -pi/2)
puma_Link(2) = Revolute('d' , L2 , 'a' , L3)

my_puma = SerialLink(puma_Link , 'name' , 'My Puma')

my_puma.teach()