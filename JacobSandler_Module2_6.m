clc ; clear all ; close all 
% Jacob Sandler 
% Robotic Manipulations
% 2.6 - MidModule Demonstration - Niryo One Kinematics
%% Link/Joint Specifications
modi = 1000

L1 = 103  / modi
L2 = 80   / modi
L3 = 210  / modi
L4 = 30   / modi
L5 = 41.5 / modi
L6 = 180  / modi
L7 = 23.7 / modi
L8 = 5.5  / modi

Lim1 = degtorad([-175   175  ])
Lim2 = degtorad([-90     36.7] + 90) % Added 90 to account for the joint collision 
Lim3 = degtorad([-80     90  ])
Lim4 = degtorad([-175   175  ])
Lim5 = degtorad([-100   110  ])
Lim6 = degtorad([-147.5 147.5])
%% Building The Robot
Niryo_Link(1) = Link('revolute' , 'd' , L1+L2   , 'a' , 0   ,'alpha' , pi/2  , 'qlim' , Lim1)
Niryo_Link(2) = Link('revolute' , 'd' , 0       , 'a' , L3  ,'alpha' , 0     , 'qlim' , Lim2)
Niryo_Link(3) = Link('revolute' , 'd' , 0       , 'a' , L4  ,'alpha' , pi/2  , 'qlim' , Lim3)
Niryo_Link(4) = Link('revolute' , 'd' , L5+L6   , 'a' , 0   ,'alpha' ,-pi/2  , 'qlim' , Lim4)
Niryo_Link(5) = Link('revolute' , 'd' , 0       , 'a' , -L8 ,'alpha' , pi/2  , 'qlim' , Lim5)
Niryo_Link(6) = Link('revolute' , 'd' , L7      , 'a' , 0   ,'alpha' , 0     , 'qlim' , Lim6) 
% 
Niryo = SerialLink(Niryo_Link , 'name' , "Niryo One")
%% Visualizing the Workspace
step = 10
q1 = linspace(Lim1(1) , Lim1(2) , step)
q2 = linspace(Lim2(1) , Lim2(2) , step) 
q3 = linspace(Lim3(1) , Lim3(2) , step)
q4 = 0 
q5 = 0
q6 = 0 

endPT1 = nan(3 , step^3);
index = 1;
for i = 1:step
    for j = 1:step
        for k = 1:step
            endPT(:,index) = Niryo.fkine([q1(i) , q2(j) , q3(k) , q4 , q5 , q6]).t;
            index = index + 1;
        end
    end
end

%% Plots
plot3(endPT(1,:) , endPT(2,:) ,endPT(3,:) , '.')
Niryo.teach()
