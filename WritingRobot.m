clc ; clear all ; close all

drawSizeModif = 100
XYvalues = fopen('mouseValues1.txt' , 'rt')
testCell = textscan(XYvalues , '%n %n ', 'Delimiter',',')
testStruct = struct('X' , testCell{1} , 'Y' , testCell{2})
xVals = testStruct.X / drawSizeModif; 
yVals = -testStruct.Y / drawSizeModif; 
zVals = 0 ;
%% Link/Joint Specifications

modi = 100
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

zeroT = Niryo.fkine(zeros(1,6))
R = zeroT.R
%% Checking if there are discontinuities in the Drawing
for i = 2 : length(xVals)
    xDiff(i) = abs(xVals(i) - xVals(i-1))/(xVals(i))
    yDiff(i) = abs(yVals(i) - yVals(i-1))/(yVals(i))
end
%% Drawing the trajectory
numStep = length(xVals)
pThresh = .001
for i = 1:numStep
    if( xDiff(i) <= pThresh || yDiff(i) <=pThresh)
        writingCrd = [xVals(i) ; yVals(i) ; zVals];
    else
        zVals = .5
        writingCrd = [xVals(i) ; yVals(i) ; zVals];

    end
    pose3 = SE3(R , writingCrd);
    
    if i == 1
        q = Niryo.ikcon(pose3);
        q0(i , :) = q;

    else
        qPrev = q0(i-1 , :);
        q = Niryo.ikcon(pose3 , qPrev);
        q0(i , :) = q;
    end
    qTraj(i , :) = q;
end

Niryo.plot(qTraj , 'delay' , .01 , 'trail' , {'k' , 'LineWidth' , 2})