clc ; clear all ; close all
% Jacob Sandler
%% Reading the input mouse X&Y values
drawSizeModif = 2000;
XYvalues = fopen('mouseValues2.txt' , 'rt');
testCell = textscan(XYvalues , '%n %n ', 'Delimiter',',');
testStruct = struct('X' , testCell{1} , 'Y' , testCell{2});
xVals =  testStruct.X / drawSizeModif; 
yVals = -testStruct.Y / drawSizeModif - 0.125;

high = floor(length(xVals) * 1.00); 
med  = floor(length(xVals) * 0.50);
low  = floor(length(xVals) * 0.25);

disp("Current Numer of Iterations (High): ") ; disp(high)
disp("Medium")  ; disp(med)
disp("Low")     ; disp(low)


%% Choosing Resolution 
userInput = input("High Res (H) vs. Medium Res (M) vs. Low Res (L) vs. Custom (C): " , 's');
testIf = false;
while testIf == false 
if userInput == 'H'
    preNumIt = high;
    break
elseif userInput == 'M'
    preNumIt = med;
    break
elseif userInput == 'L'
    preNumIt = low;
    break
elseif userInput == 'C'
    preNumIt = input("Choose Length : ")    ;
%     preNumIt = str2num(preNumIt)
    if length(preNumIt) == 0 || isnumeric(preNumIt) == false
        testIf = false;
    else
       break
    end
else
    userInput = input("High Res (H) vs. Medium Res (M) vs. Low Res (L) vs. Custom (C): " , 's');
    testIf = false;
end
end
numIt = preNumIt;

% Reshaping the Arrays
if length(xVals) >= numIt
  n = 1/(length(xVals)\numIt);
  xVals = xVals(1:n:end , :);
  yVals = yVals(1:n:end , :);
end
zVals = 0;
%% Link/Joint Specifications
% Joint Lengths
modifier = 1000;
L1 = 103  / modifier;
L2 = 80   / modifier;
L3 = 210  / modifier;
L4 = 30   / modifier;
L5 = 41.5 / modifier;
L6 = 180  / modifier;
L7 = 23.7 / modifier;
L8 = 5.5  / modifier;

% Joint Limits
Lim1 = degtorad([-175   175  ]);
Lim2 = degtorad([-90     36.7] + 90); % Added 90 to account for the joint collision 
Lim3 = degtorad([-80     90  ]);
Lim4 = degtorad([-175   175  ]);
Lim5 = degtorad([-100   110  ]);
Lim6 = degtorad([-147.5 147.5]);
%% Building The Robot
% Robot Links
Niryo_Link(1) = Link('revolute' , 'd' , L1+L2   , 'a' , 0   ,'alpha' , pi/2  , 'qlim' , Lim1)
Niryo_Link(2) = Link('revolute' , 'd' , 0       , 'a' , L3  ,'alpha' , 0     , 'qlim' , Lim2)
Niryo_Link(3) = Link('revolute' , 'd' , 0       , 'a' , L4  ,'alpha' , pi/2  , 'qlim' , Lim3)
Niryo_Link(4) = Link('revolute' , 'd' , L5+L6   , 'a' , 0   ,'alpha' ,-pi/2  , 'qlim' , Lim4)
Niryo_Link(5) = Link('revolute' , 'd' , 0       , 'a' , -L8 ,'alpha' , pi/2  , 'qlim' , Lim5)
Niryo_Link(6) = Link('revolute' , 'd' , L7      , 'a' , 0   ,'alpha' , 0     , 'qlim' , Lim6) 

Niryo = SerialLink(Niryo_Link , 'name' , "Writing Robot")

zeroT = Niryo.fkine(zeros(1,6));
R = zeroT.R;
%% Checking if there are discontinuities in the Drawing
for i = 2 : length(xVals)
    xDiff(i) = abs(xVals(i) - xVals(i-1)/(xVals(i)));
    yDiff(i) = abs(yVals(i) - yVals(i-1)/(yVals(i)));
end
%% Drawing the trajectory
numStep = length(xVals);
pThresh = .001;
for i = 1:numStep
    if( xDiff(i) <= pThresh || yDiff(i) <=pThresh)
        writingCrd = [xVals(i) ; yVals(i) ; zVals];
    else
%         zVals = .5;
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

Niryo.plot(qTraj , 'delay' , .01 ,'ortho', 'trail' , {'k' , 'LineWidth' , 2} , 'movie' , 'WritingRobot.mp4' , 'view' , [45 45])
% plot(1:1:numStep , xDiff , 1:1:numStep , yDiff )
legend('xDiff' , 'yDiff')