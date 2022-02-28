clc ; clear all ; close all
% Jacob Sandler
% Robotic Manipulations
% 2.7 - Mastery Demonstration - Niryo One Trajectory
%% Constants
modi = 1000
L1 = 103  / modi;
L2 = 80   / modi;
L3 = 210  / modi;
L4 = 30   / modi;
L5 = 41.5 / modi;
L6 = 180  / modi;
L7 = 23.7 / modi;
L8 = 5.5  / modi;

Lim1 = degtorad([-175   175  ]);
Lim2 = degtorad([-90     36.7] + 90);
Lim3 = degtorad([-80     90  ]);
Lim4 = degtorad([-175   175  ]);
Lim5 = degtorad([-100   110  ]);
Lim6 = degtorad([-147.5 147.5]);
%% Building The Robot
Niryo_Link(1) = Link('revolute' , 'd' , L1+L2   , 'a' , 0   ,'alpha' , pi/2  , 'qlim' , Lim1);
Niryo_Link(2) = Link('revolute' , 'd' , 0       , 'a' , L3  ,'alpha' , 0     , 'qlim' , Lim2);
Niryo_Link(3) = Link('revolute' , 'd' , 0       , 'a' , L4  ,'alpha' , pi/2  , 'qlim' , Lim3);
Niryo_Link(4) = Link('revolute' , 'd' , L5+L6   , 'a' , 0   ,'alpha' ,-pi/2  , 'qlim' , Lim4);
Niryo_Link(5) = Link('revolute' , 'd' , 0       , 'a' , -L8 ,'alpha' , pi/2  , 'qlim' , Lim5);
Niryo_Link(6) = Link('revolute' , 'd' , L7      , 'a' , 0   ,'alpha' , 0     , 'qlim' , Lim6);
 
Niryo = SerialLink(Niryo_Link , 'name' , "Niryo One")
zeroT = Niryo.fkine(zeros(1,6))
%% Trajectory 1 - crd 1 -> crd2
R1 = [1 0 0 ; 0 -1 0 ; 0 0 -1]    
crd1 = [0.2345 , 0 ,-0.0622]
R2 = [0 1 0 ; 0 0 1 ; 1 0 0]
crd2 = [0 , 0.4 , 0.2]

pose1 = SE3(R1 , crd1)
pose2 = SE3(R2 , crd2)

q1 = Niryo.ikcon(pose1)
q2 = Niryo.ikcon(pose2)

numStep = 20

% Creating the trajectories of angles q{1 - 6}
for i  = 1:length(q1)
    q1Traj(i,:) = linspace(q1(i) , q2(i),numStep);
end

%% Trajectory 3 - Circle
% Constants 
cX = 0.3 ; cY = 0.05 ; cZ = 0.05 ; r = .08;
theta = linspace(pi/2 , 5*pi/2, numStep);
xVAL = ones(1 , length(theta))*cX;
yVAL = r * sin(theta) + cY;
zVAL = r * cos(theta) + cZ;
R3 = [0 0 1 ; 0 -1 0 ; 1 0 0]
% Creating the  trajectory
for j = 1 : numStep
    circleCRD = [xVAL(j); yVAL(j) ; zVAL(j)];
    pose3 = SE3(R3 , circleCRD);
    % Conditional for the 1st case in the loop ...
    % otherwise assign initial cond as previous
    if j == 1
        q3 = Niryo.ikcon(pose3);
        q0(j , :)  = q3;
    else 
        q = q0(j-1 , :);
        qprev(j,:) = q3 ;
        q3 = Niryo.ikcon(pose3 ,q);
        q0(j , :)  = q3;
    end
    q3Traj(j,:) = q3;
end

%% Trajectory 2 - End of Trajectory 1 -> Start of Trajectory 2
for i = 1 : 6
    q2Traj(i , :) = linspace(q1Traj(i , end) , q3Traj(1 , i) , numStep);
end
% Combing the three trajectories
totalTrajectory = [q1Traj' ; q2Traj' ; q3Traj];
%% Plots
% Animation of the Total Trajectories
Niryo.plot(totalTrajectory , 'perspective', 'delay' , .001 , 'trail' , {'k' , 'LineWidth' , 2} , 'movie' , 'TotalTrajectory.mp4')
hold off
plot(1:1:3*numStep , totalTrajectory)
legend('q1' , 'q2' , 'q3', 'q4' , 'q5' , 'q6')
title('Joint Trajectories')
xlabel('step (#)')
ylabel('Angle (rad)')
