clc ; clear all ; close all
%% Creating the Robot
L1 = 1
L2 = 1
rr_link(1) = Revolute('a' , L1 )
rr_link(2) = Revolute('a' , L2 )
RR = SerialLink(rr_link , 'name' , 'Mr. Roboto')
% RR.teach()
%% Desired Points
Start = [.8 , 1 , 0]
End = [0.2 , 0.3 , 0]*-1
%% Form a Pose
StartPose = SE3(Start)
EndPose = SE3(End)   
%% Inverse Kinematics
RR_Mask = [1 1 0 0 0 0 ]
qStart = RR.ikine(StartPose , 'mask' , RR_Mask)
qEnd = RR.ikine(EndPose , 'mask' , RR_Mask)
% qStart = RR.ikunc(StartPose)
% qEnd = RR.ikunc(EndPose)

%% Trajectory Calculations
num = 32

qTrajectory = nan(num , 2)

xVals = linspace(Start(1) , End(1) , num)
yVals= linspace(Start(2) , End(2) , num)


for i = 1:num 
    current_translation = [xVals(i) ; yVals(i) ; 0]
    current_pose = SE3(current_translation')
    current_JointAngles = RR.ikine(current_pose , 'mask' , RR_Mask)
    qTrajectory(i,:) = current_JointAngles
end
    
q1Values = linspace(qStart(1) , qEnd(1) , num)
q2Values = linspace(qStart(2) , qEnd(2) , num)

qTraj = [q1Values' , q2Values']
figure
RR.plot(qTrajectory , 'top'  ,'delay' , .01 , 'trail' , {'k' , 'LineWidth' , 2})
