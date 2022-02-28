% Name
% Date
% Class Name
% Module: Assignment

% Reset
clear
close all
clc

%% Initialize 
% Note toolbox must be installed
if(~exist('angvec2r', 'file'))
    error('Corke Toolbox is not installed properly.')
end
% Init Grader
Grader = Module1Grader('Jacob Sandler');

%% Problem 1 (9 pts)
% Show Problem
Grader.PrintProblem1Instructions;
Grader.CreateProblem1;

% Solve Problem 
% Put your code here. Do not change the name of the fields of mySoln.
mySoln.rotationMatrix = eye(3); % A 3x3 rotation matrix
mySoln.translationVector = [0;0;0]; % A 3x1 position vector
mySoln.pose = SE3(mySoln.rotationMatrix, mySoln.translationVector); % Pose of B in Reference Frame A.

% Organize these functions as a cell array where each row is a rotation.
% The first column is the rotation in radians, the second column is the
% body-fixed axis.
mySoln.eulerSoln1 = {1, 'X'; 2, 'Y'; 3, 'Z'}; 
mySoln.eulerSoln2 = {1, 'X'; 2, 'Y'; 3, 'Z'}; 

% Check Problem
Grader.CheckProblem1(mySoln);

%% Problem 2 (9 pts)
Grader.PrintProblem2Instructions;
Grader.CreateProblem2;

% Solve Problem 
% Put your code here. Do not change the name of the fields of mySoln.
ANG = degtorad(0.41882914)
vect =[6 7 5] 
skewVect = skew(vect)

mySoln.rotationMatrix1 = expm(skewVect*ANG); % A 3x3 rotation matrix (Matrix Exponential)
mySoln.rotationMatrix2 = eye(3) + sin(ANG)*skewVect + (1-cos(ANG))*skewVect^2; % A 3x3 rotation matrix (Rodrigues's Formula)
mySoln.pose = SE3();             % Pose of B in Reference Frame A.
mySoln.worldTarget = zeros(3,1); % Location of the identified target in world coordinate system.

% Check Problem
Grader.CheckProblem2(mySoln);

%% Problem 3 (9 pts)
Grader.PrintProblem3Instructions;
Grader.CreateProblem3;

% Solve Problem 
% Put your code here. Do not change the name of the fields of mySoln.
mySoln.poseBinA = SE3(); % Pose of B in Reference Frame A.
mySoln.poseCinA = SE3(); % Pose of C in Reference Frame A.
mySoln.poseCinB = SE3(); % Pose of C in Reference Frame B.
mySoln.targetBinC = zeros(3,1);  % Location of the B's identified target in Robot C's coordinate system.
mySoln.targetBinA = zeros(3,1); % Location of the B's identified target in the world coordinate system.

% Check Problem
Grader.CheckProblem3(mySoln);
