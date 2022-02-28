%% TestFile 1.2.3.a/b
clc ; clear ; close
A = sqrt(2)/2
R = [A,0,A ; -1/2,A,1/2 ; -1/2,-A,1/2]

[theta , vect] = tr2angvec(R)

angle = acos(.5*(trace(R)-1)) %Given EQN

v = [R(3,2)-R(2,3) ; R(1,3) - R(3,1) ; R(2,1) - R(1,2)]
V = (1/2/sin(angle))*v %Given EQN

quat = [angle , V'] % Above EQN 2.22
UnitQuat = [(cos(angle/2)) , V'*sin(angle/2)] % EQN 2.22
%%

q = Quaternion(angle,V)
qUnit = UnitQuaternion(R)
