    % Reset
clear
close 
clc

%% Helpful notes
% https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlPID
% https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
%% Derive model
mA = 10; % kg
mB = 2; % kg
L = 0.5; %m
Izz = 0.0833333; %kg*m^2
g = 9.8; % m/s^2

%% Create State Space
E = [mA+mB , mB*L , 0, 0;
     mB*L , Izz + mB*L^2, 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 1];
 
A = [0, 0, 0, 0;
     0, 0, 0, mB*g*L;
     1, 0, 0, 0;
     0, 1, 0, 0];
 
B = [1;0;0;0];

C = eye(4);

D = [0;0;0;0];

%% Simulate Model
out = sim('Jacobs_S22_Student_PendulumCartLinearSimulink.slx');

%% Plot
figure
subplot(2,1,1)
plot(out.output.Time, out.output.Data(:,4))
xlabel('Time (sec)')
ylabel('Angle (rad)')

subplot(2,1,2)
plot(out.input.Time, out.input.Data(:,1))
xlabel('Time (sec)')
ylabel('Force (N)')

figure
plot(out.output.Time, out.output.Data(:,3))
xlabel('Time (sec)')
ylabel('Position (m)')

% You can stabilize the angle if you have infinite space to move the cart