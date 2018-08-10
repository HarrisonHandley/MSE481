%%  MSE481 Project 2
%   Task 1 - Creating a Sampled-Data Model of the Plant
%   Task 2 - Design of a digital PID Controller
%          - Settling time less than 2 seconds
%          - Overshoot less than 5%
%          - Steady State Error less than 1%
%   Task 3 - Stability Analysis of a Digital Controller
close all
%%  System Variable Definitions

R = 1;                 %   [Ohms] Armature Resistance
L = 0.5;               %   [Henry] Armature Inductance
K = 0.01;               %   [Nm/A]  Motor Constant
J = 0.01;               %   [kgm^2] Load Inertia
b = 0.1;                %   [Nms] Viscous Friction Constant

%%  State Space Model
% x = [theta
%      theta_dot
%      i];
  
A = [0      1       0
     0     -b/J     K/J
     0     -K/L     -R/L];
 
B = [0
     0
     1/L];
 
C = [0 1 0];

D = [];

SS = ss(A,B,C,D);

Ts = 0.001;
%%  Transfer Function of System
TF = tf(SS);

%   Not using State Space Model if you are a scrub
s = tf('s');
Laplace = K/((J*s+b)*(L*s+R)+K^2);
TestTF = tf(Laplace);
%   Convert to Discrete z-domain using Zero Order Hold and Ts 0.05s
disSS = c2d(TF, Ts, 'zoh');
disTF = tf(disSS);


%%  Task 2: Design of PID Controller

%%  Part a
FBdisTF = feedback(disTF, 1);
[y,t] = step(FBdisTF, 10);
figure(1)
stairs(t, y)
title(['No Controller - Zero Order Hold Step Response | Time Step: ' num2str(Ts) ' [s]'], 'Fontsize', 24)
xlabel('Time [s]', 'Fontsize', 24)
ylabel('Speed [rad/s]', 'Fontsize', 24)
Info = stepinfo(y,t,1); %SS error of 0.902

%%  Part b
Kp = 100;
Ki = 200;
Kd = 10;


Cont = Kp + Ki/s + Kd*s;
disPID = c2d(Cont, Ts, 'Tustin');
FBdisTFPID = feedback(disPID*disTF, 1);
[yPID,tPID] = step(FBdisTFPID, 10);
figure(2)
stairs(tPID, yPID)
title(['PID Controller with Tustin Step Response | Time Step: ' num2str(Ts) ' [s]'], 'Fontsize', 24)
xlabel('Time [s]', 'Fontsize', 24)
ylabel('Speed [rad/s]', 'Fontsize', 24)
InfoPID = stepinfo(yPID,tPID,1);

%%  Task 3: Plot Root Locus
figure(3)
rlocus(FBdisTFPID)
title(['Root Locus of PID | Time Step: ' num2str(Ts) ' [s]' ], 'Fontsize', 24)
xlabel('Real Axis', 'Fontsize', 24)
ylabel('Imaginary Axis', 'Fontsize', 24)
pole(FBdisTFPID)
zero(FBdisTFPID)

% figure(4)
% pzmap(FBdisTFPID)
% title(['Pole Zero Map of PID | Time Step: ' num2str(Ts) ' [s]' ], 'Fontsize', 24)
% xlabel('Real Axis', 'Fontsize', 24)
% ylabel('Imaginary Axis', 'Fontsize', 24)