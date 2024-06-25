%% 12 Momentum Dumping
% Name: Alan Petersen

%% Preliminaries
% This clears all variables and sets the format to disp more digits.
clearvars
close all
clc
format long

%% Addpath to Attitude Representations Folder
addpath('C:\Users\alanb\OneDrive\Documents\Homework Spring 2024\Spacecraft Attitude Control Theory\01 Attitude Representations')

%% Addpath to Attitude Kinematics Folder
addpath('C:\Users\alanb\OneDrive\Documents\Homework Spring 2024\Spacecraft Attitude Control Theory\02 Attitude Kinematics')

%% Addpath to Attitude Dynamics Folder
addpath('C:\Users\alanb\OneDrive\Documents\Homework Spring 2024\Spacecraft Attitude Control Theory\03 Attitude Dynamics')

%% Addpath to Orbital Environment Folder
addpath('C:\Users\alanb\OneDrive\Documents\Homework Spring 2024\Spacecraft Attitude Control Theory\10 Orbital Environment')

%% Load qBus.mat
load qBus.mat;

%% Load the Mass Properites
mass_properties;

%% Load the orbital environment
load Orbital_Environment.mat;

%% Reaction Wheel Properties
wn = 2*pi*10;       % Reaction wheel natural frequency
zeta = sqrt(2)/2;   % Reaction wheel damping ratio
hwmax = 0.015;      % Max angular momentum (mNms)
hwdotmax = 0.004;   % Max torque (mNm)
safety = 0.5;       % Factor of safety

%%
% Initial reaction wheel angular momentum
hw0_B = [-5e-3;5e-3;5e-3];

%%
% Desired wheel angular momentum
hwstar_B = [0;0;0];

%%
% Torque coil parameters
tau = 1/(100*2*pi);
Mmax = 0.17; % A*m^2

%% Parameters for the design and simulation
% Control System Time Delay
dt_delay = 0.01; % seconds

%% Initial Satellite Attitude
% Body Axes Aligned with Inertial Axes
q0_BI.s = 1;
q0_BI.v = [0;0;0];

%% Desired Satellite Attitude
% Create a desired attitude quaternion by rotating 150 degrees about the
% rotation axis defind below.
e = [1;2;3];  e = e/norm(e);
qstar_BI = e2q(e,150*pi/180);

%%
% The initial attitude in the Simscape simulation can be specified by a
% Body to Inertial direction cosine matrix.  Convert q0_BI to A0_IB;
A0_BI = q2A(q0_BI);
A0_IB = A0_BI';

%% Initial Satellite Angular Velocity
% The same inital angular velocity will be used as in the attitude
% kinematics assignment.
wbi0_B = [0;0;0]; % rad/s

%% Inner Loop Controller Design
% Design a proportional inner loop controller for each principle axis to
% achieve the following design specifications:
%
% * Gain Margin >= 6 dB
% * Phase Margin >= 60 degrees

%%
% Define the plant model for each of the three principal axes
s = tf('s');

G1 = 1/(J_C_P(1,1)*s);
G2 = 1/(J_C_P(2,2)*s);
G3 = 1/(J_C_P(3,3)*s);

%%
% Define the reaction wheel transfer function
Gw = wn^2/(s^2 + 2*zeta*wn*s + wn^2);

%%
% Calculate the gain crossover frequency to use for all three axes
w_crossover = 2*pi*2.11501; % rad/s

%%
% Design a proportional gain for each of the three principle axes that meet
% the design requirements above.
Kd1 = 1/bode(G1*Gw,w_crossover);
Kd2 = 1/bode(G2*Gw,w_crossover);
Kd3 = 1/bode(G3*Gw,w_crossover);

%%
% Place the control gains in a matrix to input into simulink
Kd = diag([Kd1; Kd2; Kd3]);

%% Inner Loop Proportional Controller Design
%%
% Define the controller for each system by cascading the proportional gain
% and the time delay
C1 = Kd1*exp(-s*dt_delay);
C2 = Kd2*exp(-s*dt_delay);
C3 = Kd3*exp(-s*dt_delay);

%% Inner Closed Loop Bode Plot of Controlled Plant
% Plot the closed loop bode plot and display the closed loop 3 dB bandwidth
% for each of the three axes.
CLTF1 = feedback(C1*G1*Gw,1); % C1*G1/(1+C1*G1)
CLTF2 = feedback(C2*G2*Gw,1); % C2*G2/(1+C2*G2)
CLTF3 = feedback(C3*G3*Gw,1); % C3*G3/(1+C3*G3)

%% Outer Loop Proportional Lead Design
%%
% Configure control system designer to start in correct loop configuration.
[num, den] = pade(dt_delay,8);
C_pade8 = tf(num,den);

% config = sisoinit(6);
% config.G1.value = G1;
% config.C1.value = 1;
% config.C2.value = Kd1*C_pade8*Gw;
% config.G2.value = 1/s;
% config.OL1.View = {'bode'};
% config.OL2.View = {};
% controlSystemDesigner(config);

Kp = 1/bode(CLTF1/s, 2*pi*0.92244);

OLTF = Kp*CLTF1/s;

%% Closed Loop Bode Plot of Controlled Plant
% Plot the closed loop bode plot and display the closed loop bandwidth for
% each of the axes
CLTF = feedback(OLTF,1);

%% Outer Loop Design with Integrator and Lead
% Design an integrator lead outer loop controller for each principle axis
% to achieve the following design specifications:
%
% * Gain Margin >= 6 dB
% * Phase Margin >= 60 degrees

%% Outer Loop Controller and Crossover Frequency
Go = 1/s;
% Outer loop gain crossover frequency (For a phase margin of 65 deg)
w_crossover_o = 0.88885*2*pi;
display(w_crossover_o);

z = 0.006*2*pi;
p = 100*2*pi;

Co = (s + z)/(s*(s+p));
K = 1/bode(Co*CLTF1*Go,w_crossover_o);

Co = Co * K;

OLTF = Co*CLTF1*Go;

%% Closed Loop Transfer Function
CLTF = feedback(OLTF,1);

%%
% Momentum dumping transfer function
Gm = 1/(tau*s+1)*1/s*C_pade8;
Km = 1/bode(Gm, 2*pi*0.001);
display(Km)

figure
margin(Km*Gm)

figure
[Y,T] = step(feedback(Km*Gm,1));
plot(T, 5e-3*(1-Y))
title('Step Error Rejection')

%% Simulink

sim('momentum_dumping.slx',3600)

% Error angle from momentum dumping
figure
plot(error_angle, 'LineWidth', 1)
title('Error Angle (deg)')

% Commanded magnetic moment plot
figure
plot(Mstar_B,'LineWidth',1)
title('Commanded Magnetic Moment (A-m^2)')

% Angular momentum of reaction wheels plot
figure
plot(hw_B,'LineWidth',1)
title('Angular Momentum of Reaction Wheels')






