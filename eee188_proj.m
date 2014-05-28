clear all; close all; clc;
% Motor Parameters
Va=9; %volts
J = 0.01; %Estimated rotational inertia
b = 0.1; %Estimated rotational friction
K = 0.01; %Estimated Motor Constant
R = .921; %Ohm
L = 590e-6;%Henry
Ts=4e-3; %Seconds (proven through arduino's micros() function)

%Motor transfer function:
s = tf('s');
motor_speed = (K*Va)/((J*s+b)*(L*s+R)+K^2);
motor_position = motor_speed*(1/s)

position=c2d(motor_position,Ts)
speed=c2d(motor_speed,Ts);

%pidtool(plant)
%obtained through PIDtool
Kp=6.5991;
Ki=0.64826;
Kd=0.023425;

Gc=pid(Kp,Ki,Kd,0,Ts);
figure('name','Step Speed Response of Motor');
step(feedback(speed*Gc,1)); title('Step Speed Response of Motor');
figure('name','Step Position Response of Motor');
step(feedback(position*Gc,1)); title('Step Position Response of Motor');