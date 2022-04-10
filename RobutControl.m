clear; clc; close all;
syms t 

% From Pos0 to Pos1
tspan = [0 5];
x0_x = [0; 0; 0];
x0_y = [0; 0; 0];
x0_z = [0; 0; 0];

xf_x = [0; 0; 0];
xf_y = [0; 0; 0];
xf_z = [1; 0.2; 0.04];

a1_x = QuinticTraj(tspan, x0_x, xf_x);
a1_y = QuinticTraj(tspan, x0_y, xf_y);
a1_z = QuinticTraj(tspan, x0_z, xf_z);

% Desired trajectories
q1d_z = 0.052*t^3 - 0.0144*t^4 + 0.0011*t^5;

% From Pos1 to Pos2
tspan = [0 15];

x0_x = [0; 0; 0];
x0_y = [0; 0; 0];
x0_z = [1; 0.2; 0.04];

xf_x = [1; 0.0666; 0.0044];
xf_y = [0; 0; 0];
xf_z = [1; 0; -0.01333];

a2_x = QuinticTraj(tspan, x0_x, xf_x);
a2_y = QuinticTraj(tspan, x0_y, xf_y);
a2_z = QuinticTraj(tspan, x0_z, xf_z);

% Desired trajectories
q2d_x = 0.0019*t^3 - 0.0002*t^4;

q2d_z = 1 + 0.2*t + 0.02*t^2 - 0.0098*t^3 + 0.0008*t^4;

% Pos2 to Pos3
tspan = [0 15];

x0_x = [1; 0.0666; 0.0044];
x0_y = [0; 0; 0];
x0_z = [1; 0; -0.01333];

xf_x = [1; 0; -0.0044];
xf_y = [1; 0.0666; 0.0044];
xf_z = [1; 0; 0];

a3_x = QuinticTraj(tspan, x0_x, xf_x);
a3_y = QuinticTraj(tspan, x0_y, xf_y);
a3_z = QuinticTraj(tspan, x0_z, xf_z);

% Desired trajectories
q3d_x = 1 + 0.0666*t + 0.0022*t^2 - 0.0024*t^3 + 0.0002*t^4;
q3d_y = 0.0019*t^3 - 0.0002*t^4;
q3d_z = 1 - 0.0067*t^2 + 0.0013*t^3 - 0.0001*t^4;

% Pos3 to Pos4
tspan = [0 15];

x0_x = [1; 0; -0.0044];
x0_y = [1; 0.0666; 0.0044];
x0_z = [1; 0; 0];

xf_x = [0; -0.0666; -0.0044];
xf_y = [1; 0; -0.0044];
xf_z = [1; 0; 0];

a4_x = QuinticTraj(tspan, x0_x, xf_x);
a4_y = QuinticTraj(tspan, x0_y, xf_y);
a4_z = QuinticTraj(tspan, x0_z, xf_z);

% Desired trajectories
q4d_x = 1 - 0.0022*t^2 - 0.0015*t^3 + 0.0001*t^4;
q4d_y = 1 + 0.0666*t + 0.0022*t^2 - 0.0024*t^3 + 0.0002*t^4;
q4d_z = 1;

% Pos4 to Pos5
tspan = [0 15];

x0_x = [0; -0.0666; -0.0044];
x0_y = [1; 0; -0.0044];
x0_z = [1; 0; 0];

xf_x = [0; 0; 0.0044];
xf_y = [0; -0.0666; -0.0044];
xf_z = [1; 0; 0];

a4_x = QuinticTraj(tspan, x0_x, xf_x);
a4_y = QuinticTraj(tspan, x0_y, xf_y);
a4_z = QuinticTraj(tspan, x0_z, xf_z);

% Desired trajectories
q4d_x = - 0.0666*t - 0.0022*t^2 + 0.0024*t^3 - 0.0002*t^4;
q4d_y = 1 - 0.0022*t^2 - 0.0015*t^3 + 0.0001*t^4;
q4d_z = 1;