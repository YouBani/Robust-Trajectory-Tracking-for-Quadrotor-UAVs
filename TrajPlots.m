clear; clc; close all;

% From Pos0 to Pos1
tspan1 = [0 5];

x0_z = [0; 0; 0];
xf_z = [1; 0.2; 0.04];

a1_z = vpa(QuinticTraj(tspan1, x0_z, xf_z));

t = 0:0.1:5;

% Desired trajectories
q1d_z = a1_z(1) + a1_z(2)*t + a1_z(3)*t.^2 + a1_z(4)*t.^3 + a1_z(5)*t.^4 + a1_z(6)*t.^5;
dq1d_z = 2*a1_z(2) + 2*a1_z(3)*t + 3*a1_z(4)*t.^2 + 4*a1_z(5)*t.^3 + 5*a1_z(6)*t.^4;
ddq1d_z = 2*a1_z(3) + 6*a1_z(4)*t + 12*a1_z(5)*t.^2 + 10*a1_z(6)*t.^3;

t = 0:0.3:15;
% From Pos1 to Pos2
tspan2 = [0 15];

x0_x = [0; 0; 0];
xf_x = [1; 0.0666; 0.0044];

a2_x =  vpa(QuinticTraj(tspan2, x0_x, xf_x));

% Desired trajectories
q2d_x = a2_x(1) + a2_x(2)*t + a2_x(3)*t.^2 + a2_x(4)*t.^3 + a2_x(5)*t.^4 + a2_x(6)*t.^5;
dq2d_x = 2*a2_x(2) + 2*a2_x(3)*t + 3*a2_x(4)*t.^2 + 4*a2_x(5)*t.^3 + 5*a2_x(6)*t.^4;
ddq2d_x = 2*a2_x(3) + 6*a2_x(4)*t + 12*a2_x(5)*t.^2 + 20*a2_x(6)*t.^3;

% From Pos2 to Pos3

x0_y = [0; 0; 0];
xf_y = [1; 0.0666; 0.0044];

a3_y = vpa(QuinticTraj(tspan2, x0_y, xf_y));

% Desired trajectories
q3d_y = a3_y(1) + a3_y(2)*t + a3_y(3)*t.^2 + a3_y(4)*t.^3 + a3_y(5)*t.^4 + a3_y(6)*t.^5;
dq3d_y = 2*a3_y(2) + 2*a3_y(3)*t + 3*a3_y(4)*t.^2 + 4*a3_y(5)*t.^3 + 5*a3_y(6)*t.^4;
ddq3d_y = 2*a3_y(3) + 6*a3_y(4)*t + 12*a3_y(5)*t.^2 + 20*a3_y(6)*t.^3;

% Pos3 to Pos4

x0_x = [1; 0; -0.0044];
xf_x = [0; -0.0666; -0.0044];

a4_x = vpa(QuinticTraj(tspan2, x0_x, xf_x));

% Desired trajectories
q4d_x = a4_x(1) + a4_x(2)*t + a4_x(3)*t.^2 + a4_x(4)*t.^3 + a4_x(5)*t.^4 + a4_x(6)*t.^5;
dq4d_x = 2*a4_x(2) + 2*a4_x(3)*t + 3*a4_x(4)*t.^2 + 4*a4_x(5)*t.^3 + 5*a4_x(6)*t.^4;
ddq4d_x = 2*a4_x(3) + 6*a4_x(4)*t + 12*a4_x(5)*t.^2 + 20*a4_x(6)*t.^3;

% Pos4 to Pos5

x0_y = [1; 0; -0.0044];
xf_y = [0; -0.0666; -0.0044];

a5_y = vpa(QuinticTraj(tspan2, x0_y, xf_y));

% Desired trajectories
q5d_y = a5_y(1) + a5_y(2)*t + a5_y(3)*t.^2 + a5_y(4)*t.^3 + a5_y(5)*t.^4 + a5_y(6)*t.^5;
dq5d_y = 2*a5_y(2) + 2*a5_y(3)*t + 3*a5_y(4)*t.^2 + 4*a5_y(5)*t.^3 + 5*a5_y(6)*t.^4;
ddq5d_y = 2*a5_y(3)*t + 6*a5_y(4)*t + 12*a5_y(5)*t.^2 + 20*a5_y(6)*t.^3;

% Plots

figure(1)
subplot(2,2,1)
plot(t, q1d_z);
xlabel('t', 'FontSize',14)
ylabel('q1dz','FontSize',14);
subplot(2,2,2)
plot(t, dq1d_z);
xlabel('t', 'FontSize',14)
ylabel('dq1dz','FontSize',14);
subplot(2,2,3)
plot(t, ddq1d_z);
xlabel('t', 'FontSize',14)
ylabel('ddq1d_z','FontSize',14);

figure(2)
subplot(2,2,1)
plot(t, q2d_x);
xlabel('t', 'FontSize',14)
ylabel('q2dx','FontSize',14);
subplot(2,2,2)
plot(t, dq2d_x);
xlabel('t', 'FontSize',14)
ylabel('dq2dx','FontSize',14);
subplot(2,2,3)
plot(t, ddq2d_x);
xlabel('t', 'FontSize',14)
ylabel('ddq2d_x','FontSize',14);

figure(3)
subplot(2,2,1)
plot(t, q3d_y);
xlabel('t', 'FontSize',14)
ylabel('q3d_y','FontSize',14);
subplot(2,2,2)
plot(t, dq3d_y);
xlabel('t', 'FontSize',14)
ylabel('dq3d_y','FontSize',14);
subplot(2,2,3)
plot(t, ddq3d_y);
xlabel('t', 'FontSize',14)
ylabel('ddq3d_y','FontSize',14);

figure(4)
subplot(2,2,1)
plot(t, q4d_x);
xlabel('t', 'FontSize',14)
ylabel('q4d_x','FontSize',14);
subplot(2,2,2)
plot(t, dq4d_x);
xlabel('t', 'FontSize',14)
ylabel('dq4d_x','FontSize',14);
subplot(2,2,3)
plot(t, ddq4d_x);
xlabel('t', 'FontSize',14)
ylabel('ddq4d_x','FontSize',14);
 
figure(5)
subplot(2,2,1)
plot(t, q5d_y);
xlabel('t', 'FontSize',14)
ylabel('q5d_y','FontSize',14);
subplot(2,2,2)
plot(t, dq5d_y);
xlabel('t', 'FontSize',14)
ylabel('dq5d_y','FontSize',14);
subplot(2,2,3)
plot(t, ddq5d_y);
xlabel('t', 'FontSize',14)
ylabel('ddq5d_y','FontSize',14);