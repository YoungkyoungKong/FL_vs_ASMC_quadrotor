clear;
clc;
%% parameter setting
l=1;
g=9.81;
x_d1=20;
y_d1=-10;
z_d1=10;
x_d2 = 0;
y_d2 = 0;
z_d2 = 0;
psi_d=0;
x_0 = 10;
y_0 = 10;
z_0 = 20;
phi_0 = pi/6;
theta_0 = pi/6;
psi_0 = pi/6;
% use LQR to set k1, k2, k3, k4
A = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0];
B = [0; 0; 0; 1];
C = [1 0 0 0];
sys = ss(A,B,C,0);
Q = diag([1 10 5 1]);
R = 0.01;
[K,S,P] = lqr(sys,Q,R);
k1 = K(1);
k2 = K(2);
k3 = K(3);
k4 = K(4);
% use PD tunning to set k_psi1, k_psi2
k_psi1 = 30;
k_psi2 = 50;
%% simulation
out = sim("FL_simulink.slx");
x_t = out.x.data;
y_t = out.y.data;
z_t = out.z.data;
phi_t = out.phi.data;
theta_t = out.theta.data;
psi_t = out.psi.data;
x_ref = out.x_ref.data;
y_ref = out.y_ref.data;
z_ref = out.z_ref.data;
u1 = out.u1.data;
u2 = out.u2.data;
u3 = out.u3.data;
u4 = out.u4.data;
t = out.tout;
%% trajectory of UAV
figure(1);
plot3(x_t, y_t, z_t, 'LineWidth',1,'Color','black');
title("Trajectory of UAV with FL controller");
xlabel("x (meter)");
ylabel("y (meter)");
zlabel("z (meter)");
xlim([0 20]);
ylim([-10 10]);
zlim([0 20]);
yticks([-10 -5 0 5 10]);
xticks([0 5 10 15 20]);
zticks([0 5 10 15 20]);
hold on
plot3(10,10,20,"o",'Color','red');
plot3(20,-10,10,"diamond",'Color','red');
plot3(0,0,0,"pentagram",'Color','red');
legend("Trajectory of UAV", "Initial position", "Waypoint", "Landing point");
grid on;
%% x(t)
figure(2);
plot(t, x_t, 'LineWidth',1,'Color','black');
xlabel("time (sec)");
ylabel("x (meter)");
xticks([0 10 20 30]);
yticks([0 5 10 15 20]);
ylim([-1 21]);
hold on
plot(t, x_ref,'LineWidth',0.5,'Color','red');
legend("x(t)", "reference")
title("x position with desired value");
grid on;
%% y(t)
figure(3);
plot(t, y_t, 'LineWidth',1,'Color','black');
xlabel("time (sec)");
ylabel("y (meter)");
xticks([0 10 20 30]);
yticks([-10 -5 0 5 10]);
ylim([-11 11]);
hold on
plot(t, y_ref,'LineWidth',0.5,'Color','red');
legend("y(t)", "reference")
title("y position with desired value");
grid on;
%% z(t)
figure(4);
plot(t, z_t, 'LineWidth',1,'Color','black');
xlabel("time (sec)");
ylabel("z (meter)");
xticks([0 10 20 30]);
yticks([0 5 10 15 20]);
ylim([-1 21]);
hold on
plot(t, z_ref,'LineWidth',0.5,'Color','red');
legend("z(t)", "reference")
title("z position with desired value");
grid on;
%% phi(t)
figure(5);
plot(t, phi_t, 'LineWidth',1,'Color','black');
xlabel("time (sec)");
ylabel("phi (rad)");
xticks([0 10 20 30]);
yticks([-0.7 -0.35 0 0.35 0.7]);
ylim([-0.7 0.7]);
hold on
legend("phi(t)")
title("roll angle");
grid on;
%% theta(t)
figure(6);
plot(t, theta_t, 'LineWidth',1,'Color','black');
xlabel("time (sec)");
ylabel("theta (rad)");
xticks([0 10 20 30]);
yticks([-0.7 -0.35 0 0.35 0.7]);
ylim([-0.7 0.7]);
hold on
legend("theta(t)")
title("pitch angle");
grid on;
%% psi(t)
figure(7);
plot(t, psi_t, 'LineWidth',1,'Color','black');
xlabel("time (sec)");
ylabel("psi (rad)");
xticks([0 10 20 30]);
yticks([-0.7 -0.35 0 0.35 0.7]);
ylim([-0.7 0.7]);
hold on
legend("psi(t)")
title("yaw angle");
grid on;
%% u1
figure(8);
plot(t, u1,'LineWidth',1,'Color','black');
ylim([-20 20]);
xlabel("time (sec)");
ylabel("u1 (rad/s^2)");
title("u1");
grid on;
%% u2
figure(9);
plot(t, u2,'LineWidth',1,'Color','black');
ylim([-20 20]);
xlabel("time (sec)");
ylabel("u2 (rad/s^2)");
title("u2");
grid on;
%% u3
figure(10);
plot(t, u3,'LineWidth',1,'Color','black');
ylim([-20 20]);
xlabel("time (sec)");
ylabel("u3 (rad/s^2)");
title("u3");
grid on;
%% u4
figure(11);
plot(t, u4,'LineWidth',1,'Color','black');
ylim([-20 20]);
xlabel("time (sec)");
ylabel("u4 (rad/s^2)");
title("u4");
grid on;