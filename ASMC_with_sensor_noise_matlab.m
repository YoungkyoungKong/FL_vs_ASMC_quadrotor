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
% ground effect
A = 0.4668;
z0 = 2;
% ASMC
C = [5 5 5 1 1 1];
K = diag([1 1 0.7 5 5 10]);
kp = -5;
kd = -3;
ki = -3;
c1 = C(1);
c2 = C(2);
c3 = C(3);
c4 = C(4);
c5 = C(5);
c6 = C(6);
k1 = K(1,1);
k2 = K(2,2);
k3 = K(3,3);
k4 = K(4,4);
k5 = K(5,5);
k6 = K(6,6);
tau = 0.02;
Ts = 0.002;
var_pos = 0.0025;
var_ang = 0.0001;
%% simulation
out = sim("ASMC_with_sensor_noise_simulink.slx");
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
u5 = out.u5.data;
u6 = out.u6.data;
g_r = out.g_r.data;
u5_hat = out.u5_hat.data;
u6_hat = out.u6_hat.data;
g_r_hat = out.g_r_hat.data;
noise = out.noise.data;
%% trajectory of UAV
figure(1);
plot3(x_t, y_t, z_t, 'LineWidth',1,'Color','black');
title("Trajectory of UAV with ASMC controller with sensor noise");
xlabel("x (meter)");
ylabel("y (meter)");
zlabel("z (meter)");
xlim([0 21]);
ylim([-11 10]);
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
hold on
legend("phi(t)")
title("roll angle");
grid on;
%% theta(t)
figure(6);
plot(t, theta_t, 'LineWidth',1,'Color','black');
xlabel("time (sec)");
ylabel("theta (rad)");
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
ylim([-25 50]);
xlabel("time (sec)");
ylabel("u1 (rad/s^2)");
title("u1");
grid on;
%% u2
figure(9);
plot(t, u2,'LineWidth',1,'Color','black');
xlabel("time (sec)");
ylabel("u2 (rad/s^2)");
title("u2");
grid on;
%% u3
figure(10);
plot(t, u3,'LineWidth',1,'Color','black');
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
%% u5
figure(12);
plot(t, u5_hat, 'LineWidth',1,'Color','black');
xlabel("time (sec)");
ylabel("u5 hat");
hold on
plot(t, u5,'LineWidth',0.5,'Color','red');
legend("u5 hat", "u5")
title("u5 hat with real value");
grid on;
%% u6
figure(13);
plot(t, u6_hat, 'LineWidth',1,'Color','black');
xlabel("time (sec)");
ylabel("u6 hat");
hold on
plot(t, u6,'LineWidth',0.5,'Color','red');
legend("u6 hat", "u6")
title("u6 hat with real value");
grid on;
%% g_r
figure(14);
plot(t, g_r_hat, 'LineWidth',1,'Color','black');
xlabel("time (sec)");
ylabel("g_r hat");
hold on
plot(t, g_r,'LineWidth',0.5,'Color','red');
legend("g_r hat", "g_r")
title("g_r hat with real value");
grid on;
%% filter
sys = tf(1,[tau 1]);
bode(sys);
grid on;