close all
clear all
clc

load tempo.dat
load omega_lf.dat
load omega_l.dat
load torque_lf.dat
load torque_d.dat
load theta_l.dat

d=6000;

figure
plot(5e-3*tempo(1:d),omega_l(1:d),'r')
hold on
plot(5e-3*tempo(1:d),omega_lf(1:d),'--b')
plot(5e-3*tempo(1:d),torque_d(1:d)/10,'--m')

figure
plot(5e-3*tempo(1:d),torque_d(1:d),'--m')
hold on
plot(5e-3*tempo(1:d),torque_lf(1:d),'--k')

% figure
% plot(5e-3*tempo(1:d),theta_l(1:d),'r')