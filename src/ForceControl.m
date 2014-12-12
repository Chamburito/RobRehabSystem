close all
clear all
clc

load tempo.dat

load torque_m.dat

load Im.dat

load controle.dat
load controle2.dat


load theta_m.dat
load theta_c.dat
load theta_l.dat
load theta_ld.dat

load torque_d.dat
load torque_l.dat
load torque_lf.dat

load omega_l.dat
load omega_lf.dat

load omega_m.dat


figure
plot(5e-3*tempo(1:100),floor(controle(1:100)),'r',5e-3*tempo(1:100),floor(0.7*controle2(1:100)))
grid


figure
plot(5e-3*tempo(1:100),torque_d(1:100),'r',5e-3*tempo(1:100),torque_lf(1:100))
grid

%  
% data = clock;
% arq = [ num2str(data(1)) '_' num2str(data(2)) '_' num2str(data(3)) '_' num2str(data(4)) '_' num2str(data(5)) '.mat']
% 
% eval(['save ',arq])