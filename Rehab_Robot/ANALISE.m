clear
clc

load Fwx.dat
load Fwy.dat
load Fwz.dat
load fFwx.dat
load fFwy.dat
load fFwz.dat
load Px.dat
load Py.dat
load Pz.dat
load Setpoint_X.dat
load Setpoint_Y.dat
load Setpoint_Z.dat
load xa.dat
load Ya.dat
load Za.dat

figure(1)
subplot(2,1,1)
plot(fFwx(1:35000), 'b')
subplot(2,1,2)
plot(Setpoint_X(1:35000), 'b');
hold on
grid on
plot(Px(1:35000), 'r');
plot(xa(1:35000), 'g');
plot(Setpoint_X(1:35000)+xa(1:35000), 'k');
