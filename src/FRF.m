close all
clear all
clc


load tempo.dat
load torque_lf.dat
load torque_d.dat

plot((5e-3)*tempo(1:100),torque_d(1:100),'r')
grid
hold on
plot((5e-3)*tempo(1:100),torque_lf(1:100))
% axis([0 0.5 0 6])
%%

tf = 10; % tempo final
t = [0:0.005:tf]; % intervalo de tempo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fmin = 0; % frequência mínima
fmax = 5; % frequência máxima
A = 40; % amplitude do sinal
S1 = A*chirp(t,fmin,tf,fmax,'quadratic',90); % chirp (crescente)
S2 = fliplr(S1); % inverte sinal
S3 = [S1 -S2]; % chirp (crescente e decrescente)


S_in=[-S3 -S3 -S3 -S3]';

dados=int32(S_in);


 plot((S_in))
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
clear all

load tempo.dat
load torque_d.dat
load theta_c.dat
load torque_lf.dat
load omega_lf.dat
load controle.dat

T_in = torque_d(1:16008);    % Torque de entrada
T_out = torque_lf(1:16008);   % Torque de saída

N = 4002/2;		% número de pontos dos blocos da amostra
Fs = 200;		% freqüência de amostragem
WIN = N;		% número de pontos da janela HANNING (default)
OVLP = round(2*N/3); 	% número de pontos do "overlap" (1920/2048 = 0.937)
% OVLP = round(1*N/2); 	% número de pontos do "overlap" (1920/2048 = 0.937)

% Correlacao & AutoCorrelacao

AC_in = pwelch(T_in,WIN,OVLP,N,Fs);
AC_out = pwelch(T_out,WIN,OVLP,N,Fs);
C_in_out = cpsd(T_in,T_out,WIN,OVLP,N,Fs);

% Estimadores H1 e H2

H1 = C_in_out./AC_in;

H2 = AC_out./C_in_out;


[C,F] = mscohere(T_in,T_out,WIN,OVLP,N,Fs);


figure (1)
subplot(3,1,1)
semilogx(F,20*log10(abs(H1)))
hold on
semilogx(F,20*log10(abs(H2)),'r')
grid

subplot(3,1,2)
semilogx(F,(180/pi)*(angle(H1)))
grid

subplot(3,1,3)
semilogx(F,C)
grid

% figure (2)
% semilogx(F,abs(H1))
% hold on
% semilogx(F,abs(H2),'r')
% grid

% 
% figure(4)
% plot(theta_l(1:6000))
% hold on
% plot(torque_lf(1:6000),'r')


