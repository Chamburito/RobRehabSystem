%% Programa de definição do Robo

%% Definição dos parâmetros de Denavit Hattenberg
twolink_dh = [
% alpha A	theta	D	sigma	m	rx	ry	rz	Ixx	Iyy	Izz	Ixy	Iyz	Ixz	Jm	G
  0     0.152         0     0         0     1       1       0       0       0       0       0       0       0       0        0      1
  0     0.1         0     0         0     1       1       0       0       0       0       0       0       0       0        0      1
  0     0         0     0         0     1       1       0       0       0       0       0       0       0       0        0      1
];


%% Definição do manipulador
SCARA = ROBOT(twolink_dh);


