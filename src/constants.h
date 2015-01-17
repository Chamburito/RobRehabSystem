#include <math.h>
// Constantes da junta do quadril

const double	p1 = 0.0168;
const double	p2 = 2.5749;

double filtro_analog[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

double analog_filtrado = 0.0;

double V_actual = 0.0;
double V_ref = 0.0;
double P_actual = 0.0;
double P_ref = 0.0;

double Fl = 0.0;

double Ks_1 = 78.9; //	N/m
double passo_AES1 = 3e-3;

const double a_q = 0.063, b_q = 0.060, c_q = 0.284, d_q = 0.117, l_zero_q = 0.348;

double h12_q = c_q * c_q + d_q * d_q;  //onde h12_q=h1_q^2
double h22_q = a_q * a_q + b_q * b_q;  //onde h22_q=h2_q^2
double h1_q = sqrt( h12_q );
double h2_q = sqrt( h22_q );

double alpha_q = atan( a_q / b_q );  //ângulo entre a_quadril e h2
double beta_q = atan( d_q / c_q );  //ângulo entre c_quadril e h1
double gama_q = 0.0, theta_q = 0.0, quadril = 0.0, cos_eta_q = 0.0, sin_eta_q = 0.0;

double l_real_q = 0.0, l_efetuador = 0.0, dthetadl = 0.0, Fv_q = 0.0, thetad_q = 0.0, Vel_d = 0.0;

int encoder_AES1 = 0;

int Vel_d_rpm = 0;

double K_AES1 = 0.0, B_AES1 = 0.0, Kq = 10.0, Bq = 10.0;


/////////////////////////////////////////////////////////////////////////////////////////////////
// Joelho
/////////////////////////////////////////////////////////////////////////////////////////////////

const int N = 150;						  // Redução do sistema
const int ENCODER_IN_RES = 4096;			  // Resolução do encoder do motor
const int ENCODER_OUT_RES = 2000;		      // Resolução do encoder de saída
const int Ks = 104;						  // Constante elástica
const double Ts = 0.005;				  // tempo de amostragem


double kv=0.0;						// rigidez virtual
double bv=0.0;						// amortecimento virtual              

double theta_l = 0.0;                 // posição da carga
	
double theta_m = 0.0;					// posição do motor
double omega_m = 0.0;					// velocidade do motor

double omega = 0.0;

double theta_c = 0.0;                 // posição da coroa

double theta_l_ant = 0.0;				// posição do eixo_out no instante [k-1]
double theta_ld = 0.0;				// velocidade de saída desejada

double Im = 0.0;						// corrente enviada 

double omega_ld = 0.0;				// velocidade de saída desejada
double omega_l = 0.0;					// velocidade de saída
double omega_lf = 0.0;				// velocidade de saída filtrada
double omega_lant = 0.0;				// velocidade de saída no instante [k-1]
double omega_lfant = 0.0;				// velocidade de saída filtrada no instante [k-1]
double omega_lant2 = 0.0;				// velocidade de saída no instante [k-2]
double omega_lfant2 = 0.0;			// velocidade de saída filtrada no instante [k-2]

double a2 = -0.9428;
double a3 = 0.3333;
double b1 = 0.0976;
double b2 = 0.1953;
double b3 = 0.0976;

double torque_l = 0.0;                // torque na carga
double torque_lf = 0.0;				// torque na carga filtrado
double torque_lant = 0.0;				// torque na carga no instante [k-1]
double torque_lfant = 0.0;			// torque na carga filtrado no instante [k-1]
double torque_lant2 = 0.0;			// torque na carga no instante [k-2]
double torque_lfant2 = 0.0;			// torque na carga filtrado no instante [k-2]

double c2 = -1.889;
double c3 = 0.8949;
double d1 = 0.0015;
double d2 = 0.0029;
double d3 = 0.0015;

double torque_d = 0.0;                // torque desejado
double torque_m = 0.0;				// torque no motor

double controle = 0.0;		        // entrada de controle
double controle_ant = 0.0;		    // entrada de controle no instante [k-1]

double controle2 = 0.0;		        // entrada de controle
double controle2_ant = 0.0;		    // entrada de controle no instante [k-1]
double controle2_ant2 = 0.0;

double erro = 0.0;				    // erro
double erro_ant = 0.0;				// erro no instante [k-1]
double erro_ant2 = 0.0;				// erro no instante [k-2]

double kp = 370;				    // Ganho proporcional
double ki = 3.5;					// Ganho integrativo
double kd = 0.0;					// Ganho derivativo
