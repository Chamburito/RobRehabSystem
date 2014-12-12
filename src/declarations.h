#ifndef DECLARATIONS_H
#define	DECLARATIONS_H

#include "Axis.h"
#include "EPOSNetwork.h"

    //ENDEREÇAMENTO DA BASE DE DADOS CAN
    const char *CAN_DATABASE = "database";
    const char *CAN_CLUSTER = "NETCAN";
    
    const char *NET_ID_SERVO_01 = "1";
    const char *NET_ID_SERVO_02 = "2";
    const char *NET_ID_SERVO_03 = "3";

    int COMMAND_KEY = 0;
    int COMMAND_KEY2 = 0;

    int loopTime = 0;

    int i_dt=0;

    double datalog01[60000];
    double datalog02[60000];
    double datalog03[60000];
    double datalog04[60000];
    double datalog05[60000];
    double datalog06[60000];
    double datalog07[60000];
    double datalog08[60000];
    double datalog09[60000];
    double datalog10[60000];
    double datalog11[60000];
    double datalog12[60000];
    double datalog13[60000];
    double datalog14[60000];
    double datalog15[60000];
    double datalog16[60000];
    double datalog17[60000];
    double datalog18[60000];
    double datalog19[60000];
    double datalog20[60000];

    double setpoints[60000];

    const double PI = 3.141592;

    //INICIALIZANDO O QUERY PERFORMANCE PARA CONTROLE DOS CICLOS DE 5MS
    //LARGE_INTEGER tickAfter, tickBefore, TICKS_PER_SECOND;
    //long long int ticksSampleTime, finalTime;
    double finalTime;
    const double SAMPLE_TIME = 0.001;
    int totalTime;
	
    //INICIALIZAÇÃO DA REDE CAN
    EPOSNetwork eposNetwork( CAN_DATABASE, CAN_CLUSTER );

    //INICIALIZAÇÃO DAS EPOS
	const int N_EPOS = 3;
    Axis EPOS[N_EPOS] = { Axis( CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_01 ),
					Axis( CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_02 ),
					Axis( CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_03 ) };

    //Ponteiro para arquivo
    //FILE * pFile;
	
#endif	/* DECLARATIONS_H */