#ifndef _CLIENT_H_ //part of header guard
#define _CLIENT_H_ // part of header guard


#ifdef WIN32
#include <windows.h>
#endif

#ifdef WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

#pragma comment(lib,"ws2_32.lib")

#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <conio.h>

#include <cstring>
#include <string>
#include <iostream>

#define MAX_GDLS 20
#define MAX_BUFFER 1024
#define SERVER_PORT 2000
#define DELAY_TIME_MILLIS 50
#define SERVER_ADDRESS "172.16.255.245"

DWORD WINAPI Client(void*);

extern std::string inClientMessage;
extern std::string outClientMessage;

using namespace std;

#endif // _WIN_CLIENT_H_