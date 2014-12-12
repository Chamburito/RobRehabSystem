#include "client.h"

string inClientMessage;
string outClientMessage;


DWORD WINAPI Client(void*) { 

	WSADATA wsa;
	SOCKET s;
	struct sockaddr_in server;
	char *message , server_reply[MAX_BUFFER];
	int recv_size;

	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
	{
		printf("Failed. Error Code : %d",WSAGetLastError());
		return 1;
	}
	
	printf("Initialised.\n");
	
	//Create a socket
	if((s = socket(AF_INET , SOCK_STREAM , 0 )) == INVALID_SOCKET)
	{
		printf("Could not create socket : %d" , WSAGetLastError());
	}

	printf("Socket created.\n");
	
	server.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);
	server.sin_family = AF_INET;
	server.sin_port = htons( SERVER_PORT );

	//Connect to remote server
	if (connect(s , (struct sockaddr *)&server , sizeof(server)) < 0)
	{
		puts("connect error");
		return 1;
	}
	
	printf("Connected\n");
	
	do{
		if((recv_size = recv(s , server_reply , MAX_BUFFER , 0)) == SOCKET_ERROR)
		{
			puts("recv failed");
		}

		inClientMessage = server_reply;
		Sleep(10);

	} while(true);



	//Add a NULL terminating character to make it a proper string before printing
	server_reply[recv_size] = '\0';
	puts(server_reply);

	return 0;
	
	closesocket(s);
	WSACleanup();

}