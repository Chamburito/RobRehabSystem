#ifndef CLIENT_H
#define CLIENT_H

#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#pragma comment(lib, "Ws2_32.lib")

typedef struct
{

	SOCKET sockfd;
	addrinfo* info;

} Connection;

SOCKET* socket_list = NULL;
//Connection* conn_list = NULL;
int n_sockets = 0;

int open_client_socket( const char* address, const char* port/*, int type*/ )
{
    socket_list = (SOCKET*)realloc( socket_list, (n_sockets + 1) * sizeof(SOCKET) );
	//conn_list = (Connection*)realloc( conn_list, (n_sockets + 1) * sizeof(Connection) );
  
    struct addrinfo hints;
	struct addrinfo* servinfo = NULL;

	// Initialize Winsock
	int iResult;
	WSADATA wsaData;
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed: %d\n", iResult);
        return 1;
    }
    
    ZeroMemory( &hints, sizeof(hints) );
    hints.ai_family = AF_UNSPEC;;
    hints.ai_socktype = SOCK_STREAM;//type; // SOCK_STREAM ou SOCK_DGRAM
	//hints.ai_protocol = IPPROTO_TCP;
    
	int rv;
    if( (rv = getaddrinfo( (PCSTR)address, (PCSTR)port, &hints, &servinfo )) != 0 )
        fprintf( stderr, "getaddrinfo: %s\n", gai_strerror(rv) );
      
    // loop through all the results and connect to the first we can
    char s[INET6_ADDRSTRLEN];
    struct addrinfo *p;
    for( p = servinfo; p != NULL; p = p->ai_next ) 
    {  
		if( (socket_list[ n_sockets ] = socket( p->ai_family, p->ai_socktype, p->ai_protocol )) == INVALID_SOCKET ) 
		//if( (conn_list[ n_sockets ].sockfd = socket( p->ai_family, p->ai_socktype, p->ai_protocol )) == INVALID_SOCKET ) 
		{
			fprintf( stderr, "client: socket: %s\n", WSAGetLastError() );
			continue;
		}

		if( ((struct sockaddr *)p->ai_addr)->sa_family == AF_INET )
			InetNtop( p->ai_family, &(((struct sockaddr_in*)p->ai_addr)->sin_addr), (PTSTR)s, sizeof s );
		else
			InetNtop( p->ai_family, &(((struct sockaddr_in6*)p->ai_addr)->sin6_addr), (PTSTR)s, sizeof s );
		printf( "client: connecting to %s\n", s );
	  
		if( connect( socket_list[ n_sockets ], p->ai_addr, p->ai_addrlen ) == SOCKET_ERROR ) 
		//if( connect( conn_list[ n_sockets ].sockfd, p->ai_addr, p->ai_addrlen ) == SOCKET_ERROR ) 
		{
			closesocket( socket_list[ n_sockets ] );
			//closesocket( conn_list[ n_sockets ].sockfd );
			fprintf( stderr, "client: connect: %s\n", WSAGetLastError() );
			continue;
		}

		break;
    }
      
    if( p == NULL )
    {
		fprintf( stderr, "client: failed to connect" );
		freeaddrinfo( servinfo ); // all done with this structure
		return -1;
    }
    
	//conn_list[ n_sockets ].info = p;

    return n_sockets++;
}

void send_buffer( int socket_index, const char* buffer )
{
    if( socket_index > n_sockets - 1 || socket_index < 0 )
        fprintf( stderr, "invalid socket index: %d\n", socket_index );
    else
    {
		//if( conn_list[ socket_index ].info->ai_socktype == SOCK_STREAM )
		//{
			if( send( socket_list[ socket_index ], buffer, strlen(buffer), 0 ) == SOCKET_ERROR )
			//if( send( conn_list[ socket_index ].sockfd, buffer, strlen(buffer), 0 ) == SOCKET_ERROR )
				fprintf( stderr, "client: send: %s\n", WSAGetLastError() );
		//}
		//else
		//{
		//	if( sendto( conn_list[ socket_index ].sockfd, buffer, strlen(buffer), 0, 
		//				conn_list[ socket_index ].info->ai_addr, conn_list[ socket_index ].info->ai_addrlen ) == SOCKET_ERROR )
		//		fprintf( stderr, "client: sendto: %s\n", WSAGetLastError() );
		//}
    }
    
    return;
}

char* receive_buffer( int socket_index )
{
    static char buffer[1024];
    static int n_bytes;
	static struct sockaddr_storage their_addr;
	static socklen_t addr_len;
  
    if( socket_index > n_sockets - 1 || socket_index < 0 )
    {
        fprintf( stderr, "invalid socket index: %d\n", socket_index );
		return NULL;
    }
    else
    {
		//if( conn_list[ socket_index ].info->ai_socktype == SOCK_STREAM )
		//{
			if( (n_bytes = recv( socket_list[ socket_index ], buffer, sizeof(buffer), 0 )) == SOCKET_ERROR )
			//if( (n_bytes = recv( conn_list[ socket_index ].sockfd, buffer, sizeof(buffer), 0 )) == SOCKET_ERROR )
				fprintf( stderr, "client: recv: %s\n", WSAGetLastError() );
		//}
		//else
		//{
			//if( (n_bytes = recvfrom( conn_list[ socket_index ].sockfd, buffer, sizeof(buffer) , 0,
			//			(struct sockaddr *)&their_addr, &addr_len )) == SOCKET_ERROR )
			//	fprintf( stderr, "client: send: %s\n", WSAGetLastError() );
		//}

		buffer[ n_bytes ] = '\0';
    }
    
    return buffer;
}

void close_connections()
{
    int index;
    for( index = 0; index < n_sockets; index++ )
        closesocket( socket_list[ index ] );
		//closesocket( conn_list[ index ].sockfd );
    
    WSACleanup();
    
    return;
}

#endif //CLIENT_H