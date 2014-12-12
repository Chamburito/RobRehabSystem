#ifndef CLIENT_H
#define CLIENT_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

int* socket_list = NULL;
int n_sockets = 0;

int open_client_socket( const char* address, const char* port )
{
    socket_list = (int*)realloc( socket_list, (n_sockets + 1) * sizeof(int) );  
  
    struct addrinfo hints, *servinfo;
    
    memset( &hints, 0, sizeof(hints) );
    hints.ai_family = AF_UNSPEC;
    //hints.ai_socktype = SOCK_DGRAM;
    hints.ai_socktype = SOCK_STREAM;
    
    int rv;
    if( (rv = getaddrinfo( address, port, &hints, &servinfo )) != 0 )
        fprintf( stderr, "getaddrinfo: %s\n", gai_strerror(rv) );
      
    // loop through all the results and connect to the first we can
    char s[INET6_ADDRSTRLEN];
    struct addrinfo *p;
    for( p = servinfo; p != NULL; p = p->ai_next ) 
    {  
	if( (socket_list[ n_sockets ] = socket( p->ai_family, p->ai_socktype, p->ai_protocol )) == -1 ) 
	{
	    perror( "client: socket" );
	    continue;
	}

	if( ((struct sockaddr *)p->ai_addr)->sa_family == AF_INET )
	    inet_ntop( p->ai_family, &(((struct sockaddr_in*)p->ai_addr)->sin_addr), s, sizeof s );
	else
	    inet_ntop( p->ai_family, &(((struct sockaddr_in6*)p->ai_addr)->sin6_addr), s, sizeof s );
	printf( "client: connecting to %s\n", s );
	  
	if( connect( socket_list[ n_sockets ], p->ai_addr, p->ai_addrlen ) == -1 ) 
	{
	    close( socket_list[ n_sockets ] );
	    perror( "client: connect" );
	    continue;
	}

	break;
    }
    
    freeaddrinfo( servinfo ); // all done with this structure
      
    if( p == NULL )
    {
	fprintf( stderr, "client: failed to connect" );
	return -1;
    }
    
    return n_sockets++;
}

void send_buffer( int socket_index, const char* buffer )
{
    if( socket_index > n_sockets - 1 )
        fprintf( stderr, "invalid socket index: %d\n", socket_index );
    else
    {
        if( send( socket_list[ socket_index ], buffer, strlen(buffer), 0 ) == -1 )
            perror("client: send");
    }
    
    return;
}

char* receive_buffer( int socket_index )
{
    static char buffer[1024];
    static int n_bytes;
  
    if( socket_index > n_sockets - 1 )
    {
        fprintf( stderr, "invalid socket index: %d\n", socket_index );
	return NULL;
    }
    else
    {
        if( (n_bytes = recv( socket_list[ socket_index ], buffer, sizeof(buffer), 0 )) == -1 )
	    perror("client: recv");

	buffer[ n_bytes ] = '\0';
    }
    
    return buffer;
}

void close_connections()
{
    int index;
    for( index = 0; index < n_sockets; index++ )
        close( socket_list[ index ] );
    
    return;
}

#endif //CLIENT_H