#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define MAXHOSTNAME 256
using namespace std;

main()
{
    // Communication Config
    const int portNumber = 5995;
    string ip_address = "161.253.66.39";


    // Create socket
    int socketHandle;
    if((socketHandle = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        cerr << "Server fail to create the socket." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        close(socketHandle);
        exit(EXIT_FAILURE);
   }


    // Load system information into socket data structures
    struct sockaddr_in socketInfo;
    bzero(&socketInfo, sizeof(sockaddr_in));  // Clear structure memory
    socketInfo.sin_family = AF_INET;
    socketInfo.sin_addr.s_addr = inet_addr(ip_address.c_str());
    socketInfo.sin_port = htons((u_short)portNumber);


    // Bind the socket to a local socket address
    if( bind(socketHandle, (struct sockaddr *) &socketInfo, sizeof(socketInfo)) < 0)
    {
        cerr << "Server fail to bind the socket." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        close(socketHandle);
        exit(EXIT_FAILURE);
    }


    // Wait for a client to connect
    if (listen(socketHandle, 1) < 0)
    {
        cerr << "Server fail to listen for a connection." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        close(socketHandle);
        exit(EXIT_FAILURE);
    }


    // Accept incoming connection
    int socketConnection;
    if( (socketConnection = accept(socketHandle, NULL, NULL)) < 0)
    {
        cerr << "Server fail to accept client connection." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        close(socketHandle);
        exit(EXIT_FAILURE);
    }
    close(socketHandle);


   // Send data
   char buf[80 * 60 * 2];
   int rc = 0;
   memset(buf, 1, sizeof(buf));

   int count = 0;
   while (count < 2)
   {
       // Receive Request
       rc = recv(socketConnection, buf, sizeof(buf), 0);
       cout << "Number of bytes read: " << rc << endl;
       cout << "Received: ";
       for (int i = 0; i < rc; i++)
           cout << (int)buf[i];

       // Send response
       cout << "Send Message. " << endl;
       send(socketConnection, buf, sizeof(buf), 0);
       cout << "Mesage sent! " << endl;

       count++;
   }

}

