#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
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
        cerr << "Client fail to create the socket." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        close(socketHandle);
        exit(EXIT_FAILURE);
   }


    // Load system information into socket data structures
    struct sockaddr_in remoteSocketInfo;
    bzero(&remoteSocketInfo, sizeof(sockaddr_in));  // Clear structure memory
    remoteSocketInfo.sin_family = AF_INET;
    remoteSocketInfo.sin_addr.s_addr = inet_addr(ip_address.c_str());
    remoteSocketInfo.sin_port = htons((u_short)portNumber);


    // Establish the connection with the server
    if(connect(socketHandle, (struct sockaddr *)&remoteSocketInfo, sizeof(sockaddr_in)) < 0)
    {
        cerr << "Client fail to connect to the server." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        close(socketHandle);
        exit(EXIT_FAILURE);
    }

    // Send data
    char buf[80 * 60 * 2];
    int rc = 0;
    memset(buf, 1, sizeof(buf));

    int count = 0;
    while (1)
    {
        // Send Request
        cout << "Send Message. " << endl;
        send(socketHandle, buf, sizeof(buf), 0);
        cout << "Mesage sent! " << endl;

        // Receive Msg
        rc = recv(socketHandle, buf, 80 * 60 * 2, 0);
        cout << "Number of bytes read: " << rc << endl;
        //cout << "Message: " << buf << endl;

        count++;
    }

}
