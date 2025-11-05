#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <string>
#include <iostream>

#ifdef _WIN32
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

class UdpClient {
public:
    UdpClient(const std::string& ipAdress, int port);

    ~UdpClient();

    bool sendCommand(char cmd);

    bool isInitialized() const{return m_initialized;}

private:
    std::string m_ipAddress;
    int m_port;

#ifdef _WIN32
    SOCKET m_sockfd;
    WSADATA m_wsaData;
#else
    int m_sockfd;
#endif
    struct sockaddr_in m_servaddr;
    bool m_initialized;
};

#endif