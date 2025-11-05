#include "udp_client.h"

UdpClient::UdpClient(const std::string& ipAddress, int port)
    :m_ipAddress(ipAddress), m_port(port), m_initialized(false){

#ifdef _WIN32

        int isResult = WSAStartup(MAKEWORD(2,2), &m_wsaData);
        if(isResult != 0){
            std::cerr<<"WAStartup faild: "<<isResult<<std::endl;
            return;
        }
#endif

#ifdef _WIN32
        m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if(m_sockfd==INVALID_SOCKET){
            std::cerr << "Socket creation failed with error: " << WSAGetLastError() << std::endl;
            return;
        }
#else
       m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if(m_sockfd==INVALID_SOCKET){
            perror("Socket creation failed with error: ");
            return;
        } 
#endif

        m_servaddr.sin_family = AF_INET;
        m_servaddr.sin_port = htons(m_port);
#ifdef _WIN32
        if (InetPton(AF_INET, m_ipAddress.c_str(), &m_servaddr.sin_addr) != 1) {
        std::cerr << "InetPton failed for IP: " << m_ipAddress << std::endl;
        return;
    }
#else
        if (inet_pton(AF_INET, m_ipAddress.c_str(), &m_servaddr.sin_addr) <= 0) {
        perror("Invalid address/ Address not supported");
        return;
#endif

        m_initialized = true;
        std::cout << "UDP Client initialized for " << m_ipAddress << ":" << m_port << std::endl;

    }

UdpClient::~UdpClient(){
    if(m_initialized){
#ifdef _WIN32
        closesocket(m_sockfd);
        WSACleanup();
#else 
        close(m_sockfd);
#endif
        std::cout<<"UDP Cliend cleanned up"<<std::endl;
    }
}

bool UdpClient::sendCommand(char cmd){
    if (!m_initialized) {
        std::cerr << "UDP Client not initialized. Cannot send command." << std::endl;
        return false;
    }
#ifdef _WIN32
    int byteSent = sendto(m_sockfd, &cmd, 1, 0, (struct sockaddr*)&m_servaddr,sizeof(m_servaddr));
    if(byteSent == SOCKET_ERROR){
        std::cerr<<"sendto failed with error: "<<WSAGetLastError() << std::endl;
        return false;
    }
#else
    ssize_t byteSent = sendto(m_sockfd, &cmd, 1, 0, (const struct sockaddr*)&m_servaddr, sizeof(servaddr));
    if(byteSend <0){
        perror("sendto failed.");
        return false;
    }
#endif
    return true;
}