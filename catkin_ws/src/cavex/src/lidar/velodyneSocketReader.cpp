#define M_PI 3.14159265358979323846

#include "velodyneSocketReader.h"
#include <chrono>
#include <cmath>
#include <errno.h>
#include <string>

velodyneSocketReader::velodyneSocketReader() {
    PORT = 2368;
    address.sin_family = AF_INET;
    // address.sin_addr.s_addr = inet_addr("192.168.1.201");
    // address.sin_addr.s_addr = inet_addr("169.254.154.190");
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
};

void velodyneSocketReader::connect() {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        std::cout << "Socket creation failed\n";
        exit(EXIT_FAILURE);
        return;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    int bindStatus = bind(sockfd, (struct sockaddr *)&address, sizeof(address));

    if(bindStatus < 0) {
        std::cout << "Bind failed\n";
        exit(EXIT_FAILURE);
        return;
    }

    recvfrom(sockfd, buffer, 1248, 0, (struct sockaddr *)&address, (socklen_t*)&addrlen);
    printf("Data received: %s\n", buffer);

    socketID = accept(sockfd, (struct sockaddr *)&address, (socklen_t*)&addrlen);

    if(socketID < 0) {
        std::cout << "Accept failed\n";
        std::cout << "Err: " << socketID << "\n";
        std::cout << "errno: " << errno << "\n";
        std::cout << "errnostr: " << strerror(errno) << "\n";
        exit(EXIT_FAILURE);
        return;
    }

    valRead = read(socketID, buffer, 1248);
    printf("%s\n", buffer);
    

}

void velodyneSocketReader::disconnect() {

}