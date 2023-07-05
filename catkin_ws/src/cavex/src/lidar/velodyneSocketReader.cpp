#define M_PI 3.14159265358979323846

#include "velodyneSocketReader.h"
#include <chrono>
#include <cmath>

velodyneSocketReader::velodyneSocketReader() {
    PORT = 2368;
    address.sin_family = AF_INET;
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

    socketID = accept(sockfd, (struct sockaddr *)&address, (socklen_t*)&addrlen);

    if(socketID < 0) {
        std::cout << "Accept failed\n";
        exit(EXIT_FAILURE);
        return;
    }

    valRead = read(socketID, buffer, 1248);
    printf("%s\n", buffer);
    

}