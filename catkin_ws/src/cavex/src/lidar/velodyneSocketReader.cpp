#define M_PI 3.14159265358979323846

#include "velodyneSocketReader.h"
#include <cmath>
#include <errno.h>
#include <string>
#include <fcntl.h>
#include <sstream>
#include <poll.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "floam_cpu/laserProcessingClass.h"
#include "object_detection_cpu/objPointCloudProcessor.h"
#include "object_detection_cpu/objRansac.h"
#include "object_detection_cpu/objCluster.h"
#include "object_detection_cpu/objRender.h"
#include "object_detection_cpu/objBox.h"

#include "velodyneUtils.h"

velodyneSocketReader::velodyneSocketReader() {
    PORT = 2368;
    address.sin_family = AF_INET;
    memset(&address, 0, sizeof(address));
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
};



// void velodyneSocketReader::connect(std::array<char, FRAME_SIZE_BYTES> &frameBuffer, std::array<std::array<char, FRAME_SIZE_BYTES>, MAX_FRAME_BUFFER_QUEUE_SIZE_BYTES> &frameBufferQueue) {
void velodyneSocketReader::connect(std::vector<char> &frameBuffer, std::vector<std::vector<char>> &frameBufferQueue) {

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL Visualiser"));
    viewer->setBackgroundColor(0,0,0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0,16,0,0,0,1);
    lastPacketTimestamp = std::chrono::high_resolution_clock::now();

    sockfd = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        std::cout << "[velodyneSocketReader.cpp] Socket creation failed\n";
        exit(EXIT_FAILURE);
        return;
    }

    int val = 1;
    if(setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) == -1) {
        std::cout << "[velodyneSocketReader.cpp] setsockopt failed\n";
        perror("socketopt");
        exit(EXIT_FAILURE);
        return;
    };

    int bindStatus = bind(sockfd, (struct sockaddr *)&address, sizeof(address));

    if(bindStatus < 0) {
        std::cout << "[velodyneSocketReader.cpp] Bind failed\n";
        perror("bind");
        exit(EXIT_FAILURE);
        return;
    }

    if(fcntl(sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        std::cout << "[velodyneSocketReader.cpp] fcntl failed\n";
        perror("non-block");
        exit(EXIT_FAILURE);
        return;
    }

    std::cout << "[velodyneSocketReader.cpp] sockfd: " << sockfd << "\n";
    std::cout << "[velodyneSocketReader.cpp] bindStatus: " << bindStatus << "\n";

    // getPacket
    struct pollfd fds[1];
    fds[0].fd = sockfd;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000;

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    int packetCounter = 0;
    int arrayIndexTracker = 0; // counts from 0 to 94036 (94037 bytes of data in a frame)
    int frameBufferQueueArrayIndexTracker = 0; // Counts from 0 to 50 (50 frames in the queue) then gets reset to zero to start overwriting the oldest frame in the queue

    lastPacketTimestamp = std::chrono::high_resolution_clock::now();


    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>()); // new point cloud to store the edge points from the frame

    while(true) {

        do {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if(retval < 0) {
                if(errno != EINTR) {
                    std::cout << "[velodyneSocketReader.cpp] poll() error: " << strerror(errno) << "\n";
                }
                return;
            }
            else if(retval == 0) {
                std::cout << "[velodyneSocketReader.cpp] poll() timeout\n";
                return;
            }
            if((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) {
                std::cout << "[velodyneSocketReader.cpp] poll() reports Velodyne device error\n";
                return;
            }
        } while((fds[0].revents & POLLIN) == 0);

        ssize_t nbytes = recvfrom(sockfd, buffer, 1248, 0, (sockaddr*) &sender_address, &sender_address_len);

        if(nbytes < 0) {
            if(errno != EWOULDBLOCK) {
                std::cout << "[velodneSocketReader.cpp] recvfail: " << strerror(errno) << "\n";
                return;
            }
        } else if((size_t) nbytes == 1206) {
            packetCounter++;
            std::cout << "[velodyneSocketReader.cpp] Got full velodyne packet #" << packetCounter << "\n";
            // parsePacketToPointCloud(packetBuffer, pointCloud);
            // if(sender_address.sin_addr.s_addr != "192.168.1.201") continue;
            // else break;
            // break;
            
            // std::stringstream ss;
            // ss << std::hex << std::setfill('0');
            // for (int i = 0; i < 1206; ++i) {
            //     ss << std::setw(2) << static_cast<unsigned>(buffer[i]) << " ";
            //     if(arrayIndexTracker > 94036) { // hacky way of getting a frame (94037 bytes should be 100ms of data - VLP-16 manual reports data rate of 940368 bytes/sec -> Hence array index goes up to 94036)
            //         frameBuffer[arrayIndexTracker] = buffer[i];
            //         arrayIndexTracker++;
            //     }
            //     if(frameBufferQueue.size() > 1000) frameBufferQueue.front();
            //     // frameBuffer.push_back(buffer[i]);
            // }

            // TESTING: Storing data using std::array
            // for (int i = 0; i < 1206; ++i) {
            //     if(arrayIndexTracker < 94036) {
            //         frameBuffer[arrayIndexTracker] = buffer[i];
            //         arrayIndexTracker++;
            //     } else {
            //         if(frameBufferQueueArrayIndexTracker > 50) frameBufferQueueArrayIndexTracker = 0;
            //         arrayIndexTracker = 0;
            //         frameBufferQueue[frameBufferQueueArrayIndexTracker] = frameBuffer;
            //         frameBufferQueueArrayIndexTracker++;
            //     }
            // }

            // if(frameBufferQueueArrayIndexTracker == 50) {
            //     auto VEC_TEST_T2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastPacketTimestamp);
            //     std::cout << "duration: " << VEC_TEST_T2.count() << "ms\n";
            //     break;
            // }

            // auto ARRAY_TEST_T2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now());
            // END TESTING: Storing data using std::array

            // TESTING: Storing data using std::vector
            // auto VEC_TEST_T1 = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < 1206; ++i) {
                if(frameBuffer.size() > 94036) { // hacky way of getting a frame (94037 bytes should be 100ms of data - VLP-16 manual reports data rate of 940368 bytes/sec -> Hence array index goes up to 94036)
                    frameBufferQueue.push_back(frameBuffer);
                    frameBufferQueueArrayIndexTracker++;
                    parseFrameToPointCloud(frameBufferQueue.front(), pc);
                    frameBuffer.clear();
                    
                }
                if(frameBufferQueue.size() > 50) frameBufferQueue.erase(frameBufferQueue.begin());
                frameBuffer.push_back(buffer[i]);
            }

            if(frameBufferQueueArrayIndexTracker == 50) {
                auto VEC_TEST_T2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastPacketTimestamp);
                std::cout << "duration: " << VEC_TEST_T2.count() << "ms\n";
                break;
            }

            if(pc->size() > 29000) {
                std::cout << "pointCloud size: " << pc->size() << "\n";
                viewer->updatePointCloud<pcl::PointXYZI>(pointCloud, "Frame 1");
                viewer->spinOnce(100);
            }

            // END TESTING: Storing data using std::vector

            // if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastPacketTimestamp).count() > 100) {
            //     std::cout << "time > 100ms\n";
            //     lastPacketTimestamp = std::chrono::high_resolution_clock::now();
                
            //     // TESTING: VISUALISATION
            //     // pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>);
            //     pointCloud->clear();
            //     // parsePacketToPointCloud(packetBuffer, pointCloud);
            //     // packetBuffer.clear();
            //     // viewer->removeAllPointClouds();
            //     viewer->updatePointCloud<pcl::PointXYZI>(pointCloud, "Frame 1");
            //     // END TESTING: VISUALISATION
            // }

            // std::cout << "[velodyneSocketReader.cpp] Packet: " << ss.str() << "\n";
        } else {
            std::cout << "[velodyneSocketReader.cpp] Incomplete velodyne packet read: " << nbytes << " bytes\n";
        }
    } 

    // if(socketID < 0) {
    //     std::cout << "Accept failed\n";
    //     std::cout << "Err: " << socketID << "\n";
    //     std::cout << "errno: " << errno << "\n";
    //     std::cout << "errnostr: " << strerror(errno) << "\n";
    //     exit(EXIT_FAILURE);
    //     return;
    // }

    // valRead = read(socketID, buffer, 1248);
    printf("%s\n", buffer);
    

}

void velodyneSocketReader::disconnect() {
    close(sockfd);
    std::cout << "[velodyneSocketReader.cpp] Socket closed\n";
}