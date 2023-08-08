#include "velodyneUtils.h"
#include "velodyneSocketReader.h"


// Returns the vertical angle of a laser relative to the VLP16's horizon in radians from a laser ID/channel ID
double getLaserAngleFromChannelID(int channelID) {
    if(channelID < 1 || channelID > 32) return 0; // return 0 if the channel ID is invalid
    else return laserChannelAngles[channelID - 1];
}

// Returns the delay in seconds from the first laser firing to the specified laser firing
// - The total time for all 16 lasers for fire is 55.296us
// - The time between each laser firing is 2.304us
// - There is an 18.432us idle time after the last laser firing in a sequence (happens twice per data block)
double getLaserChannelTiming(int channelID, int dataBlockID) {
    if(channelID < 1 || channelID > 32) return 0; // return 0 if the channel ID is invalid
    if(dataBlockID < 1 || dataBlockID > 12) return 0; // return 0 if the data block ID is invalid
    else return (55.296*1e-6*dataBlockID) + (2.304*1e-6*channelID);
}

void parsePacketToPointCloud(char *packet, pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud) {

}

void parsePacketToDataBlocks(char *packet, std::vector<sock_velodyneVLP16DataBlock> &dataBlocks) {
    // 1. Find start of first packet (0xFFEE flag)
    // 2. 

    // int byteIndex = 0;
    bool ffFlag = false;
    unsigned int firstPacketInBlockTimestamp = 0;
    int packetSize = sizeof(packet) / 8;

    for(int byteIndex = 0; byteIndex < packetSize; byteIndex++) {
        if(packet[byteIndex] == '\xFF') {
            ffFlag = true;
        }
        else if(packet[byteIndex] == '\xEE') {
            if(ffFlag) { // if ffFlag is true then we must be at the start of a datablock (0xFFEE)
                ffFlag = false;

                if(byteIndex - 2 > -1 && byteIndex - 3 > -1 && packet[byteIndex-2] == '\x00' && packet[byteIndex-3] == '\x00') {
                    sock_velodyneVLP16Packet curPacket;

                    for(int datablock = 0; datablock < 12; datablock++) { // loop through each data block in the packet
                        if(datablock > 0) byteIndex += 4; // add 4 to byteIndex to get to the 0xEE byte (adding 3 to get from first dist byte of last channel to 0xFF byte of 0xFFEE bytes, then add another 1 to get to the 0xEE byte)
                    }
                }
            }
        }
        else if(ffFlag) {
        }
    }
}

void parseFrameToPointCloud(char *frame, pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud) {

}

void parseFrameToDataBlocks(char *frame, std::vector<sock_velodyneVLP16DataBlock> &dataBlocks) {
    // 1. Split frame into packets and store in vector
    // 2. Loop through vector ad run parsePacketToDataBlocks on each packet
}