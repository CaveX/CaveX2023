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

/* TO BE TESTED */
// param: *packet - pointer to the buffer containing the packet's raw binary data
// param: &dataBlocks - reference to the vector of data blocks to be populated from the packet buffer (*packet)
void parsePacketToDataBlocks(char *packet, std::vector<sock_velodyneVLP16DataBlock> &dataBlocks) {
    // 1. Find start of first packet (0xFFEE flag)
    // 2. Find azimuth bytes (2 bytes)
    // 3. Convert azimuth bytes to azimuth angle (float)
    // 4. Find distance bytes (2 bytes) and reflectivity byte (1 byte)
    // 5. Convert distance bytes to distance (float) and reflectivity byte to reflectivity (float)
    // 6. Add point to data block
    // 7. Add data block to &dataBlocks

    // int byteIndex = 0;
    bool ffFlag = false;
    unsigned int firstPacketInBlockTimestamp = 0;
    size_t packetSize = sizeof(packet);

    if(packetSize < 1206) return; // return if the packet is too small (i.e. not at least one full packet minus the 42 byte UDP header)

    size_t packetSize = packetSize / 8;

    for(int byteIndex = 0; byteIndex < packetSize; byteIndex++) {
        if(packet[byteIndex] == '\xFF') {
            ffFlag = true; // Found start of packet
        }
        else if(packet[byteIndex] == '\xEE') {
            if(ffFlag) { // if ffFlag is true then we must be at the start of a datablock (0xFFEE)
                ffFlag = false;

                unsigned int timestamp = packet[byteIndex+1201] << 24 | (packet[byteIndex+1200] << 16) | (packet[byteIndex+1199] << 8) | packet[byteIndex+1198]; // Gets the timestamp from the packet

                if(byteIndex - 2 > -1 && byteIndex - 3 > -1 && packet[byteIndex-2] == '\x00' && packet[byteIndex-3] == '\x00') {
                    sock_velodyneVLP16Packet curPacket; // The data block to be populated from the packet buffer (*packet) and added to the packet

                    for(int datablock = 0; datablock < 12; datablock++) { // loop through each data block in the packet
                        if(datablock > 0) byteIndex += 4; // add 4 to byteIndex to get to the 0xEE byte (adding 3 to get from first dist byte of last channel to 0xFF byte of 0xFFEE bytes, then add another 1 to get to the 0xEE byte)
                        unsigned int blockTimestamp = timestamp + datablock*55.296*2; // Stores the time of the first laser firing in a datablock; 55.296us per firing sequence and two firing sequences per data block 
                        sock_velodyneVLP16DataBlock curDataBlock;
                        curDataBlock.azimuth = ((float)(packet[byteIndex+2] << 8 | packet[byteIndex+1])) / 100; // byteIndex+2 and byteIndex+1 are the two bytes which comprise the azimuth (see VLP16 manual)

                        for(int channel = 0; channel < 32; channel++) {
                            byteIndex += 3; // Sets byteIndex to the first byte of the channel+1th channel (e.g if channel is 0, i is the first byte of channel 1)
                            sock_velodyneVLP16Point curPoint;
                            curPoint.distance = ((float)(packet[byteIndex+1] << 8 | packet[byteIndex])) / 500; // divide by 500 to get distance in m (see VLP16 manual)
                            curPoint.reflectivity = (float) packet[byteIndex+2];
                            curDataBlock.points.push_back(curPoint); // Adds the point to the data block 
                        }
                        dataBlocks.push_back(curDataBlock); // Adds the data block to the vector of data blocks
                    }
                    byteIndex += 4; // Add 4 to the byteIndex to get the byte before the last set of distance + reflectivity bytes to the first timestamp byte
                }
            }
        }
    }
}

void parseFrameToPointCloud(char *frame, pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud) {

}

void parseFrameToPackets(char *frame, std::vector<sock_velodyneVLP16Packet> &packets) {

}

void parseFrameToDataBlocks(char *frame, std::vector<sock_velodyneVLP16DataBlock> &dataBlocks) {
    // 1. Split frame into packets and store in vector
    // 2. Loop through vector ad run parsePacketToDataBlocks on each packet
}