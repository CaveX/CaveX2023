#define M_PI 3.14159274101257324219

#include "velodynePCAPReader.h"
#include <chrono>
#include <cmath>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "floam_cpu/laserProcessingClass.h"
#include "object_detection_cpu/objPointCloudProcessor.h"
#include "object_detection_cpu/objRansac.h"
#include "object_detection_cpu/objCluster.h"
#include "object_detection_cpu/objRender.h"
#include "object_detection_cpu/objBox.h"

#include "velodyneUtils.h"

    velodynePCAPReader::velodynePCAPReader(std::string absolutePath) : pointCloud(new pcl::PointCloud<pcl::PointXYZI>) {
        this->absolutePath = absolutePath;
    }

    std::string velodynePCAPReader::charToHex(unsigned char charToConvert) {
        if(charToConvert == '\x00') return "00";
        else if(charToConvert == '\x01') return "01";
        else if(charToConvert == '\x02') return "02";
        else if(charToConvert == '\x03') return "03";
        else if(charToConvert == '\x04') return "04";
        else if(charToConvert == '\x05') return "05";
        else if(charToConvert == '\x06') return "06";
        else if(charToConvert == '\x07') return "07";
        else if(charToConvert == '\x08') return "08";
        else if(charToConvert == '\x09') return "09";
        else if(charToConvert == '\x0A') return "0A";
        else if(charToConvert == '\x0B') return "0B";
        else if(charToConvert == '\x0C') return "0C";
        else if(charToConvert == '\x0D') return "0D";
        else if(charToConvert == '\x0E') return "0E";
        else if(charToConvert == '\x0F') return "0F";

        else if(charToConvert == '\x10') return "10";
        else if(charToConvert == '\x11') return "11";
        else if(charToConvert == '\x12') return "12";
        else if(charToConvert == '\x13') return "13";
        else if(charToConvert == '\x14') return "14";
        else if(charToConvert == '\x15') return "15";
        else if(charToConvert == '\x16') return "16";
        else if(charToConvert == '\x17') return "17";
        else if(charToConvert == '\x18') return "18";
        else if(charToConvert == '\x19') return "19";
        else if(charToConvert == '\x1A') return "1A";
        else if(charToConvert == '\x1B') return "1B";
        else if(charToConvert == '\x1C') return "1C";
        else if(charToConvert == '\x1D') return "1D";
        else if(charToConvert == '\x1E') return "1E";
        else if(charToConvert == '\x1F') return "1F";

        else if(charToConvert == '\x20') return "20";
        else if(charToConvert == '\x21') return "21";
        else if(charToConvert == '\x22') return "22";
        else if(charToConvert == '\x23') return "23";
        else if(charToConvert == '\x24') return "24";
        else if(charToConvert == '\x25') return "25";
        else if(charToConvert == '\x26') return "26";
        else if(charToConvert == '\x27') return "27";
        else if(charToConvert == '\x28') return "28";
        else if(charToConvert == '\x29') return "29";
        else if(charToConvert == '\x2A') return "2A";
        else if(charToConvert == '\x2B') return "2B";
        else if(charToConvert == '\x2C') return "2C";
        else if(charToConvert == '\x2D') return "2D";
        else if(charToConvert == '\x2E') return "2E";
        else if(charToConvert == '\x2F') return "2F";

        else if(charToConvert == '\x30') return "30";
        else if(charToConvert == '\x31') return "31";
        else if(charToConvert == '\x32') return "32";
        else if(charToConvert == '\x33') return "33";
        else if(charToConvert == '\x34') return "34";
        else if(charToConvert == '\x35') return "35";
        else if(charToConvert == '\x36') return "36";
        else if(charToConvert == '\x37') return "37";
        else if(charToConvert == '\x38') return "38";
        else if(charToConvert == '\x39') return "39";
        else if(charToConvert == '\x3A') return "3A";
        else if(charToConvert == '\x3B') return "3B";
        else if(charToConvert == '\x3C') return "3C";
        else if(charToConvert == '\x3D') return "3D";
        else if(charToConvert == '\x3E') return "3E";
        else if(charToConvert == '\x3F') return "3F";

        else if(charToConvert == '\x40') return "40";
        else if(charToConvert == '\x41') return "41";
        else if(charToConvert == '\x42') return "42";
        else if(charToConvert == '\x43') return "43";
        else if(charToConvert == '\x44') return "44";
        else if(charToConvert == '\x45') return "45";
        else if(charToConvert == '\x46') return "46";
        else if(charToConvert == '\x47') return "47";
        else if(charToConvert == '\x48') return "48";
        else if(charToConvert == '\x49') return "49";
        else if(charToConvert == '\x4A') return "4A";
        else if(charToConvert == '\x4B') return "4B";
        else if(charToConvert == '\x4C') return "4C";
        else if(charToConvert == '\x4D') return "4D";
        else if(charToConvert == '\x4E') return "4E";
        else if(charToConvert == '\x4F') return "4F";

        else if(charToConvert == '\x50') return "50";
        else if(charToConvert == '\x51') return "51";
        else if(charToConvert == '\x52') return "52";
        else if(charToConvert == '\x53') return "53";
        else if(charToConvert == '\x54') return "54";
        else if(charToConvert == '\x55') return "55";
        else if(charToConvert == '\x56') return "56";
        else if(charToConvert == '\x57') return "57";
        else if(charToConvert == '\x58') return "58";
        else if(charToConvert == '\x59') return "59";
        else if(charToConvert == '\x5A') return "5A";
        else if(charToConvert == '\x5B') return "5B";
        else if(charToConvert == '\x5C') return "5C";
        else if(charToConvert == '\x5D') return "5D";
        else if(charToConvert == '\x5E') return "5E";
        else if(charToConvert == '\x5F') return "5F";

        else if(charToConvert == '\x60') return "60";
        else if(charToConvert == '\x61') return "61";
        else if(charToConvert == '\x62') return "62";
        else if(charToConvert == '\x63') return "63";
        else if(charToConvert == '\x64') return "64";
        else if(charToConvert == '\x65') return "65";
        else if(charToConvert == '\x66') return "66";
        else if(charToConvert == '\x67') return "67";
        else if(charToConvert == '\x68') return "68";
        else if(charToConvert == '\x69') return "69";
        else if(charToConvert == '\x6A') return "6A";
        else if(charToConvert == '\x6B') return "6B";
        else if(charToConvert == '\x6C') return "6C";
        else if(charToConvert == '\x6D') return "6D";
        else if(charToConvert == '\x6E') return "6E";
        else if(charToConvert == '\x6F') return "6F";

        else if(charToConvert == '\x70') return "70";
        else if(charToConvert == '\x71') return "71";
        else if(charToConvert == '\x72') return "72";
        else if(charToConvert == '\x73') return "73";
        else if(charToConvert == '\x74') return "74";
        else if(charToConvert == '\x75') return "75";
        else if(charToConvert == '\x76') return "76";
        else if(charToConvert == '\x77') return "77";
        else if(charToConvert == '\x78') return "78";
        else if(charToConvert == '\x79') return "79";
        else if(charToConvert == '\x7A') return "7A";
        else if(charToConvert == '\x7B') return "7B";
        else if(charToConvert == '\x7C') return "7C";
        else if(charToConvert == '\x7D') return "7D";
        else if(charToConvert == '\x7E') return "7E";
        else if(charToConvert == '\x7F') return "7F";

        else if(charToConvert == '\x80') return "80";
        else if(charToConvert == '\x81') return "81";
        else if(charToConvert == '\x82') return "82";
        else if(charToConvert == '\x83') return "83";
        else if(charToConvert == '\x84') return "84";
        else if(charToConvert == '\x85') return "85";
        else if(charToConvert == '\x86') return "86";
        else if(charToConvert == '\x87') return "87";
        else if(charToConvert == '\x88') return "88";
        else if(charToConvert == '\x89') return "89";
        else if(charToConvert == '\x8A') return "8A";
        else if(charToConvert == '\x8B') return "8B";
        else if(charToConvert == '\x8C') return "8C";
        else if(charToConvert == '\x8D') return "8D";
        else if(charToConvert == '\x8E') return "8E";
        else if(charToConvert == '\x8F') return "8F";

        else if(charToConvert == '\x90') return "90";
        else if(charToConvert == '\x91') return "91";
        else if(charToConvert == '\x92') return "92";
        else if(charToConvert == '\x93') return "93";
        else if(charToConvert == '\x94') return "94";
        else if(charToConvert == '\x95') return "95";
        else if(charToConvert == '\x96') return "96";
        else if(charToConvert == '\x97') return "97";
        else if(charToConvert == '\x98') return "98";
        else if(charToConvert == '\x99') return "99";
        else if(charToConvert == '\x9A') return "9A";
        else if(charToConvert == '\x9B') return "9B";
        else if(charToConvert == '\x9C') return "9C";
        else if(charToConvert == '\x9D') return "9D";
        else if(charToConvert == '\x9E') return "9E";
        else if(charToConvert == '\x9F') return "9F";

        else if(charToConvert == '\xA0') return "A0";
        else if(charToConvert == '\xA1') return "A1";
        else if(charToConvert == '\xA2') return "A2";
        else if(charToConvert == '\xA3') return "A3";
        else if(charToConvert == '\xA4') return "A4";
        else if(charToConvert == '\xA5') return "A5";
        else if(charToConvert == '\xA6') return "A6";
        else if(charToConvert == '\xA7') return "A7";
        else if(charToConvert == '\xA8') return "A8";
        else if(charToConvert == '\xA9') return "A9";
        else if(charToConvert == '\xAA') return "AA";
        else if(charToConvert == '\xAB') return "AB";
        else if(charToConvert == '\xAC') return "AC";
        else if(charToConvert == '\xAD') return "AD";
        else if(charToConvert == '\xAE') return "AE";
        else if(charToConvert == '\xAF') return "AF";

        else if(charToConvert == '\xB0') return "B0";
        else if(charToConvert == '\xB1') return "B1";
        else if(charToConvert == '\xB2') return "B2";
        else if(charToConvert == '\xB3') return "B3";
        else if(charToConvert == '\xB4') return "B4";
        else if(charToConvert == '\xB5') return "B5";
        else if(charToConvert == '\xB6') return "B6";
        else if(charToConvert == '\xB7') return "B7";
        else if(charToConvert == '\xB8') return "B8";
        else if(charToConvert == '\xB9') return "B9";
        else if(charToConvert == '\xBA') return "BA";
        else if(charToConvert == '\xBB') return "BB";
        else if(charToConvert == '\xBC') return "BC";
        else if(charToConvert == '\xBD') return "BD";
        else if(charToConvert == '\xBE') return "BE";
        else if(charToConvert == '\xBF') return "BF";

        else if(charToConvert == '\xC0') return "C0";
        else if(charToConvert == '\xC1') return "C1";
        else if(charToConvert == '\xC2') return "C2";
        else if(charToConvert == '\xC3') return "C3";
        else if(charToConvert == '\xC4') return "C4";
        else if(charToConvert == '\xC5') return "C5";
        else if(charToConvert == '\xC6') return "C6";
        else if(charToConvert == '\xC7') return "C7";
        else if(charToConvert == '\xC8') return "C8";
        else if(charToConvert == '\xC9') return "C9";
        else if(charToConvert == '\xCA') return "CA";
        else if(charToConvert == '\xCB') return "CB";
        else if(charToConvert == '\xCC') return "CC";
        else if(charToConvert == '\xCD') return "CD";
        else if(charToConvert == '\xCE') return "CE";
        else if(charToConvert == '\xCF') return "CF";

        else if(charToConvert == '\xD0') return "D0";
        else if(charToConvert == '\xD1') return "D1";
        else if(charToConvert == '\xD2') return "D2";
        else if(charToConvert == '\xD3') return "D3";
        else if(charToConvert == '\xD4') return "D4";
        else if(charToConvert == '\xD5') return "D5";
        else if(charToConvert == '\xD6') return "D6";
        else if(charToConvert == '\xD7') return "D7";
        else if(charToConvert == '\xD8') return "D8";
        else if(charToConvert == '\xD9') return "D9";
        else if(charToConvert == '\xDA') return "DA";
        else if(charToConvert == '\xDB') return "DB";
        else if(charToConvert == '\xDC') return "DC";
        else if(charToConvert == '\xDD') return "DD";
        else if(charToConvert == '\xDE') return "DE";
        else if(charToConvert == '\xDF') return "DF";

        else if(charToConvert == '\xE0') return "E0";
        else if(charToConvert == '\xE1') return "E1";
        else if(charToConvert == '\xE2') return "E2";
        else if(charToConvert == '\xE3') return "E3";
        else if(charToConvert == '\xE4') return "E4";
        else if(charToConvert == '\xE5') return "E5";
        else if(charToConvert == '\xE6') return "E6";
        else if(charToConvert == '\xE7') return "E7";
        else if(charToConvert == '\xE8') return "E8";
        else if(charToConvert == '\xE9') return "E9";
        else if(charToConvert == '\xEA') return "EA";
        else if(charToConvert == '\xEB') return "EB";
        else if(charToConvert == '\xEC') return "EC";
        else if(charToConvert == '\xED') return "ED";
        else if(charToConvert == '\xEE') return "EE";
        else if(charToConvert == '\xEF') return "EF";

        else if(charToConvert == '\xF0') return "F0";
        else if(charToConvert == '\xF1') return "F1";
        else if(charToConvert == '\xF2') return "F2";
        else if(charToConvert == '\xF3') return "F3";
        else if(charToConvert == '\xF4') return "F4";
        else if(charToConvert == '\xF5') return "F5";
        else if(charToConvert == '\xF6') return "F6";
        else if(charToConvert == '\xF7') return "F7";
        else if(charToConvert == '\xF8') return "F8";
        else if(charToConvert == '\xF9') return "F9";
        else if(charToConvert == '\xFA') return "FA";
        else if(charToConvert == '\xFB') return "FB";
        else if(charToConvert == '\xFC') return "FC";
        else if(charToConvert == '\xFD') return "FD";
        else if(charToConvert == '\xFE') return "FE";
        else if(charToConvert == '\xFF') return "FF";
        else return "NN";
    }

    // Returns the vertical angle of a laser relative to the VLP16's horizon in radians from a laser ID/channel ID
    double getLaserAngleFromLaserID(int channelID) {
        if(channelID < 1 || channelID > 32) {
            std::cout << "1\n";
            return 0;
        }
        int laserID = channelID - 1;
        if(laserID > 15) laserID -= 16;
        if(laserID < 0 || laserID > 15) {
            std::cout << "2\n";
            std::cout << "laserID: " << laserID << "\n";
            return 0; // none of the lasers are angled at 0 degrees, meaning this can be used to detect an incorrect input
        }
        else if(laserID == 0) return -0.261799;
        else if(laserID == 1) return 0.0174533;
        else if(laserID == 2) return -0.226893;
        else if(laserID == 3) return 0.0523599;
        else if(laserID == 4) return -0.191986;
        else if(laserID == 5) return 0.0872665;
        else if(laserID == 6) return -0.15708;
        else if(laserID == 7) return 0.122173;
        else if(laserID == 8) return -0.122173;
        else if(laserID == 9) return 0.15708;
        else if(laserID == 10) return -0.0872665;
        else if(laserID == 11) return 0.191986;
        else if(laserID == 12) return -0.0523599;
        else if(laserID == 13) return 0.226893;
        else if(laserID == 14) return -0.0174533;
        else if(laserID == 15) return 0.261799;
        else return 0; // none of the lasers are angled at 0 degrees, meaning this can be used to detect an incorrect input
    }

    // currently for single return mode of VLP-16
    void velodynePCAPReader::readFile() {

        objPointCloudProcessor objProcessor;
        LaserProcessingClass laserProcessing;
        auto t1 = std::chrono::high_resolution_clock::now();
        std::ifstream pcapStream(absolutePath, std::fstream::binary | std::fstream::in);
        std::ios::streampos fsize = 0;
        fsize = pcapStream.tellg();
        pcapStream.seekg(0, std::ios::end); // go to end of file
        fsize = pcapStream.tellg() - fsize; // calculate size of file in bytes
        pcapStream.seekg(0, std::ios::beg); // go back to start of file

        char* buffer = new char[fsize];
 
        int curPos = 0;
        pcapStream.read(buffer, fsize);

        int bytesFromStart = 0; // number of bytes looped over since start of packet (starting at 0xFF)
        bool ffFlag = false; // set to true when 0xFF byte is located so that we can test if the next byte is 0xEE (as per the VLP-16's packet structure)
        bool azimuthFlag = false; // set to true when the two-byte 0xFFEE flag is located as the following 2 bytes comprise the azimuth
        unsigned int firstPacketInBlockTimestamp = 0;
        velodyneVLP16FrameDataBlocks curFrameBlocks; // stores all the data blocks of a frame - each frame should be approixmately 100,000 bytes (well, 100,000 bytes including blocks of nullbytes between packets in raw data)
        
        // std::ofstream distanceBytesFile("distanceBytes.txt");
        
        for(int i = 0; i < fsize; i++) {
            if(i < fsize) {
                // if(i > 400000) break; // this is only here so that the loop doesn't keep going through the whole file for testing 
                if(buffer[i] == '\xFF') {
                    ffFlag = true;
                }
                else if(buffer[i] == '\xEE') {
                    if(ffFlag) { // if ffFlag is true then we must be at the start of a datablock (0xFFEE)
                        ffFlag = false;
                        if(i-2 > -1 && i-3 > -1 && buffer[i-2] == '\x00' && buffer[i-3] == '\x00') {
                            velodyneVLP16Packet curPacket;
                            // distanceBytesFile << "\n\nPacket " << packets.size()+1 << "\n\n";
                        
                            // velodyneVLP16DataBlock db;

                            // at the moment i is the index of the 0xEE byte
                            // so the next two bytes will be the azimuth bytes
                            // then after that we have 12 channels of 3 bytes each
                            // doing i += 3 once gets us to the first byte in the first channel
                            // then i += 3 gets us to the next channels (by doing it multiple times)
                            // keep count of how many times we've done this so we know when to stop
                            // unsigned char azimuthByte1 = buffer[i+1];
                            // unsigned char azimuthByte2 = buffer[i+2];
                            // db.azimuth = ((float)(azimuthByte2 << 8 | azimuthByte1))/100; // combining azimuth bytes in reverse order as int to get azimuth*100 as an integer, then devide y 100 to get true azimuth as an angle from 0 to 359.99deg
                            for(int datablock = 0; datablock < 12; datablock++) { // this is currently broken as we need to take the 0xFFEE and azimuth bytes in each datablock into account
                                // distanceBytesFile << "\n";
                                // need to get azimuth bytes and account for 0xFFEE bytes here as this code only runs when we go to a new datablock
                                if(datablock > 0) i += 4; // add 4 to i to get to the 0xEE byte (adding 3 to get from first dist byte of last channel to 0xFF byte of 0xFFEE bytes, then add another 1 to get to the 0xEE byte)
                                
                                velodyneVLP16DataBlock db;
                                unsigned char aziByte1 = buffer[i+1];
                                unsigned char aziByte2 = buffer[i+2];
                                db.azimuth = ((float)(aziByte2 << 8 | aziByte1))/100; // combining azimuth bytes in reverse order as int to get azimuth*100 as an integer, then devide y 100 to get true azimuth as an angle from 0 to 359.99deg
                                for(int channel = 0; channel < 32; channel++) {
                                    i += 3; // now i is the first byte of the (channel+1) channel (e.g if channel is 0, i is the first byte of channel 1)
                                    // std::cout << "point: " << pointCounter+(datablock*channel) << "\n";
                                    velodyneVLP16Point point;
                                
                                    unsigned char distByte1 = buffer[i];
                                    unsigned char distByte2 = buffer[i+1];
                                    unsigned char reflectivityByte = buffer[i+2];
                                
                                    point.channel = channel+1;
                                    point.distance = (float) (((float)(distByte2 << 8 | distByte1))/500); // distance is in mm, so divide by 500 to get distance in m
                                    point.reflectivity = (float) reflectivityByte; // TODO: not sure if reflectivity is a float or int
                                    db.points.push_back(point);
                                    std::string aziByteStr = charToHex(aziByte1);
                                    std::string aziByteStr2 = charToHex(aziByte2);
                                    std::string distByteStr = charToHex(distByte1);
                                    std::string distByteStr2 = charToHex(distByte2);

                                    // distanceBytesFile << "a " << aziByteStr << " " << aziByteStr2 << " d " << distByteStr << " " << distByteStr2 << "\n";
                                
                                    // std::cout << "i: " << i << "\n";
                                    // std::cout << "dist byte: 0x" << distByteStr2 << distByteStr << "\n";
                                    // std::cout << "dist: " << point.distance << "\n";
                                    // std::cout << "refl: " << point.reflectivity << "\n";

                                    // next thing I need to do is convert a series of packets into a frame (however many packets it takes to cover 100ms since the VLP16's motor spins at 10Hz)
                                    // to do this I need to parse the timestamp from the packet as well and when the difference between the current timestamp and the last starting packet's timestamp exceeds 100ms then we know a frame has been collected
                                }

                                curPacket.dataBlocks.push_back(db);
                            }
                            // Get timestamp (an unsigned in composed of the 4 bytes after the final reflectivity byte in the final data block)
                            // - Timestamp is the time of the first laser firing in the first data block of the packet (as per VLP-16 User Manual: https://velodynelidar.com/wp-content/uploads/2019/12/63-9243-Rev-E-VLP-16-User-Manual.pdf)
                            // Current i is @ the final reflectivity byte of the final data block, so we need the i+1, i+2, i+3, and i+4 bytes
                            i += 4;
                            unsigned char timeByte1 = buffer[i-1];
                            unsigned char timeByte2 = buffer[i];
                            unsigned char timeByte3 = buffer[i+1];
                            unsigned char timeByte4 = buffer[i+2];
                            std::string timeByteStr1 = charToHex(timeByte1);
                            std::string timeByteStr2 = charToHex(timeByte2);
                            std::string timeByteStr3 = charToHex(timeByte3);
                            std::string timeByteStr4 = charToHex(timeByte4);

                            // std::cout << "------------------------------------------------\n";
                            // std::cout << "time bytes: 0x" << timeByteStr1 << " " << timeByteStr2 << " " << timeByteStr3 << " " << timeByteStr4 << "\n";
                            // std::cout << "time byte: 0x" << timeByteStr4 << timeByteStr3 << timeByteStr2 << timeByteStr1 << "\n";

                            unsigned int timestamp = timeByte4 << 24 | (timeByte3 << 16) | (timeByte2 << 8) | timeByte1;

                            // Go through each data block in curPacket, calculate its timestamp, check if 100ms as passed
                            int blockCounter = 0;
                            for(velodyneVLP16DataBlock block : curPacket.dataBlocks) {
                                unsigned int blockTimestamp = timestamp + blockCounter*55.296; // 55.296us per firing sequence
                                if(blockTimestamp - firstPacketInBlockTimestamp > 99999) { // if block timestamp is at least 100ms (100,000us) after the timestamp of the first packet int he frame then we need to start a new frame
                                    firstPacketInBlockTimestamp = blockTimestamp; // reset firstBlockInFrameTimestamp
                                    frameDataBlocks.push_back(curFrameBlocks); // add current frame struct to frames vector
                                    velodyneVLP16FrameDataBlocks newFrameBlocks; // create next frame struct
                                    curFrameBlocks = newFrameBlocks; // assign next frame struct to current frame struct

                                    curFrameBlocks.dataBlocks.push_back(block); // add block to new frame struct's data blocks vector
                                } else {
                                    // curPacket.timestamp = timestamp;
                                    curFrameBlocks.dataBlocks.push_back(block);
                                }
                                blockCounter++;
                            }
                            packets.push_back(curPacket);
                        }
                    } 
                } 
            }
            // packets.push_back(curPacket);
        }
        // distanceBytesFile.close();

        int fBlockCount = 1;
        int tooSmallCount = 0;
        int pointsConverted = 0;
        // std::ofstream pointsFile("points.txt"); // this was used for debugging - remove later if not needed
        for(velodyneVLP16FrameDataBlocks fBlock : frameDataBlocks) {
            // std::cout << "frameDataBlocks," << fBlockCount << ": " << fBlock.dataBlocks.size() << " data blocks\n";
            if(fBlock.dataBlocks.size() < 750) tooSmallCount++;
            // pointsFile << "frameDataBlocks," << fBlockCount << ": " << fBlock.dataBlocks.size() << " data blocks\n\n";
            fBlockCount++;
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr curFrameCloud(new pcl::PointCloud<pcl::PointXYZI>);

            for(velodyneVLP16DataBlock fBlockDB : fBlock.dataBlocks) {
                for(velodyneVLP16Point p : fBlockDB.points) {
                    double laserAngle = getLaserAngleFromLaserID(p.channel);
                    // std::cout << "channel: " << p.channel << "\n";
                    // std::cout << "laserAngle: " << laserAngle << "deg \n";
                    if(laserAngle != 0) {
                        // std::cout << "pointsConverted: " << pointsConverted << "\n";
                        // std::cout << "laserAngle: " << laserAngle << "rad \n";
                        // std::cout << "cos(laserAngle): " << laserAngle << "\n";
                        // std::cout << "azimuth: " << fBlockDB.azimuth << "deg \n";
                        // std::cout << "azimuth: " << fBlockDB.azimuth*(M_PI/180) << "rad \n";
                        // std::cout << "sin(azimuth): " << std::sin(fBlockDB.azimuth*(M_PI/180)) << "\n";
                        // std::cout << "cos(azimuth): " << std::cos(fBlockDB.azimuth*(M_PI/180)) << "\n";
                        double x = p.distance*std::cos(laserAngle)*std::sin(fBlockDB.azimuth*(M_PI/180));
                        double y = p.distance*std::cos(laserAngle)*std::cos(fBlockDB.azimuth*(M_PI/180));
                        double z = p.distance*std::sin(laserAngle);
                        // pointsFile << x << "\t\t" << y << "\t\t" << z << "\t\t\t\t" << p.distance << "\t\t" << (std::atan(z/p.distance)*(180/M_PI)) << "\n";
                        // std::cout << "r: " << p.distance << "\n";
                        pcl::PointXYZI cartesianPoint;
                        cartesianPoint.x = x;
                        // std::cout << "x: " << cartesianPoint.x << "\n";
                        cartesianPoint.y = y;
                        // std::cout << "y: " << cartesianPoint.y << "\n";
                        cartesianPoint.z = z;
                        // std::cout << "z: " << cartesianPoint.z << "\n";
                        cartesianPoint.intensity = p.reflectivity;
                        // std::cout << "ref: " << cartesianPoint.intensity << "\n";
                        
                        pointCloud->push_back(cartesianPoint);
                        curFrameCloud->push_back(cartesianPoint);
                        // std::cout << "push\n";
                        
                        pointsConverted++;
                    }
                }
                // pointsFile << "\n";
            }

            frameClouds.push_back(curFrameCloud);
        }
        // pointsFile.close(); // was used for debugging - remove later if not needed
        std::cout << "frameClouds count: " << frameClouds.size() << "\n";

        std::cout << "tooSmallCount: " << tooSmallCount << "\n";

        auto t2 = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::chrono::duration<double, std::milli> ms_double = t2 - t1;
        std::cout << "duration: " << dur.count() << "ms\n";
        std::cout << "duration (d): "  << ms_double.count() << "ms\n";
        std::cout << "packet count: " <<  packets.size() << "\n"; 
        std::cout << "frameDataBlocks: " << frameDataBlocks.size() << "\n";
        std::cout << "points converted: " << pointsConverted << "\n";

        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL Visualiser"));
        viewer->setBackgroundColor(0,0,0);
        std::cout << "Frame 1 Points: " << frameClouds[3]->points.size() << "\n";
        viewer->addPointCloud<pcl::PointXYZI>(frameClouds[3], "Frame 1");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Frame 1");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        viewer->setCameraPosition(0,16,0,0,0,1);

        int frameCounter = 1;
        auto lastTime = std::chrono::high_resolution_clock::now();
        while(!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::cout << "frameNumber: " << frameCounter << "\n";
            if(frameCounter < frameClouds.size() && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastTime).count() > 100) {
                if(frameClouds[frameCounter]->points.size() > 24000) {
                    viewer->removeAllPointClouds();
                    viewer->removeAllShapes();
                    // viewer->updatePointCloud(frameClouds[frameCounter], "Frame 1");
                    std::string frameName = "Frame " + std::to_string(frameCounter);
                    
                    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudEdge(new pcl::PointCloud<pcl::PointXYZI>()); // JUST FOR DEBUGGING - REMOVE LATER
                    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudSurf(new pcl::PointCloud<pcl::PointXYZI>()); // JUST FOR DEBUGGING - REMOVE LATER
                    pcl::PointCloud<pcl::PointXYZI>::Ptr pcFilter(new pcl::PointCloud<pcl::PointXYZI>()); // JUST FOR DEBUGGING - REMOVE LATER

                    
                    laserProcessing.featureExtraction(frameClouds[frameCounter], pointCloudEdge, pointCloudSurf); // JUST FOR DEBUGGING - REMOVE LATER
                    
                    Eigen::Vector4f minVec = Eigen::Vector4f(-10, -6.2, -2, 1);
                    Eigen::Vector4f maxVec = Eigen::Vector4f(15, 7, 10, 1);

                    pcFilter = objProcessor.filterCloud(frameClouds[frameCounter], 0.05, minVec, maxVec); // JUST FOR DEBUGGING - REMOVE LATER

                    std::unordered_set<int> inliers = ransacPlane(pcFilter, 100, 0.2); // JUST FOR DEBUGGING - REMOVE LATER

                    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudInliers(new pcl::PointCloud<pcl::PointXYZI>()); // JUST FOR DEBUGGING - REMOVE LATER
                    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudOutliers(new pcl::PointCloud<pcl::PointXYZI>()); // JUST FOR DEBUGGING - REMOVE LATER

                    for(int index = 0; index < pcFilter->points.size(); index++) { // JUST FOR DEBUGGING - REMOVE LATER
                        pcl::PointXYZI point = pcFilter->points[index];
                        if(inliers.count(index)) {
                            pointCloudInliers->points.push_back(point);
                        } else {
                            pointCloudOutliers->points.push_back(point);
                        }
                    }

                    renderPointCloud(viewer, pointCloudInliers, "Inliers", Colour(0,1,0));
                    renderPointCloud(viewer, pointCloudOutliers, "Outliers", Colour(1,0,0.5));

                    KdTree* tree = new KdTree;
                    std::vector<std::vector<float>> pointVectors;

                    for(int i = 0; i < pointCloudOutliers->points.size(); i++) {
                        std::vector<float> pointVector;
                        pointVector.push_back(pointCloudOutliers->points[i].x);
                        pointVector.push_back(pointCloudOutliers->points[i].y);
                        pointVector.push_back(pointCloudOutliers->points[i].z);
                        pointVectors.push_back(pointVector);
                        tree->insert(pointVector, i);
                    }

                    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = euclideanCluster(pointVectors, tree, 0.25, 10);

                    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedPlanes = objProcessor.segmentPlane(frameClouds[frameCounter], 100, 0.3);

                    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> surfColourHandler(pointCloudSurf, 0, 255, 0);
                    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> edgeColourHandler(pointCloudEdge, 255, 0, 0);


                    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> segment1ColourHandler(segmentedPlanes.first, 0, 255, 0);
                    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> segment2ColourHandler(segmentedPlanes.second, 255, 0, 0);


                    viewer->updatePointCloud<pcl::PointXYZI>(frameClouds[frameCounter], frameName);
                    // viewer->removeAllPointClouds();
                    // viewer->addPointCloud<pcl::PointXYZI>(frameClouds[frameCounter], frameName);
                    // viewer->addPointCloud<pcl::PointXYZI>(pointCloudSurf, surfColourHandler, "Surf " + std::to_string(frameCounter));
                    // viewer->addPointCloud<pcl::PointXYZI>(pointCloudEdge, edgeColourHandler, "Edge " + std::to_string(frameCounter));

                    // For testing segmentation for obj detection
                    // viewer->addPointCloud<pcl::PointXYZI>(segmentedPlanes.first, segment1ColourHandler, "Segment 1 " + std::to_string(frameCounter));
                    // viewer->addPointCloud<pcl::PointXYZI>(segmentedPlanes.second, segment2ColourHandler, "Segment 2 " + std::to_string(frameCounter));
                    
                    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = objProcessor.clusterCloud(segmentedPlanes.first, 0.5, 3, 50);
                    int clusterID = 1;
                    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters) {
                        // std::cout << "cluster size: " << cluster->points.size() << "\n";
                        renderPointCloud(viewer, cluster, "Cluster " + std::to_string(clusterID), Colour(0,0,1));

                        Box box = objProcessor.boundingBox(cluster);
                        renderBox(viewer, box, clusterID);
                        clusterID++;
                    }

                    frameCounter++;
                } else frameCounter++;
               
            }
        }

        delete[] buffer;

    }

    

    void velodynePCAPReader::readNextPacket() {

    }

    void velodynePCAPReader::readBytes(int byteCount) {

    }
