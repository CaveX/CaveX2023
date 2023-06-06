#include "velodynePCAPReader.h"
#include <chrono>

    velodynePCAPReader::velodynePCAPReader(std::string absolutePath) {
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

    // currently for single return mode of VLP-16
    void velodynePCAPReader::readFile() {
        auto t1 = std::chrono::high_resolution_clock::now();
        // pcapBuffer.push_back()
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
        // std::vector<velodyneVLP16Packet> packets;

        for(int i = 0; i < fsize; i++) {
            if(i < fsize) {
                // if(i > 2000) break; // this is only here so that the loop doesn't keep going through the whole file for testing 
                if(buffer[i] == '\xFF') {
                    ffFlag = true;
                }
                else if(buffer[i] == '\xEE') {
                    if(ffFlag) { // if ffFlag is true then we must be at the start of a datablock (0xFFEE)
                        ffFlag = false;
                        if(i-2 > -1 && i-3 > -1 && buffer[i-2] == '\x00' && buffer[i-3] == '\x00') {
                            velodyneVLP16Packet curPacket;
                            int pointCounter = 0;
                        
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
                                // need to get azimuth bytes and account for 0xFFEE bytes here as this code only runs when we go to a new datablock
                                i += 4; // add 4 to i to get to the 0xEE byte (adding 3 to get from first dist byte of last channel to 0xFF byte of 0xFFEE bytes, then add another 1 to get to the 0xEE byte)
                                
                                velodyneVLP16DataBlock db;
                                unsigned char aziByte1 = buffer[i];
                                unsigned char aziByte2 = buffer[i+1];
                                db.azimuth = ((float)(aziByte2 << 8 | aziByte1))/100; // combining azimuth bytes in reverse order as int to get azimuth*100 as an integer, then devide y 100 to get true azimuth as an angle from 0 to 359.99deg
                                for(int channel = 0; channel < 32; channel++) {
                                    i += 3; // now i is the first byte of the (channel+1) channel (e.g if channel is 0, i is the first byte of channel 1)
                                    
                                    velodyneVLP16Point point;
                                
                                    unsigned char distByte1 = buffer[i];
                                    unsigned char distByte2 = buffer[i+1];
                                    unsigned char reflectivityByte = buffer[i+2];
                                
                                    point.channel = channel+1;
                                    point.distance = (float) ((int)(distByte2 << 8 | distByte1))/200; // distance is in mm, so divide by 500 to get distance in m
                                    point.reflectivity = (float) reflectivityByte; // TODO: not sure if reflectivity is a float or int

                                    // std::string distByteStr = charToHex(distByte1);
                                    // std::string distByteStr2 = charToHex(distByte2);
                                
                                    // std::cout << "i: " << i << "\n";
                                    // std::cout << "dist byte: 0x" << distByteStr2 << distByteStr << "\n";
                                    // std::cout << "dist: " << point.distance << "\n";
                                    // std::cout << "refl: " << point.reflectivity << "\n";
                                }
                            }
                        }
                        
                        // the condition below is not met if the 0xFFEE flag is not at the start of a new packet. If it is not met, we ignore the data because losing 16 points of data will not make a difference when 290k points are retrieved per second
//                         if(i-2 > -1 && i-3 > -1 && buffer[i-2] == '\x00' && buffer[i-3] == '\x00') { // this condition checks that the i-2 and i-3 indices are non-negative and if the two bytes before the 0xFFEE flag are null bytes (0x0000). This should (i think) indicate the start of a packet (well, the first data block in a packet)
//                             velodyneVLP16DataBlock db;
//                             velodyneVLP16Packet curPacket;
                            
//                             // ffFlag = false; // reset ffFlag
//                             ffeeCount++;
//                             unsigned char azimuthByte1 = buffer[i+1]; // unsafe since we're assuming another byte exists - also casting to unsigned char as this is required to get the right output
//                             unsigned char azimuthByte2 = buffer[i+2]; // unsafe since we're assuming another 2 bytes exist - also casting to unsigned char as this is required to get ther right output
                            
//                             int azi = azimuthByte2 << 8 | azimuthByte1;
//                             db.azimuth = ((float)((azimuthByte2 << 8) | azimuthByte1))/100; // combining azimuth bytes in reverse order as int to get azimuth*100 as an integer, then divide by 100 to get true azimuth as an angle from 0 to 359.99deg
//                             // std::cout << "azb1: 0x" << std::hex << azimuthByte1 << "\n";
//                             // std::cout << "azb2: 0x" << std::hex << azimuthByte2 << "\n";
//                             // std::cout << "azi: " <<  azi << "\n";
//                             std::cout <<  "azimuth: " <<  db.azimuth << "\n";

//                             // loop through all data blocks to determine the distance and reflectivity 
//                             for(int dbIndex = 0; dbIndex < 11; dbIndex++) {
//                                 std::cout << "dbIndex++: " << dbIndex << "\n";
//                                 for(int c = 0; c < 15; c++) { // c = channel
//                                     std::cout << "c++: " << c << "\n";
//                                     velodyneVLP16Point p;
//                                     unsigned char distByte1 = buffer[i+2+(((dbIndex+1)*c+1))]; // i = 0xEE starting point; (dbIndex+1) to go to block dbIndex+1; add 2 to get to second azimuth byte; add c+1 to get to first distance point of block c
//                                     unsigned char distByte2 = buffer[i+2+(((dbIndex+1)*c+2))]; // i = 0xEE starting point; (dbIndex+1) to go to block dbIndex+1; add 2 to get to second azimuth byte; add c+2 to get to second distance point of block c
//                                     // std::cout << "index1: " << (i+2+(((dbIndex+1)*c+1))) << "\n";
//                                     // std::cout << "index2: " << (i+2+(((dbIndex+1)*c+2))) << "\n";
//                                     if(c == 0) { // if true then we need to account for the azimuth bytes at the start of the packet
//                                         distByte1 = buffer[i+2+(((dbIndex+1)*c+1))];
//                                         distByte2 = buffer[i+2+(((dbIndex+1)*c+2))];
//                                         std::cout << "index1: " << (i+2+(((dbIndex+1)*c+1))) << "\n";
//                                         std::cout << "index2: " << (i+2+(((dbIndex+1)*c+2))) << "\n";
//                                         i += 2;
//                                     } else {
//                                         distByte1 = buffer[i+(((dbIndex+1)*c)+1)];
//                                         distByte2 = buffer[i+(((dbIndex+1)*c)+2)];
//                                         std::cout << "index1e: " << (i+((((dbIndex+1)*c)+1))) << "\n";
//                                         std::cout << "index2e: " << (i+((((dbIndex+1)*c)+2))) << "\n";
//                                         i += 3;
//                                     }

//                                     // i = 86: 89/90 -> 90/91 -> 91/92

//                                     // unsigned char distByteTest1 = buffer[i];
//                                     // unsigned char distByteTest2;

//                                     // if(dbIndex == 0 && c == 0) {
//                                         // distByteTest1 = buffer[i+2+(((dbIndex+1)*c+1))];
//                                         // distByteTest2 = buffer[i+2+(((dbIndex+1)*c))]
//                                     // } else {
// // 
//                                     // }

//                                     unsigned char reflectByte = buffer[i+2+(((dbIndex+1)*c+3))]; // i = 0xEE starting point; (dbIndex+1) to go to block dbIndex+1; add 2 to get to second azimuth byte; add c+3 to get to reflectivity point of block c
//                                     std::string distByteStr = charToHex(distByte1);
//                                     std::string distByteStr2 = charToHex(distByte2);
//                                     p.distance = ((float)((distByte2 << 8) | distByte1))*2/100; // convert the two distance bytes to distance in metres as per VLP-16 manual
//                                     p.reflectivity = (float)reflectByte;
//                                     db.points.push_back(p);
//                                     curPacket.dataBlocks.push_back(db);
//                                     if(dbIndex == 11 && c == 15) {
//                                         unsigned char timestampByte1 = buffer[i+2+(((dbIndex+1)*c+1))+1];
//                                         packets.push_back(curPacket);
//                                     }
//                                     // i += 3; // without this i only goes up by 1 and results in distBytes and reflectBytes getting mixed up - this likely causes a problem since the +3 will then be incremented by 1 by the loop
//                                     std::cout << "i: " << i << "\n";
//                                     int d = distByte2 << 8 | distByte1;
//                                     std::cout << "dist byte: 0x" << distByteStr2 << distByteStr << "\n";
//                                     std::cout << "dist bval: " << d << "\n";
//                                     std::cout << "dist: " << p.distance << "\n";
//                                     std::cout << "refl: " << p.reflectivity << "\n";
//                                 }
//                             }
                            // std::cout << "curPacket datablocks size: " << curPacket.dataBlocks.size() << "\n";
                        // }
                        
                        
                    } 
                } 
            }
            // packets.push_back(curPacket);
        }

        

        auto t2 = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::chrono::duration<double, std::milli> ms_double = t2 - t1;
        // std::cout << "duration: " << dur.count() << "ms\n";
        std::cout << "duration (d): "  << ms_double.count() << "ms\n";
        std::cout << "packet count: " <<  packets.size() << "\n"; 

        delete[] buffer;

    }

    

    void velodynePCAPReader::readNextPacket() {

    }

    void velodynePCAPReader::readBytes(int byteCount) {

    }
