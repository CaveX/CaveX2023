#include "velodynePCAPReader.h"
#include <chrono>

    velodynePCAPReader::velodynePCAPReader(std::string absolutePath) {
        this->absolutePath = absolutePath;
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

        int ffeeCount = 0;
        int bytesFromStart = 0; // number of bytes looped over since start of packet (starting at 0xFF)
        bool ffFlag = false; // set to true when 0xFF byte is located so that we can test if the next byte is 0xEE (as per the VLP-16's packet structure)
        bool azimuthFlag = false; // set to true when the two-byte 0xFFEE flag is located as the following 2 bytes comprise the azimuth
        std::vector<velodyneVLP16Packet> packets;
        velodyneVLP16Packet curPacket;

        int increment = 1;
        for(int i = 0; i < fsize; i += increment) {
            if(i < fsize) {
                // if(i > 85) break; // this is only here so that the loop doesn't keep going through the whole file for testing 
                if(buffer[i] == '\xFF') {
                    ffFlag = true;
                }
                else if(buffer[i] == '\xEE') {
                    if(ffFlag) { // if ffFlag is true then we must be at the start of a datablock (0xFFEE)
                        
                        velodyneVLP16DataBlock db;
                        
                        // the condition below is not met if the 0xFFEE flag is not at the start of a new packet. If it is not met, we ignore the data because losing 16 points of data will not make a difference when 290k points are retrieved per second
                        if(i-2 > -1 && i-3 > -1 && buffer[i-2] == '\x00' && buffer[i-3] == '\x00') { // this condition checks if the two bytes before the 0xFFEE flag are null bytes (0x0000). This should (i think) indicate the start of a packet (well, the first data block in a packet)
                            ffFlag = false; // reset ffFlag
                            ffeeCount++;
                            unsigned char azimuthByte1 = buffer[i+1]; // unsafe since we're assuming another byte exists - also casting to unsigned char as this is required to get the right output
                            unsigned char azimuthByte2 = buffer[i+2]; // unsafe since we're assuming another 2 bytes exist - also casting to unsigned char as this is required to get ther right output

                            int azi = azimuthByte2 << 8 | azimuthByte1;
                            db.azimuth = ((float)((azimuthByte2 << 8) | azimuthByte1))/100; // combining azimuth bytes in reverse order as int to get azimuth*100 as an integer, then divide by 100 to get true azimuth as an angle from 0 to 359.99deg
                            // std::cout << "azb1: 0x" << std::hex << azimuthByte1 << "\n";
                            // std::cout << "azb2: 0x" << std::hex << azimuthByte2 << "\n";
                            // std::cout << "azi: " <<  azi << "\n";
                            // std::cout <<  "azimuth: " <<  db.azimuth << "\n";

                            // loop through all data blocks to determine the distance and reflectivity 
                            for(int db = 0; db < 11; db++) {
                                for(int c = 0; c < 15; c++) { // c = channel
                                    velodyneVLP16Point p;
                                    unsigned char distByte1 = buffer[i+2+(((db+1)*c+1))]; // i = 0xEE starting point; add 2 to get to second azimuth byte; add c+1 to get to first distance point of block c
                                    unsigned char distByte2 = buffer[i+2+(((db+1)*c+2))]; // i = 0xEE starting point; add 2 to get to second azimuth byte; add c+1 to get to first distance point of block c
                                    unsigned char reflectByte = buffer[i+2+(((db+1)*c+3))]; // i = 0xEE starting point; add 2 to get to second azimuth byte; add c+1 to get to first distance point of block c
                                    
                                    p.distance = ((float)((distByte2 << 8) | distByte1))*2/100; // convert the two distance bytes to distance in metres as per VLP-16 manual
                                    p.reflectivity = (float)reflectByte;
                                    // std::cout << "dist: " << p.distance << "\n";
                                    // std::cout << "refl: " << p.reflectivity << "\n";
                                }
                            }
                        }
                    } 
                } 
            }
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::chrono::duration<double, std::milli> ms_double = t2 - t1;
        // std::cout << "duration: " << dur.count() << "ms\n";
        std::cout << "duration (d): "  << ms_double.count() << "ms\n";
        std::cout << "ffeeCount: " << ffeeCount << "\n";


        delete[] buffer;

    }

    void velodynePCAPReader::readNextPacket() {

    }

    void velodynePCAPReader::readBytes(int byteCount) {

    }

