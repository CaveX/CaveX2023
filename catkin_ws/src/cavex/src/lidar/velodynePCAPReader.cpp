#include "velodynePCAPReader.h"

    velodynePCAPReader::velodynePCAPReader(std::string absolutePath) {
        this->absolutePath = absolutePath;
    }

    // currently for single return mode of VLP-16
    void velodynePCAPReader::readFile() {
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

        for(int i = 0; i < fsize; i++) {
            if(i > 85) break; // this is only here so that the loop doesn't keep going through the whole file for testing 
            if(buffer[i] == '\xFF') {
                ffFlag = true;
            }
            else if(buffer[i] == '\xEE') {
                if(ffFlag) {
                    if(buffer[i-2] != NULL && buffer[i-3] != NULL && buffer[i-2] == '\x00' && buffer[i-3] == '\x00') { // this condition checks if the two bytes before the 0xFFEE flag are null bytes (0x0000). This should (i think) indicate the start of a packet (well, the first data block in a packet)

                    } else if(buffer[i-2] != NULL && buffer[i-3] != NULL && buffer[i-2] != '\x00' && buffer[i-3] != '\x00') { // this condition checks if the two bytes before the 0xFFEE flag are not null bytes (0x0000). This should (I think) indicate the start of a data block which isn't the first one in a packet

                    } else { // this branch runs when the two previous conditions are not met (so when )

                    }
                    ffFlag = false; // reset ffFlag
                    ffeeCount++;
                    velodyneVLP16DataBlock db;
                    unsigned char azimuthByte1 = buffer[i+1]; // unsafe since we're assuming another byte exists - also casting to unsigned char as this is required to get the right output
                    unsigned char azimuthByte2 = buffer[i+2]; // unsafe since we're assuming another 2 bytes exist - also casting to unsigned char as this is required to get ther right output

                    int azi = azimuthByte2 << 8 | azimuthByte1;
                    db.azimuth = ((float)((azimuthByte2 << 8) | azimuthByte1))/100; // combining azimuth bytes in reverse order as int to get azimuth*100 as an integer, then divide by 100 to get true azimuth as an angle from 0 to 359.99deg
                    std::cout << "azb1: 0x" << std::hex << azimuthByte1 << "\n";
                    std::cout << "azb2: 0x" << std::hex << azimuthByte2 << "\n";
                    std::cout << "azi: " <<  azi << "\n";
                    std::cout <<  "azimuth: " <<  db.azimuth << "\n";

                    // loop through 
                    for(int j = 1; j < 3; j++) {

                    }

                } 
            } 
        }

        std::cout << "ffeeCount: " << ffeeCount << "\n";


        delete[] buffer;

    }

    void velodynePCAPReader::readNextPacket() {

    }

    void velodynePCAPReader::readBytes(int byteCount) {

    }

