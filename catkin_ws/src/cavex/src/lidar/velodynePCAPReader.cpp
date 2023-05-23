#include "velodynePCAPReader.h"

    velodynePCAPReader::velodynePCAPReader(std::string absolutePath) {
        this->absolutePath = absolutePath;
    }

    void velodynePCAPReader::readFile() {
        // pcapBuffer.push_back()
        std::ifstream pcapStream(absolutePath);
        std::string content((std::istreambuf_iterator<char>(pcapStream)), (std::istreambuf_iterator<char>()));
        std::cout << content << "\n";
    }

    void velodynePCAPReader::readNextPacket() {

    }

    void velodynePCAPReader::readBytes(int byteCount) {

    }

