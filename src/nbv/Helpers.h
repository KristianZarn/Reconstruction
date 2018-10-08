#ifndef SANDBOX_NBV_HELPERS_H
#define SANDBOX_NBV_HELPERS_H

#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <vector>


void writeBufferToFile(const std::string& filename, int width, int height, const std::vector<unsigned int>& data) {
    assert((width * height) == data.size());

    // Open file
    std::ofstream outf(filename);
    if (!outf) {
        std::cerr << "Could not open file: " << filename << std::endl;
        exit(1);
    }

    // Write data to file
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            outf << data[i*width + j] << " ";
        }
        outf << "\n";
    }
}

template <typename T>
void writeMeasureToFile(const std::string& filename, const std::vector<T>& data) {
    // Open file
    std::ofstream outf(filename);
    if (!outf) {
        std::cerr << "Could not open file: " << filename << std::endl;
        exit(1);
    }

    // Write data to file
    for (double face_value : data) {
        outf << face_value << "\n";
    }
}

#endif //SANDBOX_NBV_HELPERS_H
