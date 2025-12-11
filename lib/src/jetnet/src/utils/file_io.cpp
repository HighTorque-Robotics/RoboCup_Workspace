#include "file_io.h"
#include <fstream>
#include <regex>
#include <cstdlib>

bool jetnet::write_binary_file(const void* data, size_t len, std::string filename)
{
    std::ofstream outfile(filename, std::ofstream::binary);

    if (!outfile)
        return false;

    // write size to file
    //outfile.write(reinterpret_cast<const char *>(len), sizeof(size_t));
    // write data to file
    outfile.write(reinterpret_cast<const char *>(data), len);
    return outfile.good();
}

std::vector<char> jetnet::read_binary_file(std::string filename)
{
    // Expand user path
    if (filename.find("~") != std::string::npos) {
        filename = std::regex_replace(filename, std::regex("~"), std::string(std::getenv("HOME")));
    }

    std::ifstream infile(filename, std::ifstream::binary | std::istream::ate);

    if (!infile) {
        std::cerr << "Unable to read binary file: " << filename << std::endl;
        return std::vector<char>();
    }

    infile.seekg(0, infile.end);
    int length = infile.tellg();
    infile.seekg(0, infile.beg);

    std::vector<char> buffer(length);
    infile.read(buffer.data(), length);
    infile.close();

    return buffer;
}

bool jetnet::read_text_file(std::vector<std::string>& names, std::string filename)
{
    // Expand user path
    if (filename.find("~") != std::string::npos) {
      filename = std::regex_replace(filename, std::regex("~"), std::string(std::getenv("HOME")));
    }

    std::ifstream infile(filename);

    if (!infile) {
        std::cerr << "Unable to read binary file: " << filename << std::endl;
        return false;
    }

    std::string name;
    while (std::getline(infile, name)) {
        names.push_back(name);
    }

    return true;
}

bool jetnet::save_tensor_text(const float* data, size_t size, std::string filename)
{
    size_t i;
    FILE* fp;

    //TODO: rewrite with ofstream
    fp = fopen(filename.c_str(), "w");
    if (!fp)
        return false;

    for (i=0; i<size; i++) {
        fprintf(fp, "%.8f\n", data[i]);
    }

    fclose(fp);
    return true;
}

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

cv::Mat jetnet::read_image(std::string filename, int expected_channels)
{
    cv::Mat image;
    int cols, rows, channels, type;

    unsigned char *data = stbi_load(filename.c_str(), &cols, &rows, &channels, expected_channels);

    // return empty image on read failure
    if (!data)
        return image;

    if (expected_channels)
        channels = expected_channels;

    switch (channels) {
        case 1: type = CV_8UC1; break;
        case 2: type = CV_8UC2; break;
        case 3: type = CV_8UC3; break;
        case 4: type = CV_8UC4; break;
        default:
            // unsupported number of channels
            stbi_image_free(data);
            return image;
    }

    cv::Mat image_unmanaged(rows, cols, type, data);

    // clone image data to a managed context (need to test overhead of copy) and free stbi memory
    image = image_unmanaged.clone();
    stbi_image_free(data);

    return image;
}
