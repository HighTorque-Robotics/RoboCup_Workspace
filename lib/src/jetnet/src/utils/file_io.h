#ifndef JETNET_FILE_IO_H
#define JETNET_FILE_IO_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace jetnet
{
    /*
     *  data:       binary data to be written
     *  len:        number of bytes in data
     *  filename:   filename to write to
     *  returns true on success, false on failure
     */
    bool write_binary_file(const void* data, size_t len, std::string filename);

    /*
     *  filename:   filename to read from
     *  returns a char vector containing the bytes of the file. On failure of reading the vector is empty
     */
    std::vector<char> read_binary_file(std::string filename);

    /*
     *  Read a text file
     *  lines:      text lines in file
     *  filename:   name of the file to read
     *  returns true on success, false on failure
     */
    bool read_text_file(std::vector<std::string>& lines, std::string filename);

    /*
     *  Save a float tensor to a file in text format (one float text value per line)
     */
    bool save_tensor_text(const float* data, size_t size, std::string filename);

    /*
     *  Read an image from disk to opencv. This reader is faster but less versatile compared to
     *  opencv's imread function. See stb_image.h for more details on supported image formats.
     *
     *  filename:           filename of the image
     *  expected_channels:  number of 8-bit components per pixel. If this is zero, the number
     *                      of image channels present in the image will be read, otherwise:
     *
     *                      N=#comp     components
     *                      1           grey
     *                      2           grey, alpha
     *                      3           red, green, blue
     *                      4           red, green, blue, alpha
     *
     *                      For example, if a grey image is read and expected_channels = 3,
     *                      the resulting mat will have 3 grey channels
     */
    cv::Mat read_image(std::string filename, int expected_channels = 0);
};

#endif /* JETNET_FILE_IO_H */
