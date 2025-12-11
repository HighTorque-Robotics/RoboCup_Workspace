#include "jetnet.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using ::testing::Return;
using ::testing::_;

/*
 *  Mocks
 */

// TensorRT API mock
class MockCudaEngine : public nvinfer1::ICudaEngine
{
public:
    MOCK_CONST_METHOD0(getNbBindings, int());
    MOCK_CONST_METHOD1(getBindingIndex, int(const char* name));
    MOCK_CONST_METHOD1(getBindingName, const char* (int bindingIndex));
    MOCK_CONST_METHOD1(bindingIsInput, bool (int bindingIndex));
    MOCK_CONST_METHOD1(getBindingDimensions, nvinfer1::Dims (int bindingIndex));
    MOCK_CONST_METHOD1(getBindingDataType, nvinfer1::DataType (int bindingIndex));
    MOCK_CONST_METHOD0(getMaxBatchSize, int ());
    MOCK_CONST_METHOD0(getNbLayers, int ());
    MOCK_CONST_METHOD0(getWorkspaceSize, size_t ());
    MOCK_CONST_METHOD0(serialize, nvinfer1::IHostMemory* ());
    MOCK_METHOD0(createExecutionContext, nvinfer1::IExecutionContext* ());
    MOCK_METHOD0(destroy, void ());
    MOCK_CONST_METHOD1(getLocation, nvinfer1::TensorLocation (int bindingIndex));
    MOCK_METHOD0(createExecutionContextWithoutDeviceMemory, nvinfer1::IExecutionContext* ());
    MOCK_CONST_METHOD0(getDeviceMemorySize, size_t ());
};

/*
 *  Helper functions
 */

cv::Mat generate_chessboard_image(int num_blocks_width, int num_blocks_height, int block_size,
                                  cv::Scalar fg_color, cv::Scalar bg_color, int type)
{
    int image_width = block_size * num_blocks_width;
    int image_height = block_size * num_blocks_height;

    cv::Mat image = cv::Mat::zeros(cv::Size(image_width, image_height), type);
    bool fg = true;
    for (int i=0; i<image_height; i+=block_size) {
        for (int j=0; j<image_width; j+=block_size) {
            cv::Mat roi_image(image, cv::Rect(j, i, block_size, block_size));
            roi_image = fg ? fg_color : bg_color;
            fg = !fg;
        }

        if (num_blocks_width % 2 == 0) {
            fg = !fg;
        }
    }

    return image;
}

// resize and letterbox like the DUT, but simplified (coordinates are provided i.s.o calculated)
cv::Mat generate_expected_image(cv::Mat image, cv::Size net_size, cv::Rect image_rect)
{
    const int num_channels = image.channels();
    cv::Mat float_image;
    cv::Mat out(cv::Size(net_size.width, net_size.height * num_channels), CV_32FC1);
    std::vector<cv::Mat> channels(num_channels);

    cv::resize(image, image, cv::Size(image_rect.width, image_rect.height), 0, 0, cv::INTER_LINEAR);
    image.convertTo(float_image, CV_32FC(num_channels), 1/255.0);

    for (size_t i=0; i<channels.size(); ++i) {
        channels[i] = cv::Mat(out, cv::Rect(image_rect.x, i * net_size.height + image_rect.y,
                                            image_rect.width, image_rect.height));
    }

    out = cv::Scalar(0.5, 0.5, 0.5);
    cv::split(float_image, channels);

    return out;
}

bool expect_equal_image(cv::Mat expected, cv::Mat actual, float threshold=0)
{
    cv::Mat diff;
    cv::absdiff(expected, actual, diff);
    cv::Scalar val = cv::sum(diff);
    if (val.val[0] <= threshold)
        return true;

    std::cout << val.val[0] << std::endl;
    return false;
}

/*
 *  Test fixtures
 */

class PreprocRunTest : public ::testing::Test
{
protected:

    void SetUp() override
    {
        // create logger needed by DUT
        auto logger = std::make_shared<jetnet::Logger>(nvinfer1::ILogger::Severity::kINFO);

        // define network input dimensions
        dims.nbDims = 3;
        dims.d[0] = channels;
        dims.d[1] = net_h;
        dims.d[2] = net_w;

        // create result blobs the DUT must fill
        jetnet::GpuBlob blob(channels * net_w * net_h);
        blobs.insert(std::pair<int, jetnet::GpuBlob>(0, std::move(blob)));

        // create DUT
        std::vector<unsigned int> channel_map{0, 1, 2};
        pre = std::unique_ptr<jetnet::CvLetterBoxPreProcessor>(
                new jetnet::CvLetterBoxPreProcessor("data", channel_map, logger));

        // initialise DUT
        MockCudaEngine engine;
        EXPECT_CALL(engine, getBindingIndex(_)).WillOnce(Return(0));
        EXPECT_CALL(engine, getBindingDimensions(_)).WillOnce(Return(dims));
        EXPECT_CALL(engine, getMaxBatchSize()).WillOnce(Return(1));

        pre->init(&engine);
    }

    cv::Mat mat_from_blob(jetnet::GpuBlob blob)
    {
        blob.download(raw_data);
        cv::Mat data(cv::Size(net_w, channels * net_h), CV_32FC1,
                            reinterpret_cast<void*>(&raw_data[0]));
        return data;
    }

    nvinfer1::Dims dims;
    int channels = 3;
    int net_w = 32;
    int net_h = 32;

    // DUT
    std::unique_ptr<jetnet::CvLetterBoxPreProcessor> pre;

    std::map<int, jetnet::GpuBlob> blobs;
    std::vector<float> raw_data;
};

/*
 *  Tests
 */

// Test scenario's:
//
//  image_w == net_w, image_h == net_h, same aspect ratio (no letterbox, no resize)
//  image_w <  net_w, image_h == net_h (letterbox left, right, no resize)
//  image_w == net_w, image_h <  net_h (letterbox top, bottom, no resize)
//  image_w >  net_w, image_h == net_h (letterbox top, bottom, resize)
//  image_w == net_w, image_h >  net_h (letterbox left, right, resize)
//  image_w >  net_w, image_h >  net_h, same aspect ratio (no letterbox, resize)
//  image_w <  net_w, image_h <  net_h, same aspect ratio (no letterbox, resize)
//  image_w >> net_w, image_h >  net_h  (letterbox top, bottom, resize)
//  image_w >  net_w, image_h >> net_h  (letterbox left, right, resize)
//  image_w << net_w, image_h <  net_h  (letterbox left, right, resize)
//  image_w <  net_w, image_h << net_h  (letterbox top, bottom, resize)
//  image_w <  net_w, image_h >  net_h  (letterbox left, right, resize)
//  image_w >  net_w, image_h <  net_h  (letterbox top, bottom, resize)
//

//  image_w == net_w, image_h == net_h, same aspect ratio (no letterbox, no resize)
TEST_F(PreprocRunTest, SameWidthSameHeight) 
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(4, 4, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(0, 0, net_w, net_h));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes);  // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));
    EXPECT_TRUE(expect_equal_image(expected, actual));
}

//  image_w <  net_w, image_h == net_h (letterbox left, right, no resize)
TEST_F(PreprocRunTest, SmallerWidthSameHeight) 
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(3, 4, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(net_w / 8, 0, (3 * net_w) / 4, net_h));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes);  // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));
    EXPECT_TRUE(expect_equal_image(expected, actual));
}

//  image_w == net_w, image_h <  net_h (letterbox top, bottom, no resize)
TEST_F(PreprocRunTest, SameWidthSmallerHeight) 
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(4, 3, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(0, net_h / 8, net_w, (3 * net_h) / 4));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));
    EXPECT_TRUE(expect_equal_image(expected, actual));
}

//  image_w >  net_w, image_h == net_h (letterbox top, bottom, downsample)
TEST_F(PreprocRunTest, LargerWidthSameHeight) 
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(8, 4, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(0, net_h / 4, net_w, net_h / 2));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));
    EXPECT_TRUE(expect_equal_image(expected, actual));
}

//  image_w == net_w, image_h >  net_h (letterbox left, right, downsample)
TEST_F(PreprocRunTest, SameWidthLargerHeight) 
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(4, 8, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(net_w / 4, 0, net_w / 2, net_h));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));
    EXPECT_TRUE(expect_equal_image(expected, actual));
}

//  image_w >  net_w, image_h >  net_h, same aspect ratio (no letterbox, downsample)
TEST_F(PreprocRunTest, LargerWidthLargerHeightSameAspect) 
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(8, 8, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(0, 0, net_w, net_h));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));
    EXPECT_TRUE(expect_equal_image(expected, actual));
}

//  image_w <  net_w, image_h <  net_h, same aspect ratio (no letterbox, upsample)
TEST_F(PreprocRunTest, SmallerWidthSmallerHeightSameAspect) 
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(2, 2, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(0, 0, net_w, net_h));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));

    // larger threshold to be tolerant to differences in upsample interpolation
    EXPECT_TRUE(expect_equal_image(expected, actual, 100.0));
}

//  image_w >> net_w, image_h >  net_h  (letterbox top, bottom, downsample)
TEST_F(PreprocRunTest, MuchLargerWidthLargerHeight) 
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(16, 8, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(0, net_h / 4, net_w, net_h / 2));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));
    EXPECT_TRUE(expect_equal_image(expected, actual));
}

//  image_w >  net_w, image_h >> net_h  (letterbox left, right, resize)
TEST_F(PreprocRunTest, LargerWidthMuchLargerHeight) 
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(8, 16, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(net_w / 4, 0, net_w / 2, net_h));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));
    EXPECT_TRUE(expect_equal_image(expected, actual));
}

//  image_w << net_w, image_h <  net_h  (letterbox left, right, upsample)
TEST_F(PreprocRunTest, MuchSmallerWidthSmallerHeight)
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(1, 2, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(net_w / 4, 0, net_w / 2, net_h));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));

    // larger threshold to be tolerant to differences in upsample interpolation
    EXPECT_TRUE(expect_equal_image(expected, actual, 100.0));
}

//  image_w <  net_w, image_h << net_h  (letterbox top, bottom, upsample)
TEST_F(PreprocRunTest, SmallerWidthMuchSmallerHeight)
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(2, 1, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(0, net_h / 4, net_w, net_h / 2));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));

    // larger threshold to be tolerant to differences in upsample interpolation
    EXPECT_TRUE(expect_equal_image(expected, actual, 100.0));
}

//  image_w <  net_w, image_h >  net_h  (letterbox left, right, downsample)
TEST_F(PreprocRunTest, SmallerWidthLargerHeight)
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(2, 8, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect((3 * net_w) / 8, 0, net_w / 4, net_h));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));
    EXPECT_TRUE(expect_equal_image(expected, actual));
}

//  image_w >  net_w, image_h <  net_h  (letterbox top, bottom, downsample)
TEST_F(PreprocRunTest, LargerWidthSmallerHeight)
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    inputs.push_back(generate_chessboard_image(8, 2, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3));
    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(0, (3 * net_h) / 8, net_w, net_h / 4));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));
    EXPECT_TRUE(expect_equal_image(expected, actual));
}

// image_w == net_w - 1, image_h == net_h (letterbox top only, no resize)
TEST_F(PreprocRunTest, WidthMinusOneSameHeight)
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    cv::Mat input_base = generate_chessboard_image(4, 4, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3);
    inputs.push_back(input_base(cv::Rect(0, 0, input_base.cols, input_base.rows - 1)));

    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(0, 1, net_w, net_h - 1));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));
    EXPECT_TRUE(expect_equal_image(expected, actual));
}

// image_w == net_w, image_h == net_h - 1 (letterbox left only, no resize)
TEST_F(PreprocRunTest, SameWidthHeightMinusOne)
{
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> inputs;

    // setup
    cv::Mat input_base = generate_chessboard_image(4, 4, net_w / 4, cv::Scalar(255, 255, 255),
                                               cv::Scalar(0, 0, 0), CV_8UC3);
    inputs.push_back(input_base(cv::Rect(0, 0, input_base.cols - 1, input_base.rows)));

    // calculate expected output
    cv::Mat expected = generate_expected_image(inputs[0], cv::Size(net_w, net_h),
                                               cv::Rect(1, 0, net_w - 1, net_h));
    // act
    pre->register_images(inputs);
    bool res = (*pre)(blobs, image_sizes); // blobs are pre-allocated by test fixture

    // assert
    ASSERT_TRUE(res);
    cv::Mat actual = mat_from_blob(std::move(blobs.at(0)));
    EXPECT_TRUE(expect_equal_image(expected, actual));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
