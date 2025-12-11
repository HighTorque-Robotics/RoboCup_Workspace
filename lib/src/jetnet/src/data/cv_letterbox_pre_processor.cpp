#include "cv_letterbox_pre_processor.h"
#include "custom_assert.h"
#include <cuda_runtime.h>

using namespace jetnet;
using namespace nvinfer1;

CvLetterBoxPreProcessor::CvLetterBoxPreProcessor(std::string input_blob_name,
                                                 std::vector<unsigned int> channel_map,
                                                 std::shared_ptr<Logger> logger) :
    m_input_blob_name(input_blob_name),
    m_channel_map(channel_map),
    m_logger(logger)
{
}

bool CvLetterBoxPreProcessor::init(DimsCHW input_dims, size_t max_batch_size, int input_blob_index)
{
    m_net_in_w = input_dims.w();
    m_net_in_h = input_dims.h();
    m_net_in_c = input_dims.c();
    m_max_batch_size = max_batch_size;
    m_input_blob_index = input_blob_index;

    m_in_row_step = m_net_in_w;
    m_in_channel_step = m_net_in_w * m_net_in_h;
    m_in_batch_step = m_net_in_w * m_net_in_h * m_net_in_c;

    if (m_channel_map.size() != static_cast<size_t>(m_net_in_c)) {
        m_logger->log(ILogger::Severity::kERROR, "The size of the channel map (" + std::to_string(m_channel_map.size()) +
                      ") does not match the number of network input channels (" + std::to_string(m_net_in_c) + ")");
        return false;
    }

    m_image_resized = cv::cuda::GpuMat(m_net_in_h, m_net_in_w, CV_8UC(m_net_in_c));
    m_image_resized_float = cv::cuda::GpuMat(m_net_in_h, m_net_in_w, CV_32FC(m_net_in_c));

    return true;
}

bool CvLetterBoxPreProcessor::init(const ICudaEngine* engine)
{
    int index = engine->getBindingIndex(m_input_blob_name.c_str());
    Dims dims = engine->getBindingDimensions(index);
    DimsCHW dims_chw(dims.d[0], dims.d[1], dims.d[2]);

    return init(dims_chw, engine->getMaxBatchSize(), index);
}

void CvLetterBoxPreProcessor::register_images(std::vector<cv::Mat> images)
{
    m_registered_images = images;
    m_use_direct_gpu_input = false;
}


void CvLetterBoxPreProcessor::register_images(std::vector<cv::cuda::GpuMat> images)
{
    m_registered_images_cuda = images;
    m_use_direct_gpu_input = true;
}

bool CvLetterBoxPreProcessor::operator()(std::map<int, GpuBlob>& input_blobs, std::vector<cv::Size>& image_sizes)
{
    // ensure input_blobs are allocated
    if (input_blobs.empty()) {
        GpuBlob blob(m_in_batch_step * m_max_batch_size);
        input_blobs.insert(std::pair<int, GpuBlob>(m_input_blob_index, std::move(blob)));
    }

    if (!m_use_direct_gpu_input) {
      if (m_registered_images.size() > m_max_batch_size) {
        m_logger->log(ILogger::Severity::kERROR, "The number of registered images (" +
            std::to_string(m_registered_images.size()) + ") exceeds the maximum batch size ");
        return false;
      }

      // fill all batches of the input blob
      float* data = reinterpret_cast<float*>(input_blobs.at(m_input_blob_index).get());
      for (size_t i=0; i<m_registered_images.size(); i++) {
        if (!cv_to_tensor_data(m_registered_images[i], &data[i * m_in_batch_step]))
          return false;
        image_sizes.push_back(cv::Size(m_registered_images[i].cols, m_registered_images[i].rows));
      }
    } else {
      if (m_registered_images_cuda.size() > m_max_batch_size) {
        m_logger->log(ILogger::Severity::kERROR, "The number of registered images (" +
            std::to_string(m_registered_images_cuda.size()) + ") exceeds the maximum batch size ");
        return false;
      }

      // fill all batches of the input blob
      float* data = reinterpret_cast<float*>(input_blobs.at(m_input_blob_index).get());
      for (size_t i=0; i<m_registered_images_cuda.size(); i++) {
        if (!cv_to_tensor_data(m_registered_images_cuda[i], &data[i * m_in_batch_step]))
          return false;
        image_sizes.push_back(cv::Size(m_registered_images_cuda[i].cols, m_registered_images_cuda[i].rows));
      }

    }

    return true;
}

bool CvLetterBoxPreProcessor::cv_to_tensor_data(const cv::Mat& input, float* output)
{
    const int in_width = input.cols;
    const int in_height = input.rows;
    const cv::Scalar border_color = cv::Scalar(0.5, 0.5, 0.5);
    cv::Rect rect_image, rect_greyborder1, rect_greyborder2;
    cv::cuda::GpuMat input_gpu, roi_image, roi_image_float, roi_greyborder1, roi_greyborder2;
    std::vector<cv::cuda::GpuMat> float_mat_channels(3);

    if (input.channels() != m_net_in_c) {
        m_logger->log(ILogger::Severity::kERROR, "Number of image channels (" + std::to_string(input.channels()) +
                      ") does not match number of network input channels (" + std::to_string(m_net_in_c) + ")");
        return false;
    }

    if (input.depth() != CV_8U) {
        m_logger->log(ILogger::Severity::kERROR, "This pre-processor currently only supports CV_8U image channels");
        return false;
    }

    // copy input image to gpu
    input_gpu.upload(input);

    // assign Mat objs in case some steps can be bypassed
    roi_image_float = m_image_resized_float;

    // if image does not fit network input resolution, check if resize and or letterboxing is needed
    if (in_width != m_net_in_w || in_height != m_net_in_h) {

        // if aspect ratio differs, apply letterboxing
        if (in_height * m_net_in_w != in_width * m_net_in_h) {

            // calculate rectangles for letterboxing
            if (in_height * m_net_in_w < in_width * m_net_in_h) {
                const int image_h = (in_height * m_net_in_w) / in_width;
                const int border_h = std::ceil((m_net_in_h - image_h) / 2.0);
                rect_image = cv::Rect(0, border_h, m_net_in_w, image_h);
                rect_greyborder1 = cv::Rect(0, 0, m_net_in_w, border_h);
                rect_greyborder2 = cv::Rect(0, (m_net_in_h + image_h) / 2, m_net_in_w, border_h);
            } else {
                const int image_w = (in_width * m_net_in_h) / in_height;
                const int border_w = std::ceil((m_net_in_w - image_w) / 2.0);
                rect_image = cv::Rect(border_w, 0, image_w, m_net_in_h);
                rect_greyborder1 = cv::Rect(0, 0, border_w, m_net_in_h);
                rect_greyborder2 = cv::Rect((m_net_in_w + image_w) / 2, 0, border_w, m_net_in_h);
            }

            roi_image = cv::cuda::GpuMat(m_image_resized, rect_image);
            roi_image_float = cv::cuda::GpuMat(m_image_resized_float, rect_image);          // image area

            roi_greyborder1 = cv::cuda::GpuMat(m_image_resized_float, rect_greyborder1);    // grey area top/left
            roi_greyborder2 = cv::cuda::GpuMat(m_image_resized_float, rect_greyborder2);    // grey area bottom/right

            // paint borders grey
            roi_greyborder1.setTo(border_color);
            roi_greyborder2.setTo(border_color);

        } else {
            roi_image = m_image_resized;
        }

        // resize if dimensions of the input image are different from the
        // ROI dimensions within the network input area
        if (input_gpu.size() != roi_image.size()) {
            cv::cuda::resize(input_gpu, roi_image, roi_image.size(), 0, 0, cv::INTER_LINEAR);
        } else {
            roi_image = input_gpu;
        }

    } else {
        roi_image = input_gpu;
    }

    // uint8 to float image and normalise (between 0 and 1)
    roi_image.convertTo(roi_image_float, CV_32FC(m_net_in_c), 1/255.0);

    // Note: output is a pointer to a GPU buffer
    // Remap image channels from XYZXYZXYZ...XYZ -> XXX...XXXYYY...YYYZZZ...ZZZ
    for (size_t i=0; i<m_channel_map.size(); ++i) {
        float_mat_channels[m_channel_map[i]] = cv::cuda::GpuMat(m_net_in_h, m_net_in_w, CV_32FC1, &output[i * m_in_channel_step]);
    }

    cv::cuda::split(m_image_resized_float, float_mat_channels);

    return true;
}

bool CvLetterBoxPreProcessor::cv_to_tensor_data(const cv::cuda::GpuMat& input_gpu, float* output)
{
  const int in_width = input_gpu.cols;
  const int in_height = input_gpu.rows;
  const cv::Scalar border_color = cv::Scalar(0.5, 0.5, 0.5);
  cv::Rect rect_image, rect_greyborder1, rect_greyborder2;
  cv::cuda::GpuMat roi_image, roi_image_float, roi_greyborder1, roi_greyborder2;
  std::vector<cv::cuda::GpuMat> float_mat_channels(3);

  if (input_gpu.channels() != m_net_in_c) {
    m_logger->log(ILogger::Severity::kERROR, "Number of image channels (" + std::to_string(input_gpu.channels()) +
        ") does not match number of network input channels (" + std::to_string(m_net_in_c) + ")");
    return false;
  }

  if (input_gpu.depth() != CV_8U) {
    m_logger->log(ILogger::Severity::kERROR, "This pre-processor currently only supports CV_8U image channels");
    return false;
  }

  // assign Mat objs in case some steps can be bypassed
  roi_image_float = m_image_resized_float;

  // if image does not fit network input resolution, check if resize and or letterboxing is needed
  if (in_width != m_net_in_w || in_height != m_net_in_h) {

    // if aspect ratio differs, apply letterboxing
    if (in_height * m_net_in_w != in_width * m_net_in_h) {

      // calculate rectangles for letterboxing
      if (in_height * m_net_in_w < in_width * m_net_in_h) {
        const int image_h = (in_height * m_net_in_w) / in_width;
        const int border_h = std::ceil((m_net_in_h - image_h) / 2.0);
        rect_image = cv::Rect(0, border_h, m_net_in_w, image_h);
        rect_greyborder1 = cv::Rect(0, 0, m_net_in_w, border_h);
        rect_greyborder2 = cv::Rect(0, (m_net_in_h + image_h) / 2, m_net_in_w, border_h);
      } else {
        const int image_w = (in_width * m_net_in_h) / in_height;
        const int border_w = std::ceil((m_net_in_w - image_w) / 2.0);
        rect_image = cv::Rect(border_w, 0, image_w, m_net_in_h);
        rect_greyborder1 = cv::Rect(0, 0, border_w, m_net_in_h);
        rect_greyborder2 = cv::Rect((m_net_in_w + image_w) / 2, 0, border_w, m_net_in_h);
      }

      roi_image = cv::cuda::GpuMat(m_image_resized, rect_image);
      roi_image_float = cv::cuda::GpuMat(m_image_resized_float, rect_image);          // image area

      roi_greyborder1 = cv::cuda::GpuMat(m_image_resized_float, rect_greyborder1);    // grey area top/left
      roi_greyborder2 = cv::cuda::GpuMat(m_image_resized_float, rect_greyborder2);    // grey area bottom/right

      // paint borders grey
      roi_greyborder1.setTo(border_color);
      roi_greyborder2.setTo(border_color);

    } else {
      roi_image = m_image_resized;
    }

    // resize if dimensions of the input image are different from the
    // ROI dimensions within the network input area
    if (input_gpu.size() != roi_image.size()) {
      cv::cuda::resize(input_gpu, roi_image, roi_image.size(), 0, 0, cv::INTER_LINEAR);
    } else {
      roi_image = input_gpu;
    }

  } else {
    roi_image = input_gpu;
  }

  // uint8 to float image and normalise (between 0 and 1)
  roi_image.convertTo(roi_image_float, CV_32FC(m_net_in_c), 1/255.0);

  // Note: output is a pointer to a GPU buffer
  // Remap image channels from XYZXYZXYZ...XYZ -> XXX...XXXYYY...YYYZZZ...ZZZ
  for (size_t i=0; i<m_channel_map.size(); ++i) {
    float_mat_channels[m_channel_map[i]] = cv::cuda::GpuMat(m_net_in_h, m_net_in_w, CV_32FC1, &output[i * m_in_channel_step]);
  }

  cv::cuda::split(m_image_resized_float, float_mat_channels);

  return true;
}
