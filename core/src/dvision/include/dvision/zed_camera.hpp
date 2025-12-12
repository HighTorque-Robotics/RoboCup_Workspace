#pragma once

#include <sl/Camera.hpp>
#include <dvision/icamera.hpp>

namespace dvision {
  //! ZED M 相机捕获流类
  class ZedCamera : public ICamera {
  public:
    //! 构造函数
    ZedCamera();

    //! 析构函数
    ~ZedCamera() override;

    /**
     * \brief 初始化相机
     *
     * \return 初始化结果
     */
    bool initialize();

    /**
     * \brief 捕获相机图像
     *
     * \return 捕获的图像
     */
    Frame capture() override;

  private:
    sl::Camera zed;  // ZED 相机对象
    sl::Mat image;   // 图像数据
  };
}