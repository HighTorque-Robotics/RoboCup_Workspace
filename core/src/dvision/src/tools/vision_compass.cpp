#include "dvision/vision_compass.hpp"



namespace dvision {
  //! TODO(JXR): mask feild region

  //! Initialize VisionCompass
  void VisionCompass::Init(double fx, double fy, double ucx, double ucy){
    Timer t;
    //! camera instrinsic paras
    fx_ = fx;
    fy_ = fy;
    cx_ = ucx;
    cy_ = ucy;
    loadInfo();
    
    //! initialize
    keypoints_list = std::vector<std::vector<cv::KeyPoint>>(imgs.size()); 
    descriptors_list = std::vector<cv::Mat>(imgs.size());
    detector = cv::ORB::create();
    descriptor = cv::ORB::create();
    matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    max_keypoints = 0;

    //! ORB keypoints detection
    for(int i = 0; i < imgs.size(); i++){
      detector->detect(imgs[i], keypoints_list[i]);
      max_keypoints = MAX(max_keypoints, keypoints_list[i].size());
    }

    //! calculate brief descriptors
    for(int i = 0; i < imgs.size(); i++){
      descriptor->compute(imgs[i], keypoints_list[i], descriptors_list[i]);
    }

    //! calculate theta under the global coordinate
    std::vector<std::vector<double>> thetas_list(imgs.size(),std::vector<double>(max_keypoints));
    for(int i = 0; i < imgs.size(); i++){
      for(int j = 0; j < keypoints_list[i].size(); j++){
        thetas_list[i][j] = CalRawTheta(keypoints_list[i][j].pt) + camera_poses[i].head_yaw + camera_poses[i].imu_yaw;
      }
    }

    ROS_INFO("[Vision Compass]: init done, use %lf ms", t.elapsedMsec());
  }

  void VisionCompass::loadInfo(){
    //! load parameters
    img_dir = parameters.vision_compass.img_dir;
    paras_path = parameters.vision_compass.paras_path;
    
    //! 读取图像
    if((dp = opendir(img_dir.c_str())) == NULL)
        ROS_INFO("Can't open %s", img_dir);
    while((dirp = readdir(dp)) != NULL){
        //! ignore uncommon file type
        if(dirp->d_type != 8) continue;
            img_names.push_back(img_dir + "/" + dirp->d_name);
    }
    sort(img_names.begin(), img_names.end());

    for(int i = 0; i < img_names.size(); i++){
        cv::Mat pSrc;
        pSrc = cv::imread(img_names[i]); 
        imgs.push_back(pSrc);
    }
    closedir(dp);

    //! load imu_yaw && head_command
    std::ifstream txt_in (paras_path, std::ifstream::in);
    double a, b, c;
    
    for(int i = 0; i < imgs.size(); i++){
        txt_in >> a >> b >> c;
        camera_poses.push_back( CameraPose(a,b,c) );
    }
    txt_in.close();
  }

  double VisionCompass::Process(const cv::Mat &target_img, double imu_yaw, double head_yaw){

    detector->detect(target_img, target_keypoints);
    descriptor->compute(target_img, target_keypoints, target_descriptors);

    for(int i = 0; i < imgs.size(); i++){
      //! 如果两张图像视角相差过大，则不进行匹配
      if(abs(camera_poses[i].head_yaw + camera_poses[i].imu_yaw - 
        head_yaw - imu_yaw) > 50)
        continue;

      std::vector<cv::DMatch> matches, good_matches;
      matcher->match(descriptors_list[i], target_descriptors, matches);

      //! 匹配点对筛选
      //! 计算最小距离和最大距离
      auto min_max = minmax_element(matches.begin(), matches.end(),
                                    [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
      double min_dist = min_max.first->distance;
      double max_dist = min_max.second->distance;

      //! 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
      for (int j = 0; j < descriptors_list[i].rows; j++) {
        if (matches[j].distance <= MAX(2 * min_dist, 30.0)) {
          good_matches.push_back(matches[j]);    
          psrc = keypoints_list[i][matches[j].queryIdx].pt;
          pdst = target_keypoints[matches[j].trainIdx].pt;
          theta_src = CalRawTheta(psrc) - camera_poses[i].head_yaw - camera_poses[i].imu_yaw;
          theta_dst = CalRawTheta(pdst) - head_yaw - imu_yaw;
          theta_delta = theta_src - theta_dst; 
          delta_list.push_back(theta_delta);
          }
      }
    }
    double sum = accumulate(begin(delta_list), end(delta_list), 0.0);
    double mean =  sum / delta_list.size();
    return mean;
  }

} // namespace dvision