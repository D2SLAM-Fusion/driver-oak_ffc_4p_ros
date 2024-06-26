#ifndef OAK_FFC_4P_H_
#define OAK_FFC_4P_H_
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <chrono>
#include <depthai/depthai.hpp>
#include <depthai/utility/Clock.hpp>
#include <thread>
#include <vector>

namespace OAKCAM {

class FFC4PDriver {
 public:
  // Mono cam resolution
  std::map<std::string, dai::MonoCameraProperties::SensorResolution>
      mono_res_opts = {
          {"400", dai::MonoCameraProperties::SensorResolution::THE_400_P},
          {"480", dai::MonoCameraProperties::SensorResolution::THE_480_P},
          {"720", dai::MonoCameraProperties::SensorResolution::THE_720_P},
          {"800", dai::MonoCameraProperties::SensorResolution::THE_800_P},
          {"1200", dai::MonoCameraProperties::SensorResolution::THE_1200_P},
  };

  // rgb cam resolution
  std::map<std::string, dai::ColorCameraProperties::SensorResolution>
      color_res_opts = {
          {"720", dai::ColorCameraProperties::SensorResolution::THE_720_P},
          {"800", dai::ColorCameraProperties::SensorResolution::THE_800_P},
          {"1080", dai::ColorCameraProperties::SensorResolution::THE_1080_P},
          {"1200", dai::ColorCameraProperties::SensorResolution::THE_1200_P},
          {"4k", dai::ColorCameraProperties::SensorResolution::THE_4_K},
          {"5mp", dai::ColorCameraProperties::SensorResolution::THE_5_MP},
          {"12mp", dai::ColorCameraProperties::SensorResolution::THE_12_MP},
          {"48mp", dai::ColorCameraProperties::SensorResolution::THE_48_MP},
  };

  struct CameraModuleConfig {
    bool rgb = true;
    bool auto_expose = false;
    bool ros_defined_freq = true;
    bool show_img_info = false;
    bool auto_awb = false;
    bool sharpness_calibration_mode = false;
    bool compressed_mode = false;
    bool enable_upside_down = false;
    int32_t fps = 30.0;
    int32_t resolution = 720;
    int32_t expose_time_us = 10000;
    int32_t iso = 400;
    int32_t awb_value = 4000;
  };

  struct FFCCameraConfig {
    dai::CameraBoardSocket socket;
    dai::ColorCameraProperties::SensorResolution resolution =
        dai::ColorCameraProperties::SensorResolution::THE_720_P;
    std::string stream_name;
    bool is_master;
    FFCCameraConfig(dai::CameraBoardSocket sck,
                    dai::ColorCameraProperties::SensorResolution res,
                    std::string name, bool master)
        : socket(sck), resolution(res), stream_name(name), is_master(master){};
  };

  struct ImagePubNode {
    std::string topic;
    std::shared_ptr<ros::Publisher> ros_publisher_ptr;
    cv::Mat image;
    std::chrono::time_point<std::chrono::steady_clock,
                            std::chrono::steady_clock::duration>
        cap_time_stamp;
    int32_t frame_counter;
    ImagePubNode() {}
    ImagePubNode(std::shared_ptr<ros::NodeHandle> ros_node, std::string topic)
        : topic(topic) {
      ros_publisher_ptr = std::make_shared<ros::Publisher>(
          ros_node->advertise<sensor_msgs::Image>(topic, 1));
      frame_counter = 0;
    };
  };

  std::vector<FFCCameraConfig> CameraList = {
      {dai::CameraBoardSocket::CAM_A,
       dai::ColorCameraProperties::SensorResolution::THE_720_P,
       std::string("CAM_A"), true},  //
      {dai::CameraBoardSocket::CAM_B,
       dai::ColorCameraProperties::SensorResolution::THE_720_P,
       std::string("CAM_B"), false},
      {dai::CameraBoardSocket::CAM_C,
       dai::ColorCameraProperties::SensorResolution::THE_720_P,
       std::string("CAM_C"), false},
      {dai::CameraBoardSocket::CAM_D,
       dai::ColorCameraProperties::SensorResolution::THE_720_P,
       std::string("CAM_D"), false}};

  FFC4PDriver(std::shared_ptr<ros::NodeHandle>& nh);
  ~FFC4PDriver();
  void GetParameters(ros::NodeHandle& nh);
  int32_t Init();
  void StartVideoStream();
  void GrapThreadJoin() { grab_thread_.join(); };
  void StopVideoStream() {
    is_run_ = false;
    return;
  };

 private:
  void RosGrabImgThread();
  void StdGrabImgThread();
  void GrabImg();
  void ShowImg(ImagePubNode& image_node,
               std::chrono::_V2::steady_clock::time_point& time_now);

  ros::Publisher expose_time_publisher_;
  ros::Publisher assemble_image_publisher_;  // to publish all camera images in

  std::unique_ptr<dai::Pipeline> pipeline_ = nullptr;
  std::unique_ptr<dai::Device> device_ = nullptr;

  std::map<std::string, std::shared_ptr<dai::node::ColorCamera>> cam_color_;
  std::map<std::string, std::shared_ptr<dai::node::MonoCamera>> cam_mono_;
  std::map<std::string, ImagePubNode> image_pub_node_;

  int32_t device_is_detected_ = 0;
  int32_t pipeline_is_init_ = 0;
  CameraModuleConfig module_config_;

  // config translate
  dai::ColorCameraProperties::SensorResolution rgb_resolution_;
  dai::MonoCameraProperties::SensorResolution mono_resolution_;

  // ros
  std::shared_ptr<ros::NodeHandle> ros_node_ = nullptr;
  ros::Timer thread_timer_;
  std::unique_ptr<ros::Rate> ros_rate_ptr_ = nullptr;

  // thread
  std::thread grab_thread_;
  bool is_run_ = true;
  // image_tmp
};

double Clearness(cv::Mat& img);  // calculate clearness to manuel focus

ros::Time convertToRosTime(
    const std::chrono::time_point<std::chrono::steady_clock,
                                  std::chrono::steady_clock::duration>&
        time_point);

}  // namespace OAKCAM

#endif