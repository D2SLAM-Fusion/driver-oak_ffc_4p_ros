#pragma once
#include "oak_ffc_4p.h"

#include <cv_bridge/cv_bridge.h>
#include <fcntl.h>
#include <image_transport/image_transport.h>
#include <linux/videodev2.h>
#include <memory.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <depthai/build/version.hpp>
#include <depthai/depthai.hpp>
#include <depthai/utility/Clock.hpp>
#include <depthai-shared/datatype/RawCameraControl.hpp>
#include <deque>
#include <iostream>
#include <list>
#include <map>
#include <opencv2/opencv.hpp>
#include <vector>

#define SECOND 1000000
#define IMAGE_WIDTH 1280

namespace OAKCAM {

FFC4PDriver::FFC4PDriver(std::shared_ptr<ros::NodeHandle>& nh) {
  if (nh == nullptr) {
    ROS_ERROR("Init with a invalid Nodehandler");
    return;
  }
  cv::setNumThreads(1);
  this->ros_node_ = nh;
  ROS_INFO("FFC 4P Device Detecting\n");
  auto deviceInfoVec = dai::Device::getAllAvailableDevices();
  const auto usbSpeed = dai::UsbSpeed::SUPER_PLUS;
  auto openVinoVersion = dai::OpenVINO::Version::VERSION_2021_4;
  if (deviceInfoVec.size() != 1) {
    ROS_ERROR("Multiple devices or No device detected\n");
    this->device_is_detected_ = 0;
    return;
  }
  this->device_ = std::make_unique<dai::Device>(
      openVinoVersion, deviceInfoVec.front(), usbSpeed);
  if (device_ == nullptr) {
    ROS_ERROR("device init failed\n");
    return;
  }
  // print device infomation
  std::cout << "===Connected to " << deviceInfoVec.front().getMxId()
            << std::endl;
  auto mxId = this->device_->getMxId();
  auto cameras = this->device_->getConnectedCameras();
  auto usbSpeed_dev = this->device_->getUsbSpeed();
  auto eepromData = this->device_->readCalibration2().getEepromData();
  std::cout << "   >>> MXID:" << mxId << std::endl;
  std::cout << "   >>> Num of cameras:" << cameras.size() << std::endl;
  std::cout << "   >>> USB speed:" << usbSpeed_dev << std::endl;
  if (eepromData.boardName != "") {
    std::cout << "   >>> Board name:" << eepromData.boardName << std::endl;
  }
  if (eepromData.productName != "") {
    std::cout << "   >>> Product name:" << eepromData.productName << std::endl;
  }
  this->device_is_detected_ = 1;
  ROS_INFO("FFC 4P Device detected!\n");
  this->GetParameters(*nh);
}

FFC4PDriver::~FFC4PDriver() { this->device_->close(); }

void FFC4PDriver::GetParameters(ros::NodeHandle& nh) {
  nh.getParam("fps", this->module_config_.fps);
  nh.getParam("rgb", this->module_config_.rgb);
  nh.getParam("resolution", this->module_config_.resolution);
  nh.getParam("auto_expose", this->module_config_.auto_expose);
  nh.getParam("expose_time_us", this->module_config_.expose_time_us);
  nh.getParam("iso", this->module_config_.iso);
  nh.getParam("image_info", this->module_config_.show_img_info);
  nh.getParam("auto_awb", this->module_config_.auto_awb);
  nh.getParam("awb_value", this->module_config_.awb_value);
  nh.getParam("ros_defined_freq", this->module_config_.ros_defined_freq);
  nh.getParam("sharpness_calibration_mode",
              this->module_config_.sharpness_calibration_mode);
  nh.getParam("compressed_mode", this->module_config_.compressed_mode);
  nh.getParam("enable_upside_down", this->module_config_.enable_upside_down);
  if (this->module_config_.rgb) {
    if (color_res_opts.find(this->module_config_.resolution) ==
        color_res_opts.end()) {
      ROS_ERROR("Resolution %d not supported, use default\n",
                this->module_config_.resolution);
      this->rgb_resolution_ = color_res_opts["720"];
    } else {
      this->rgb_resolution_ = color_res_opts[this->module_config_.resolution];
    }
  } else {
    if (mono_res_opts.find(this->module_config_.resolution) ==
        mono_res_opts.end()) {
      ROS_ERROR("Resolution %d not supported, use default\n",
                this->module_config_.resolution);
      this->mono_resolution_ = mono_res_opts["720"];
    } else {
      this->mono_resolution_ = mono_res_opts[this->module_config_.resolution];
    }
  }
  return;
}

int32_t FFC4PDriver::Init() {
  // Init and Start pipeline
  this->pipeline_ = std::make_unique<dai::Pipeline>();
  this->pipeline_->setXLinkChunkSize(0);
  auto sync = this->pipeline_->create<dai::node::Sync>();
  auto xOut = this->pipeline_->create<dai::node::XLinkOut>();
  xOut->setStreamName("msgOut");
  sync->out.link(xOut->input);
  for (int i = 0; i < this->CameraList.size(); i++) {
    if (this->module_config_.rgb) {  // RGB camera
      auto rgb_cam = this->pipeline_->create<dai::node::ColorCamera>();
      rgb_cam->setResolution(this->rgb_resolution_);
      rgb_cam->setBoardSocket(CameraList[i].socket);
      rgb_cam->setInterleaved(false);
      rgb_cam->setFps(this->module_config_.fps);
      rgb_cam->isp.link(sync->inputs[CameraList[i].stream_name]);
      rgb_cam->initialControl.setFrameSyncMode(
          CameraList[i].is_master ? dai::CameraControl::FrameSyncMode::OUTPUT
                                  : dai::CameraControl::FrameSyncMode::INPUT);
      if (this->module_config_.auto_expose) {
        rgb_cam->initialControl.setAutoExposureEnable();
      } else {
        rgb_cam->initialControl.setManualExposure(
            this->module_config_.expose_time_us, this->module_config_.iso);
      }
      if (!this->module_config_.auto_awb) {
        rgb_cam->initialControl.setManualWhiteBalance(
            this->module_config_.awb_value);
      }
      this->cam_color_[CameraList[i].stream_name] = rgb_cam;
    } else {
      auto mono_cam = this->pipeline_->create<dai::node::MonoCamera>();
      mono_cam->setResolution(this->mono_resolution_);
      mono_cam->setBoardSocket(CameraList[i].socket);
      mono_cam->setFps(this->module_config_.fps);
      mono_cam->out.link(sync->inputs[CameraList[i].stream_name]);
      mono_cam->initialControl.setFrameSyncMode(
          CameraList[i].is_master ? dai::CameraControl::FrameSyncMode::OUTPUT
                                  : dai::CameraControl::FrameSyncMode::INPUT);
      if (this->module_config_.auto_expose) {
        mono_cam->initialControl.setAutoExposureEnable();
      } else {
        mono_cam->initialControl.setManualExposure(
            this->module_config_.expose_time_us, this->module_config_.iso);
      }
      if (!this->module_config_.auto_awb) {
        mono_cam->initialControl.setManualWhiteBalance(
            this->module_config_.awb_value);
      }
      this->cam_mono_[CameraList[i].stream_name] = mono_cam;
    }
  }
  this->device_->startPipeline(*this->pipeline_);

  // create ros publisher
  if (this->module_config_.sharpness_calibration_mode) {
    ROS_INFO("Sharpness Calibration mode\n");
    for (int i = 0; i < this->CameraList.size(); i++) {
      std::string topic_name = "/oak_ffc_4p/" + CameraList[i].stream_name;
      ImagePubNode image_node(this->ros_node_, topic_name);
      image_pub_node_[CameraList[i].stream_name] = image_node;
    }
  } else {
    this->expose_time_publisher_ = this->ros_node_->advertise<std_msgs::Int32>(
        "/oak_ffc_4p/expose_time_us", 1);
    if (this->module_config_.compressed_mode) {
      this->assemble_image_publisher_ =
          this->ros_node_->advertise<sensor_msgs::CompressedImage>(
              "/oak_ffc_4p/assemble_image/compressed", 1);
      ROS_INFO("Create publish topic /oak_ffc_4p/assemble_image/compressed\n");
    } else {
      this->assemble_image_publisher_ =
          this->ros_node_->advertise<sensor_msgs::Image>(
              "/oak_ffc_4p/assemble_image", 1);
    }
  }
  return 0;
}

void FFC4PDriver::StartVideoStream() {
  if (this->module_config_.ros_defined_freq) {
    this->ros_rate_ptr_ = std::make_unique<ros::Rate>(this->module_config_.fps);
    this->grab_thread_ = std::thread(&FFC4PDriver::RosGrabImgThread, this);
  } else {
    printf("Use std thread\n");
    this->grab_thread_ = std::thread(&FFC4PDriver::StdGrabImgThread, this);
  }
  ROS_INFO("Start streaming\n");
  return;
}

void FFC4PDriver::RosGrabImgThread() {
  while (this->ros_node_->ok() && this->is_run_) {
    GrabImg();
    this->ros_rate_ptr_->sleep();
  }
  ROS_INFO("Stop grab tread\n");
}

void FFC4PDriver::StdGrabImgThread() {
  while (this->is_run_) {
    GrabImg();
    usleep(SECOND / (2.0f * this->module_config_.fps));
  }
  ROS_INFO("Stop grab tread\n");
}

void FFC4PDriver::GrabImg() {
  static cv_bridge::CvImage cv_img, assemble_cv_img;
  static std_msgs::Int32 expose_time_msg;
  static cv::Mat assemble_cv_mat = cv::Mat::zeros(720, 5120, CV_8UC3);
  static auto const msgGrp = this->device_->getOutputQueue("msgOut", 4, false);

  auto msg_data = msgGrp->get<dai::MessageGroup>();
  if (msg_data == nullptr) {
    return;
  }
  for (const auto& camera : CameraList) {
    auto packet = msg_data->get<dai::ImgFrame>(camera.stream_name);
    if (packet) {
      image_pub_node_[camera.stream_name].image = packet->getCvFrame();
      // resize if need, here for best latency, we don't resize
      // cv::resize(image_pub_node_[camera.stream_name].image,
      //            image_pub_node_[camera.stream_name].image,
      //            cv::Size(1280, 720));
      image_pub_node_[camera.stream_name].cap_time_stamp =
          packet->getTimestamp();
      // image_pub_node_[camera.stream_name]->frame_counter++;
    } else {
      ROS_WARN("Get %s frame failed\n", camera.stream_name.c_str());
      return;
    }
  }
  auto host_ros_now_time = ros::Time::now();
  assemble_cv_img.header.stamp = host_ros_now_time;
  assemble_cv_img.header.frame_id = "depth ai";
  assemble_cv_img.encoding = "bgr8";
  assemble_cv_img.image = assemble_cv_mat;

  cv_img.header.stamp = host_ros_now_time;
  cv_img.header.frame_id = "depth ai";
  cv_img.encoding = "bgr8";

  expose_time_msg.data = this->module_config_.expose_time_us;

  auto host_time_now = dai::Clock::now();
  int colow_position = 0;
  if (this->module_config_.enable_upside_down) {
    for (auto& image_node : this->image_pub_node_) {
      cv::flip(image_node.second.image, image_node.second.image, -1);
    }
  }

  if (this->module_config_.sharpness_calibration_mode) {
    // for (auto& image_node : this->image_pub_node_) {
    //   cv_img.image = image_node.second.image;
    //   image_node.second.ros_publisher_ptr->publish(
    //       cv_img.toCompressedImageMsg());
    // }
  } else {
    for (auto& image_node : this->image_pub_node_) {
      image_node.second.image.copyTo(
          assemble_cv_img.image(cv::Rect(colow_position, 0, 1280, 720)));
      colow_position += IMAGE_WIDTH;
    }
    if (this->module_config_.compressed_mode) {
      assemble_image_publisher_.publish(assemble_cv_img.toCompressedImageMsg());
    } else {
      assemble_image_publisher_.publish(assemble_cv_img.toImageMsg());
    }
    this->expose_time_publisher_.publish(expose_time_msg);
  }
  if (this->module_config_.sharpness_calibration_mode) {
    for (auto& image_node : image_pub_node_) {
      this->ShowImg(image_node.second, host_time_now);
    }
  }
}

// TODO fps counter
void FFC4PDriver::ShowImg(
    ImagePubNode& image_node,
    std::chrono::_V2::steady_clock::time_point& time_now) {
  if (image_node.image.empty()) {
    return;
  } else {
    double clearness = Clearness(image_node.image);
    uint32_t latency_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              time_now - image_node.cap_time_stamp)
                              .count();
    std::stringstream info;
    info << image_node.topic << "clearness: = " << clearness
         << "    image_delay ms:=" << (latency_us / 1000);
    cv::putText(image_node.image, info.str(), cv::Point(10, 30),
                cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 0));
    cv::imshow(image_node.topic, image_node.image);
    cv::waitKey(1);
  }
  return;
}

double Clearness(cv::Mat& img) {
  // Clearness for focus
  if (img.empty()) {
    // printf("img is empty");
    return 0.0f;
  } else {
    cv::Mat gray, imgSobel;
    cv::Rect2d roi(img.cols / 3, img.rows / 3, img.cols / 3, img.rows / 3);
    cv::rectangle(img, roi, cv::Scalar(255, 0, 0), 1);
    cv::cvtColor(img(roi), gray, cv::COLOR_BGR2GRAY);
    cv::Sobel(gray, imgSobel, CV_16U, 1, 1);
    return cv::mean(imgSobel)[0];
  }
}

ros::Time convertToRosTime(
    const std::chrono::time_point<std::chrono::steady_clock,
                                  std::chrono::steady_clock::duration>&
        time_point) {
  // 获取从程序启动开始的时间点,单位为秒
  std::chrono::duration<double> duration = time_point.time_since_epoch();
  double seconds = duration.count();
  // 获取 ROS 启动时的时间
  ros::Time rosStartTime = ros::Time::now();
  // 计算当前时间
  ros::Time currentTime = rosStartTime + ros::Duration(seconds);

  return currentTime;
};

}  // namespace OAKCAM