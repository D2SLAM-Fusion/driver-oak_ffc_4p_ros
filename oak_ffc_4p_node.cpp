#pragma once
#include <fcntl.h>
#include <linux/videodev2.h>
#include <memory.h>
#include <ros/ros.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <list>
#include <vector>

#include "oak_ffc_4p.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "oak_ffc_4p_ros");
  auto cam_node = std::make_shared<ros::NodeHandle>("oak_ffc_4p_ros");
  OAKCAM::FFC4PDriver cam_driver(cam_node);
  int32_t ret = 0;
  ret = cam_driver.Init();
  if (ret != 0) {
    printf("init pipeline failed\n");
    return -1;
  }
  cam_driver.StartVideoStream();
  ros::spin();
  cam_driver.StopVideoStream();
  usleep(500);
  return 0;
}