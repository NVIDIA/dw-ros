/* Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 *
 * SPDX-License-Identifier: MIT 
 */

#include "nv_sensors/camera_start.h"
#include "nv_sensors/camera_stop.h"
#include "sensor_msgs/Image.h"
#include "camera.h"

#include "nvcommon.h"
#include "ros/ros.h"

//dw core
#include <dw/core/Context.h>
#include <dw/core/VersionCurrent.h>

using namespace nv;

SensorCamera cameraSensor;

/* Service callback funtions*/
bool camera_start(nv_sensors::camera_start::Request &req,
    nv_sensors::camera_start::Response &res)
{
  if(cameraSensor.isSensorsRunning()) {
    ROS_WARN("Service already running. camera sensor data being published on topic /cameraData");
    res.success = false;
    return false;
  }

  ROS_INFO("Service params called are as follows: %s %s", req.driver.c_str(), req.params.c_str());

  dwSensorParams params;
  params.parameters   = req.params.c_str();
  params.protocol     = req.driver.c_str();
  res.success = cameraSensor.start(params);

  return res.success;

}

bool camera_stop(nv_sensors::camera_stop::Request &req,
    nv_sensors::camera_stop::Response &res)
{
  if(!cameraSensor.isSensorsRunning()) {
    ROS_WARN("camera sensor is not running");
    res.success = false;
    return false;
  }

  res.success = cameraSensor.stop();

  return res.success;
}

/* Main function*/
int main(int argc, char **argv)
{
  /*ROS node initialization*/
  ROS_DEBUG("Nv sensors producer node initialization");
  ros::init(argc, argv, "nv_sensors_producer");
  ros::NodeHandle nh;

  dwContextHandle_t sdk       = DW_NULL_HANDLE;
  dwSALHandle_t     hal       = DW_NULL_HANDLE;
  /* instantiate Driveworks SDK context*/
  dwContextParameters sdkParams = {};
  dwStatus status = dwInitialize(&sdk, DW_VERSION, &sdkParams);
  if (status != DW_SUCCESS) {
    ROS_ERROR("Failed to init Driveworks SDK. Error: %s", dwGetStatusName(status));
    exit(NV_ERR);
  }

  /* create HAL module of the SDK */
  status = dwSAL_initialize(&hal, sdk);
  if (status != DW_SUCCESS) {
    ROS_ERROR("Failed to create HAL module of Driveworks SDK. Error: %s", dwGetStatusName(status));
    exit(NV_ERR);
  }

  cameraSensor.initialize(sdk, hal);

  /*Service callback functions*/
  ROS_INFO("Advertising Camera Start and Camera Stop Services.");
  ros::ServiceServer cameraService_start = nh.advertiseService("camera_start", camera_start);
  ros::ServiceServer cameraService_stop  = nh.advertiseService("camera_stop", camera_stop);

  /*Publisher*/
  ros::Publisher cameraPub   = nh.advertise<sensor_msgs::Image>("cameraData", 1);

  cameraSensor.setPublisher(&cameraPub);

  ros::spin();

  if(cameraSensor.isSensorsRunning()) {
    cameraSensor.stop();
  }

  // release used objects in correct order
  dwSAL_release(hal);
  dwRelease(sdk);

  return 0;
}
