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


#include "camera.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/fill_image.h"

using namespace sensor_msgs;

namespace nv {

  void SensorCamera::initialize(dwContextHandle_t context, dwSALHandle_t hal)
  {
    m_sdk = context;
    m_hal = hal;

    m_cameraRun = false;
  }

  bool SensorCamera::start(dwSensorParams paramsClient)
  {
    dwStatus status;
    //------------------------------------------------------------------------------
    // initializes cameras
    // -----------------------------------------
    {
      status = dwSAL_createSensor(&m_camera[0], paramsClient, m_hal);
      if(status != DW_SUCCESS) {
        ROS_ERROR("Cannot create sensor %s with %s. Error: %s", paramsClient.protocol, paramsClient.parameters, dwGetStatusName(status));

        return false; 
      }
    }


    dwImageProperties imageProperties{};
    status = dwSensorCamera_getImageProperties(&imageProperties, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, m_camera[0]);
    if(status != DW_SUCCESS) {
      ROS_ERROR("Error");

      dwSAL_releaseSensor(m_camera[0]);

      return false;
    }
    imageProperties.format = DW_IMAGE_FORMAT_RGBA_UINT8;

    // create an image to hold the conversion from native to rgba, fit for streaming to gl
    status = dwImage_create(&m_rgbaFrame[0], imageProperties, m_sdk);
    if(status != DW_SUCCESS) {
      ROS_ERROR("Error");

      dwSAL_releaseSensor(m_camera[0]);

      return false;
    }

    // setup streamer for frame grabbing
    status = dwImageStreamer_initialize(&m_streamerNvmediaToCpuProcessed[0], &imageProperties, DW_IMAGE_CPU, m_sdk);
    if(status != DW_SUCCESS) {
      ROS_ERROR("Error");

      dwImage_destroy(m_rgbaFrame[0]);
      dwSAL_releaseSensor(m_camera[0]);

      return false;
    }

    status = dwSensor_start(m_camera[0]);
    if(status != DW_SUCCESS) {
      ROS_ERROR("Cannot start camera. Error: %s", dwGetStatusName(status));

      dwSAL_releaseSensor(m_camera[0]);

      return false;
    }

    m_cameraThread   = std::thread(&SensorCamera::run_camera, this);
    m_cameraRun = true;

    return true;
  }

  bool SensorCamera::stop()
  {
    if(!m_cameraRun) {
      ROS_WARN("CAMERA sensor not running");
      return false;
    }

    m_cameraRun = false;

    if(m_cameraThread.joinable())
      m_cameraThread.join();

    if (m_streamerNvmediaToCpuProcessed[0]) {
      dwImageStreamer_release(m_streamerNvmediaToCpuProcessed[0]);
    }

    if (m_rgbaFrame[0]) {
      dwImage_destroy(m_rgbaFrame[0]);
    } 

    if (m_camera[0]) {
      dwSAL_releaseSensor(m_camera[0]);
    }

    return true;
  }

  void SensorCamera::run_camera()
  {
    while(m_cameraRun) {
      dwCameraFrameHandle_t frame;
      dwStatus status = dwSensorCamera_readFrameNew(&frame, 33333, m_camera[0]);
      if (status == DW_END_OF_STREAM) {
        ROS_WARN("camera sensor end of stream reached.");
        break;
      } else if (status == DW_TIME_OUT) {
        ROS_WARN("camera sensor readFrame timed-out.");
        continue;
      } else if (status == DW_NOT_READY) {
        ROS_WARN("camera sensor not ready.");
        continue;
      } 

      if (status != DW_SUCCESS) {
        ROS_ERROR("camera sensor readFrame failed. Error: %s", dwGetStatusName(status));
        break;
      } else {
        ROS_INFO("camera sensor readFrame success.");

        dwImageHandle_t img;
        dwCameraOutputType outputType = DW_CAMERA_OUTPUT_NATIVE_PROCESSED;
        status = dwSensorCamera_getImage(&img, outputType, frame);
        if(status != DW_SUCCESS) {
          ROS_ERROR("Error");
          break;
        }

        // convert native (yuv420 planar nvmedia) to rgba nvmedia
        status = dwImage_copyConvert(m_rgbaFrame[0], img, m_sdk);
        if(status != DW_SUCCESS) {
          ROS_ERROR("Error");
          break;
        }

        // stream that image to the CPU domain
        status = dwImageStreamer_producerSend(m_rgbaFrame[0], m_streamerNvmediaToCpuProcessed[0]);
        if(status != DW_SUCCESS) {
          ROS_ERROR("Error");
          break;
        }

        // receive the streamed image as a handle
        dwImageHandle_t cpuFrame;
        status = dwImageStreamer_consumerReceive(&cpuFrame, 33000, m_streamerNvmediaToCpuProcessed[0]);
        if(status != DW_SUCCESS) {
          ROS_ERROR("Error");
          break;
        }

        dwImageProperties prop;
        status = dwImage_getProperties(&prop, cpuFrame);
        if (status != DW_SUCCESS) {
          ROS_ERROR("dwImage_getProperties() failed. Error: %s", dwGetStatusName(status));
          break;
        }

        dwImageCPU *imgCPU;
        status = dwImage_getCPU(&imgCPU, cpuFrame);
        if (status != DW_SUCCESS) {
          ROS_ERROR("dwImage_getCPU() failed. Error: %s", dwGetStatusName(status));
          break;
        }

        unsigned int pair_id = 0;
        ImagePtr image(new Image);

        dwTime_t timestamp;
        dwImage_getTimestamp(&timestamp, img);
        image->header.stamp.sec = (timestamp / 1000000L);
        image->header.stamp.nsec = (timestamp % 1000000L) * 1000;
        ROS_DEBUG("timestamp:  %u.%u", image->header.stamp.sec, image->header.stamp.nsec);

        image->header.seq = pair_id;
        pair_id++;

        image->header.frame_id = "camera";

        fillImage(*image, sensor_msgs::image_encodings::RGBA8, prop.height, prop.width, 4 * prop.width, imgCPU->data[0]);

        dwImageStreamer_consumerReturn(&cpuFrame, m_streamerNvmediaToCpuProcessed[0]);
        dwImageStreamer_producerReturn(nullptr, 33000, m_streamerNvmediaToCpuProcessed[0]);
        dwSensorCamera_returnFrame(&frame);

        m_cameraPub.publish(image);
      }
    }
  }

} //namespace nv
