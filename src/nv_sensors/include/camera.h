/*Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.

  NVIDIA CORPORATION and its licensors retain all intellectual property
  and proprietary rights in and to this software, related documentation
  and any modifications thereto.  Any use, reproduction, disclosure or
  distribution of this software and related documentation without an express
  license agreement from NVIDIA CORPORATION is strictly prohibited.

  SPDX-License-Identifier: MIT*/

#ifndef _NV_SENSORS_CAMERA_H_
#define _NV_SENSORS_CAMERA_H_

#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/interop/streamer/ImageStreamer.h>
#include <dw/rig/Rig.h>

#include <ros/ros.h>

#include <thread>

/**
 * @file camera.h
 * @author Prasun Kumar
 * @date Feb 9, 2019
 * @version 0.0
 *
 * @brief Declarations of enums, structures and a class for camera sensor related
 * utility functions.
 */

/**
 *  @namespace nv
 *  @brief A global namespace for Nv packages
 */
namespace nv
{

  /**
   * @class SensorCamera
   * @brief A utility class for initialization of CAMERA sensor
   * @details SensorCamera is a class through which user can
   * perform initialization, start and stop of data acquisition
   * from camera sensor.
   */
  class SensorCamera {

    public:
      /**
       * @brief Initailization of SensorCamera class
       * @details This API is required to initailize the SensorCamera class
       * and provide the Driveworks handle and SAL handle.
       *
       * @param context Driveworks SDK handle
       * @param hal Driveworks SAL handle
       */
      void initialize(dwContextHandle_t context, dwSALHandle_t hal);

      /**
       * @brief Initailization of camera sensor
       * @details This API is required to initailize the camera sensor
       * and start data acquitsion
       *
       * @param params Driveworks Sensors params list
       *
       * @return true if execution successful
       *         false otherwise
       */
      bool start(dwSensorParams params);

      /**
       * @brief Release of camera sensor
       * @details This API is required to release the camera sensor
       * and stop data acquitsion
       *
       * @return true if execution successful
       *         false otherwise
       */
      bool stop();

      /**
       * @brief Setting of Publisher
       * @details This API is required to set ros::Publisher for the camera sensor
       *
       * @params pub Pointer to ros::Publisher
       *
       * @return true if execution successful
       *         false otherwise
       */
      void setPublisher(ros::Publisher* pub) {
        m_cameraPub = *pub;
      }

      /**
       * @brief Query of Sensor state
       * @details This API is required to quesy the camera sensor state
       *
       * @return true if dta acquisition from camera sensor is active
       *         false otherwise
       */
      bool isSensorsRunning() {
        return m_cameraRun;
      }

    private:
      void run_camera();

      dwContextHandle_t m_sdk       = DW_NULL_HANDLE;
      dwSALHandle_t     m_hal       = DW_NULL_HANDLE;
      dwSensorHandle_t  m_cameraSensor = DW_NULL_HANDLE;
      dwSensorHandle_t m_cameraMaster;
      dwSensorHandle_t m_camera[16];
      dwImageHandle_t m_rgbaFrame[16] = {DW_NULL_HANDLE};
      // Frame grab variables
      dwImageStreamerHandle_t m_streamerNvmediaToCpuProcessed[16] = {DW_NULL_HANDLE};

      bool m_cameraRun = false;

      std::thread m_cameraThread;

      ros::Publisher m_cameraPub;
  };

} // namespace nv

#endif // _NV_SENSORS_CAMERA_H_
