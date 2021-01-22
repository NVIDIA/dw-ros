/*Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.

  NVIDIA CORPORATION and its licensors retain all intellectual property
  and proprietary rights in and to this software, related documentation
  and any modifications thereto.  Any use, reproduction, disclosure or
  distribution of this software and related documentation without an express
  license agreement from NVIDIA CORPORATION is strictly prohibited.

  SPDX-License-Identifier: MIT*/

#ifndef _NVCOMMON_H_
#define _NVCOMMON_H_

/**
 * @file
 * @brief <b> NVIDIA Robot Operating System : Common Enums, Macros, and Constants  </b>
 *
 * Contains common enums, macros, and constants for Nv related
 * functions.
 */

#include <string>


/**
 *  @namespace nv
 *  @brief A global namespace for Nv packages
 */
namespace nv
{
  /**
   * @defgroup nv_common_group Common Declarations
   *
   * Defines common values and error codes for Nv related
   * functions and NVIDIA<sup>&reg;</sup> CUDA<sup>&reg;</sup> processing
   * on the EGL CUDA IO pipeline.
   * @ingroup nv_group
   * @{
   */
  /**
   *  @brief Declares error codes for all
   *  Nv modules and packages.
   *
   */
  typedef enum _NvErrCode {

    /** Indicates the API was successfully executed. */
    NV_SUCCESS=0,

    /** Indicates dynamic memory allocation failed during
      API execution.
     */
    NV_ERR_NO_MEM=-1,

    /** Indicates an error occurred while executing one or more NvMedia API. */
    NV_ERR_NVMEDIA=-2,

    /** Indicates an error occurred due to a miscellaneous reason other than
      NV_ERR_NO_MEM and NV_ERR_NVMEDIA.
     */
    NV_ERR=-3,

    /** Indicates an error occurred due to bad parameters passed in the API. */
    NV_ERR_BAD_PARAMS=-4,

  } NvErr ;

  /** Default width of the display. */
#define DEFAULT_DISPLAY_WIDTH     1920

  /** Default height of the display. */
#define DEFAULT_DISPLAY_HEIGHT    1080

  /** Default horizontal offset. */
#define DEFAULT_WINDOW_OFFSET_X   0

  /** Default vertical offset. */
#define DEFAULT_WINDOW_OFFSET_Y   0

  /** Default display ID. */
#define DEFAULT_DISPLAY_ID      0

  /** Secondary display ID. */
#define SECONDARY_DISPLAY_ID        2

  /** Default window ID. */
#define DEFAULT_WINDOW_ID     0

  /** Sets the EGLStream mode to EGLDEVICE by default. */
#define DEFAULT_EGLSTREAM_MODE    1

  /** Default CUDA kernel ID to select
    rgba_to_grey CUDA processing kernel for EGL CUDA IO pipeline. */
#define CUDA_KERNEL_RGBA_TO_GREY    1

  /** CUDA kernel ID to select sobel CUDA processing kernel for EGL CUDA IO pipeline. */
#define CUDA_KERNEL_SOBEL           2

  /** CUDA kernel ID to select negative_rgba CUDA processing kernel for EGL CUDA IO pipeline. */
#define CUDA_KERNEL_NEGATIVE_RGBA   3

  /** Selects 0th port of group A on NVIDIA DRIVE<sup>&trade;</sup> AGX
    for CSI camera by default. */
#define DEFAULT_CSI_PORT_ID         0

  /** Time, in microseconds, for one frame at 30 FPS frame rate. */
#define ONE_FRAME_TIME_US_30_FPS  33334

  /** Maximum wait time in number of frames.
    @note The value of this macro is not perfectly tuned
    but is observed to be good for many use cases. This may likely
    cause some performance issue in unexplored use cases.
   */
#define MAX_WAIT_TIMEOUT_FRAMES   100

  /** Maximum wait time for a service to become available
    after the node is launched.
    @note The value of this macro is not perfectly
    tuned but observed to be good for many use cases. This may likely
    cause some performance issues in unexplored use cases.
   */
#define MAX_SERVICE_WAIT_TIMEOUT_MS 100

  /** Default value to enable yuv422p frame conversion to RGBA frame on capture from GMSL camera
    before passing to EGLStream for further processing.
   */
#define IS_RGBA_CONVERSION      1

  /** Constant string for initializing camera preview services. */
  const std::string InitCamPreviewService("nv_init_cam_preview");

  /** Constant string for closing camera preview services. */
  const std::string CloseCamPreviewService("nv_close_cam_preview");

  /** Constant string for starting camera capture services. */
  const std::string StartCameraCaptureService("camera_start");

  /** Constant string for closing camera capture services. */
  const std::string StopCameraCaptureService("camera_stop");

  /** Constant string for initializing CUDA processing services. */
  const std::string InitCudaProcessingService("nv_init_cuda_processing");

  /** Constant string for closing CUDA processing services. */
  const std::string CloseCudaProcessingService("nv_close_cuda_processing");

  /** Constant string for default output EGL socket_path value for the cuda_processor node. */
  const std::string SocketPathOutput("/tmp/nvmedia_egl_socket_out");

  /** Default group name for the camera capture node. */
  const std::string Default_GroupName("A");

  /** Default configuration file for OV10635 camera parameters. */
  const std::string Default_FileName("drive-px2-a.conf");

  /** @} */
}
#endif
