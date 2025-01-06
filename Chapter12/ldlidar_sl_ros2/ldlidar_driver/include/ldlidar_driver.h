/**
 * @file ldlidar_driver.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  ldlidar processing App
 *         This code is only applicable to LDROBOT LiDAR LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-05-12
 *
 * @copyright Copyright (c) 2022  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef __LDLIDAR_DRIVER_NODE_H__
#define __LDLIDAR_DRIVER_NODE_H__

#include <chrono>

#include "lipkg.h"
#include "serial_interface_linux.h"
#include "log_module.h"

namespace ldlidar {

class LDLidarDriver {
public:
  LDLidarDriver();

  ~LDLidarDriver();

  std::string GetLidarSdkVersionNumber(void);
  
  /**
   * @brief start lidar device handle node
   * @param product_name
   *        ldlidar product type: ldlidar::LDType, value:
   *          - ldlidar::LDType::NOVER
   *          - ldlidar::LDType::LD_14
   *            ....
   *        - else definition in "ldlidar_driver/include/ldlidar_datatype.h"
   * @param serial_port_name
   *        serial device system path, eg: "/dev/ttyUSB0"
   * @param serial_baudrate
   *       serial communication baudrate value(> 0), unit is bit/s.
   * @retval value is true, start is success;
   *   value is false, start is failed.
  */
  bool Start(LDType product_name, std::string serial_port_name, uint32_t serial_baudrate = 115200);
  
  bool Stop(void);
  
  void EnableFilterAlgorithnmProcess(bool is_enable);

  /**
   * @brief Whether the connection of the communication channel is normal after the lidar is powered on
   * @param[in]
   * *@param timeout:  Wait timeout, in milliseconds
   * @retval if times >= 1000, return false, communication connection is fail;
   *   if "times < 1000", return ture, communication connection is successful.
  */
  bool WaitLidarCommConnect(int64_t timeout = 1000);

  /**
   * @brief get lidar laser scan point cloud data
   * @param [output]
   * *@param dst: type is ldlidar::Point2D, value is laser scan point cloud data
   * @param [in]
   * *@param timeout: Wait timeout, in milliseconds
   * @retval value is ldlidar::LidarStatus Enum Type, definition in "ldlidar_driver/include/ldlidar_datatype.h", value is:
   *  ldlidar::LidarStatus::NORMAL   // 雷达正常
   *  lldlidar::LidarStatus::ERROR    // 雷达异常错误
   *  ....
  */
  LidarStatus GetLaserScanData(Points2D& dst, int64_t timeout = 1000);
  
  bool GetLidarScanFreq(double& spin_hz);  

  /**
   * @brief register get timestamp handle functional.
   * @param [input]
   * *@param get_timestamp_handle: type is `uint64_t (*)(void)`, value is pointer get timestamp fuction.
   * @retval none
  */
  void RegisterGetTimestampFunctional(std::function<uint64_t(void)> get_timestamp_handle);

  /**
   * @brief When the lidar is in an error state, get the error code fed back by the lidar
   * @param none
   * @retval errcode
  */
  uint8_t GetLidarErrorCode(void);

  static bool IsOk() { return is_ok_; }

  static void SetIsOkStatus(bool status) { is_ok_ = status;}

private:
  std::string sdk_pack_version_;
  static bool is_ok_;
  bool is_start_flag_;
  LiPkg* comm_pkg_;
  SerialInterfaceLinux* comm_serial_;
  std::function<uint64_t(void)> register_get_timestamp_handle_;
  std::chrono::_V2::steady_clock::time_point last_pubdata_times_;
};

} // namespace ldlidar

#endif // __LDLIDAR_DRIVER_NODE_H__
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/