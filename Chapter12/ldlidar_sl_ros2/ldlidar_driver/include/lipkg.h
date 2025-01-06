/**
 * @file lipkg.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  LiDAR data protocol processing App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-09
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
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
#ifndef __LIPKG_H
#define __LIPKG_H

#include <string.h>

#include <chrono>
#include <mutex>
#include <functional>

#include "slbf.h"
#include "sl_transform.h"

namespace ldlidar {

enum {
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

class LiPkg {
public:
  LiPkg();

  ~LiPkg();

  void SetProductType(LDType typenumber);

  void SetNoiseFilter(bool is_enable);

  void RegisterTimestampGetFunctional(std::function<uint64_t(void)> timestamp_handle);

  void CommReadCallBack(const char *byte, size_t len);

  /**
   * @brief get lidar scan data
  */
  bool GetLaserScanData(Points2D& out); 

  /**
   * @brief get Lidar spin speed (Hz)
  */
  double GetSpeed(void);

  LidarStatus GetLidarStatus(void);

  uint8_t GetLidarErrorCode(void);

  bool GetLidarPowerOnCommStatus(void);

  void ClearDataProcessStatus(void) {
    is_frame_ready_ = false;
    is_poweron_comm_normal_ = false;
    lidarstatus_ = LidarStatus::NORMAL;
    lidarerrorcode_ = LIDAR_NO_ERROR;
    last_pkg_timestamp_ = 0;
  }

private:
  int lidar_measure_freq_;
  LDType typenumber_;
  LidarStatus lidarstatus_;
  uint8_t lidarerrorcode_;
  bool is_frame_ready_;
  bool is_noise_filter_;
  uint16_t timestamp_; 
  double speed_;
  std::function<uint64_t(void)> get_timestamp_;
  bool is_poweron_comm_normal_;
  uint8_t poweron_datapkg_count_;
  uint64_t last_pkg_timestamp_;

  LiDARFrameTypeDef datapkg_;
  Points2D lidar_frame_data_;
  Points2D frame_tmp_;
  std::mutex mutex_lock1_;
  std::mutex mutex_lock2_;

  void SetLidarStatus(LidarStatus status);

  void SetLidarErrorCode(uint8_t errorcode);

  bool AnalysisOne(uint8_t byte); // parse single packet

  bool Parse(const uint8_t *data, long len); 

  bool AssemblePacket(); // combine stantard data into data frames and calibrate

  bool IsFrameReady(void);  // get Lidar data frame ready flag

  void ResetFrameReady(void);  // reset frame ready flag

  void SetFrameReady(void);    // set frame ready flag

  void SetLaserScanData(Points2D& src);

  Points2D GetLaserScanData(void);

  // void AnalysisLidarIsBlocking(uint16_t lidar_speed_val);

  // void AnalysisLidarIsOcclusion(Points2D& lidar_data);
};

} // namespace ldlidar

#endif  // __LIPKG_H
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/