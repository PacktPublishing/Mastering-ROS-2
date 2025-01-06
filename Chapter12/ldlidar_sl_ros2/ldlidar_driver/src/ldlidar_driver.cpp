/**
 * @file ldlidar_node.cpp
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
#include "ldlidar_driver.h"

namespace ldlidar {

bool LDLidarDriver::is_ok_ = false;

LDLidarDriver::LDLidarDriver() : sdk_pack_version_("3.0.5"),
  is_start_flag_(false),
  comm_pkg_(new LiPkg()),
  comm_serial_(new SerialInterfaceLinux()){
  
  last_pubdata_times_ = std::chrono::steady_clock::now();
}

LDLidarDriver::~LDLidarDriver() {
  if (comm_pkg_ != nullptr) {
    delete comm_pkg_;
  }

  if (comm_serial_ != nullptr) {
    delete comm_serial_;
  }
}

std::string LDLidarDriver::GetLidarSdkVersionNumber(void) {
  return sdk_pack_version_;
}

bool LDLidarDriver::Start(LDType product_name, std::string serial_port_name, uint32_t serial_baudrate) {
  if (is_start_flag_) {
    return true;
  }

  if (serial_port_name.empty()) {
    LD_LOG_ERROR("input <serial_port_name> is empty.","");
    return false;
  }

  if (register_get_timestamp_handle_ == nullptr) {
    LD_LOG_ERROR("get timestamp fuctional is not register.","");
    return false;
  }

  comm_pkg_->ClearDataProcessStatus();
  comm_pkg_->RegisterTimestampGetFunctional(register_get_timestamp_handle_);
  comm_pkg_->SetProductType(product_name);

  comm_serial_->SetReadCallback(std::bind(&LiPkg::CommReadCallBack, comm_pkg_, std::placeholders::_1, std::placeholders::_2));

  if (!(comm_serial_->Open(serial_port_name, serial_baudrate))) {
    LD_LOG_ERROR("serial is not open:%s", serial_port_name.c_str());
    return false;
  }

  is_start_flag_ = true;

  SetIsOkStatus(true);

  return true;
}

bool LDLidarDriver::Stop(void)  {
  if (!is_start_flag_) {
    return true;
  }

  SetIsOkStatus(false);

  comm_serial_->Close();
  
  is_start_flag_ = false;
  
  return true;
}

void LDLidarDriver::EnableFilterAlgorithnmProcess(bool is_enable) {
  comm_pkg_->SetNoiseFilter(is_enable);
}

bool LDLidarDriver::WaitLidarCommConnect(int64_t timeout) {
  auto last_time = std::chrono::steady_clock::now();

  bool is_recvflag = false;
  do {
    if (comm_pkg_->GetLidarPowerOnCommStatus()) {
      is_recvflag = true;
    }
    usleep(1000);
  } while (!is_recvflag && (std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - last_time).count() < timeout));

  if (is_recvflag) {
    last_pubdata_times_ = std::chrono::steady_clock::now();
    SetIsOkStatus(true);
    return true;
  } else {
    SetIsOkStatus(false);
    return false;
  }
}

LidarStatus LDLidarDriver::GetLaserScanData(Points2D& dst, int64_t timeout) {
  if (!is_start_flag_) {
    return LidarStatus::STOP;
  }

  LidarStatus status = comm_pkg_->GetLidarStatus();
  if (LidarStatus::NORMAL == status) {
    if (comm_pkg_->GetLaserScanData(dst)) {
      last_pubdata_times_ = std::chrono::steady_clock::now(); 
      return LidarStatus::NORMAL;
    }
    
    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_pubdata_times_).count() > timeout) {
      return LidarStatus::DATA_TIME_OUT;
    } else {
      return LidarStatus::DATA_WAIT;
    }
  } else {
    last_pubdata_times_ = std::chrono::steady_clock::now(); 
    return status;
  }
}

bool  LDLidarDriver::GetLidarScanFreq(double& spin_hz) {
  if (!is_start_flag_) {
    return false;
  }
  spin_hz = comm_pkg_->GetSpeed();
  return true;
}

void LDLidarDriver::RegisterGetTimestampFunctional(std::function<uint64_t(void)> get_timestamp_handle) {
  register_get_timestamp_handle_ = get_timestamp_handle;
}

uint8_t LDLidarDriver::GetLidarErrorCode(void) {
  if (!is_start_flag_) {
    return LIDAR_NO_ERROR;
  }
  
  uint8_t errcode = comm_pkg_->GetLidarErrorCode();
  return errcode;
}

} // namespace ldlidar
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/