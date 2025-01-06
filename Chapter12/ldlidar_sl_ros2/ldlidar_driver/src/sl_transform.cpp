/**
 * @file transform.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  Ranging center conversion with left and right hand system changes App
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

#include "sl_transform.h"

namespace ldlidar {

/*!
        \brief     transfer the origin to the center of lidar circle
        \param[in]
          \arg  version  lidar version, different versions have different
   parameter settings \arg  data     lidar raw data \arg  to_right_hand_  a flag
   whether convert to right-hand coordinate system

        \param[out]  data
        \retval      Data after coordinate conversion
*/
SlTransform::SlTransform(LDType version, bool to_right_hand) {
  switch (version) {
    case LDType::LD_14:
    case LDType::LD_14P_2300HZ:
    case LDType::LD_14P_4000HZ:
      offset_x_ = 5.9;
      offset_y_ = -18.975571;
      break;
    default:
      offset_x_ = 5.9;
      offset_y_ = -20.14;
      break;
  }
  to_right_hand_ = to_right_hand;
  version_ = version;
}

Points2D SlTransform::Transform(const Points2D &data) {
  Points2D tmp2;
  static double last_shift_delta = 0;
  for (auto n : data) {
    // transfer the origin to the center of lidar circle
    // The default direction of radar rotation is clockwise
    // transfer to the right-hand coordinate system
    double angle;
    if (n.distance > 0) {
      double x = n.distance + offset_x_;
      double y = n.distance * 0.11923 + offset_y_;
      double shift = atan(y / x) * 180.f / 3.14159;
      // Choose whether to use the right-hand system according to the flag
      if (to_right_hand_) {
        float right_hand = (360.f - n.angle);
        angle = right_hand + shift;
      } else {
        angle = n.angle - shift;
      }
      last_shift_delta = shift;
    } else {
      if (to_right_hand_) {
        float right_hand = (360.f - n.angle);
        angle = right_hand + last_shift_delta;
      } else {
        angle = n.angle - last_shift_delta;
      }
    }
    
    if (angle > 360) {
      angle -= 360;
    }
    if (angle < 0) {
      angle += 360;
    }
    
    switch (version_) {
      case LDType::LD_14:
      case LDType::LD_14P_2300HZ:
      case LDType::LD_14P_4000HZ:
        if (n.distance == 0) {
          tmp2.push_back(PointData(angle, n.distance, 0, n.stamp));
        } else {
          tmp2.push_back(PointData(angle, n.distance, n.intensity, n.stamp));
        }
        break;
      default:
        break;
    }
  }

  return tmp2;
}

SlTransform::~SlTransform() {}

} // namespace ldlidar

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/