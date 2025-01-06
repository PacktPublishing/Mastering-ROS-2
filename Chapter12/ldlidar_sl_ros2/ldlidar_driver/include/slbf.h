/**
 * @file slbf.h
 * @author LDRobot (support@ldrobot.com)
 * @brief   LiDAR near-range filtering algorithm
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
#ifndef __SLBF_H_
#define __SLBF_H_

#include <math.h>

#include <algorithm>

#include "ldlidar_datatype.h"

namespace ldlidar {

class Slbf {
private:
  const int kConfidenceHigh = 200;
  const int kConfidenceMiddle = 150;
  const int kConfidenceLow = 92;
  const int kScanFre = 2300;  // Default scanning frequency,
                              // which can be changed according to radar protocol
  double curr_speed_;
  bool enable_strict_policy_;  // whether strict filtering is enabled within 300
                               // mm, the effective value may be lost, and the
                               // time sequence of recharging needs to be
                               // disabled
  Slbf() = delete;
  Slbf(const Slbf &) = delete;
  Slbf &operator=(const Slbf &) = delete;

public:
  Slbf(int speed, bool strict_policy = true);
  Points2D NearFilter(const Points2D &tmp) const;
  void EnableStrictPolicy(bool enable);
  ~Slbf();
};

} // namespace ldlidar

#endif  // __SLBF_H_
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/