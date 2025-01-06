/**
 * @file slbf.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  LiDAR near-range filtering algorithm
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
#include "slbf.h"

namespace ldlidar {

/*!
        \brief      Set current speed
        \param[in]
          \arg  speed           Current lidar speed
          \arg  strict_policy   The flag to enable very strict filtering
        \param[out] none
        \retval     none
*/
Slbf::Slbf(int speed, bool strict_policy) {
  curr_speed_ = speed;
  enable_strict_policy_ = strict_policy;
}

Slbf::~Slbf() {}

/*!
        \brief        Filter within 1m to filter out unreasonable data points
        \param[in]
          \arg data   A circle of lidar data packed
        \param[out]   none
        \retval       Standard data
*/
Points2D Slbf::NearFilter(const Points2D &data) const {
  Points2D normal, pending, item;
  std::vector<Points2D> group;
  int sunshine_amount = 0;
  int dis_limit = 0;

  for (auto n : data) {
    if (n.distance < 20000) {
      pending.push_back(n);
    } else {
      normal.push_back(n);
    }
  }

  if (data.empty()) return normal;

  double angle_delta_up_limit = curr_speed_ / kScanFre * 1.5;
  double angle_delta_down_limit = curr_speed_ / kScanFre - 0.18;
  std::sort(pending.begin(), pending.end(), [](PointData a, PointData b) { return a.angle < b.angle; });

  PointData last(-10, 0, 0);

  for (auto n : pending) {
    dis_limit = 50;
    if (n.distance > 1000) dis_limit = n.distance / 20;
    if (fabs(n.angle - last.angle) > angle_delta_up_limit ||
        abs(n.distance - last.distance) > dis_limit) {
      if (item.empty() == false) {
        group.push_back(item);
        item.clear();
      }
    }
    item.push_back(n);
    last = n;
  }

  if (item.empty() == false) group.push_back(item);

  if (group.empty()) return normal;

  auto first_item = group.front().front();
  auto last_item = group.back().back();
  dis_limit = (first_item.distance + last_item.distance) / 2 / 20;
  if (dis_limit < 50) dis_limit = 50;
  if (fabs(first_item.angle + 360.f - last_item.angle) < angle_delta_up_limit &&
      abs(first_item.distance - last_item.distance) < dis_limit) {
    if (group.size() > 1) {
      group.front().insert(group.front().begin(), group.back().begin(), group.back().end());
      group.erase(group.end() - 1);
    }
  }

  for (auto n : group) {
    if (n.size() == 0) continue;

    if (n.size() > 35) {
      normal.insert(normal.end(), n.begin(), n.end());
      continue;
    }

    for (auto m : n) {
      int flag = m.intensity & 0x01;
      sunshine_amount += (flag == 1);
    }

    double sunshine_rate = (double)sunshine_amount / (double)n.size();

    double confidence_avg = 0;
    double dis_avg = 0;
    for (auto m : n) {
      confidence_avg += m.intensity;
      dis_avg += m.distance;
    }
    confidence_avg /= n.size();
    dis_avg /= n.size();

    if (dis_avg > 1000 && sunshine_rate < 0.2 &&
        confidence_avg > kConfidenceHigh && n.size() > 2) {
      normal.insert(normal.end(), n.begin(), n.end());
      continue;
    }

    if (sunshine_rate > 0.5 && confidence_avg < kConfidenceLow) {
      for (auto& point : n) {
        point.distance = 0;
        point.intensity = 0;
      }
      normal.insert(normal.end(), n.begin(), n.end());
      continue;
    }

    if (enable_strict_policy_) {
      if (dis_avg > 8100 && confidence_avg < kConfidenceLow && n.size() < 1) {
        for (auto& point : n) {
          point.distance = 0;
          point.intensity = 0;
        }
        normal.insert(normal.end(), n.begin(), n.end());
        continue;
      } else if (dis_avg > 6000 && confidence_avg < kConfidenceLow && n.size() < 2) {
        for (auto& point : n) {
          point.distance = 0;
          point.intensity = 0;
        }
        normal.insert(normal.end(), n.begin(), n.end());
        continue;
      } else if (dis_avg > 4000 && confidence_avg < kConfidenceHigh && n.size() < 2) {
        for (auto& point : n) {
          point.distance = 0;
          point.intensity = 0;
        }
        normal.insert(normal.end(), n.begin(), n.end());
        continue;
      } else if (dis_avg > 300 /*&& confidence_avg < kConfidenceHigh */ && n.size() < 2) {
        for (auto& point : n) {
          point.distance = 0;
          point.intensity = 0;
        }
        normal.insert(normal.end(), n.begin(), n.end());
        continue;
      }

      if (dis_avg < 300 && confidence_avg < kConfidenceHigh && n.size() < 3) {
        for (auto& point : n) {
          point.distance = 0;
          point.intensity = 0;
        }
        normal.insert(normal.end(), n.begin(), n.end());
        continue;
      }

      if (dis_avg < 300 && sunshine_rate > 0.5 &&
          confidence_avg < kConfidenceMiddle && n.size() < 5) {
        for (auto& point : n) {
          point.distance = 0;
          point.intensity = 0;
        }
        normal.insert(normal.end(), n.begin(), n.end());
        continue;
      }

      if (dis_avg < 200 && sunshine_rate > 0.4 &&
          confidence_avg < kConfidenceMiddle && n.size() < 6) {
        for (auto& point : n) {
          point.distance = 0;
          point.intensity = 0;
        }
        normal.insert(normal.end(), n.begin(), n.end());
        continue;
      }

      if (dis_avg < 500 && sunshine_rate > 0.9 && n.size() < 3) {
        for (auto& point : n) {
          point.distance = 0;
          point.intensity = 0;
        }
        normal.insert(normal.end(), n.begin(), n.end());
        continue;
      }

      if (dis_avg < 200 && confidence_avg < kConfidenceMiddle && n.size() < 3) {
        for (auto& point : n) {
          point.distance = 0;
          point.intensity = 0;
        }
        normal.insert(normal.end(), n.begin(), n.end());
        continue;
      }
    }

    double diff_avg = 0;
    int n_size = (int)n.size();
    for (int i = 1; i < n_size; i++) {
      if (n[i].angle > n[i - 1].angle) {
        diff_avg += fabs(n[i].angle - n[i - 1].angle);
      } else {
        diff_avg += fabs(n[i].angle + 360.0 - n[i - 1].angle);
      }
    }
    diff_avg /= (double)(n.size() - 1);

    if (diff_avg > angle_delta_down_limit) {
      normal.insert(normal.end(), n.begin(), n.end());
    } else {
      for (auto& point : n) {
          point.distance = 0;
          point.intensity = 0;
      }
      normal.insert(normal.end(), n.begin(), n.end());
    }
  }

  return normal;
}

/*!
        \brief           Enable strong filtering
        \param[in]
          \arg  enable : true ，false
        \param[out] none
        \retval
*/
void Slbf::EnableStrictPolicy(bool enable) { enable_strict_policy_ = enable; }

} // namespace ldlidar

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
