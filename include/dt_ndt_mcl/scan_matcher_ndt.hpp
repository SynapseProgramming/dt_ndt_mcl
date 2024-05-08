/*
 * Copyright (c) 2023 Michael Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the opyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NDT_2D__SCAN_MATCHER_NDT_HPP_
#define NDT_2D__SCAN_MATCHER_NDT_HPP_

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <dt_ndt_mcl/ndt_model.hpp>
#include <dt_ndt_mcl/scan_matcher.hpp>
#include <memory>
#include <string>
#include <vector>

namespace ndt_2d {

class ScanMatcherNDT : public ScanMatcher {
 public:
  virtual ~ScanMatcherNDT() = default;

  /**
   * @brief Initialize an NDT scan matcher instance.
   * @param name Name for ths scan matcher instance.
   * @param node Node instance to use for getting parameters.
   * @param range_max Maximum range of laser scanner.
   */
  void initialize(const std::string& name, ros::NodeHandle& node,
                  double range_max);

  /**
   * @brief Add scans to the internal NDT map.
   * @param begin Starting iterator of scans for NDT map building.
   * @param end Ending iterator of scans for NDT map building.
   */
  void addScans(const std::vector<ScanPtr>::const_iterator& begin,
                const std::vector<ScanPtr>::const_iterator& end);

  /**
   * @brief Match a scan against the internal NDT map.
   * @param scan Scan to match against internal NDT map.
   * @param pose The corrected pose that best matches scan to NDT map.
   * @param covariance Covariance matrix for the match.
   * @returns The score when scan is at corrected pose.
   */
  double matchScan(const ScanPtr& scan, Pose2d& pose,
                   Eigen::Matrix3d& covariance) const;

  /**
   * @brief Score a scan against the internal NDT map.
   * @param scan Scan to score against internal NDT map.
   */
  double scoreScan(const ScanPtr& scan) const;

  /**
   * @brief Score a set of points against the internal NDT map.
   * @param points Points to score against internal NDT map.
   * @param pose The pose of the points within the internal NDT map.
   */
  double scorePoints(const std::vector<Point>& points,
                     const Pose2d& pose) const;

  /**
   * @brief Reset the internal NDT map, removing all scans.
   */
  void reset();

  /**
   * @brief Add a map to the NDT map.
   * @param map The map to add to the NDT map.
   */
  void addMap(const nav_msgs::OccupancyGrid& map);

  void updateLocalMap(const ScanPtr& scan);

 protected:
  // Resolution of the NDT map
  double resolution_;

  // Search parameters
  double angular_res_, angular_size_;
  double linear_res_, linear_size_;
  size_t laser_max_beams_;

  // Max range of laser scanner
  double range_max_;

  std::unique_ptr<NDT> ndt_;

  std::unique_ptr<NDT> m_local_map;
};

}  // namespace ndt_2d

#endif  // NDT_2D__SCAN_MATCHER_NDT_HPP_
