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

#ifndef NDT_2D__NDT_MODEL_HPP_
#define NDT_2D__NDT_MODEL_HPP_

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <dt_ndt_mcl/point.hpp>
#include <dt_ndt_mcl/pose_2d.hpp>
#include <dt_ndt_mcl/scan.hpp>
#include <memory>
#include <vector>

namespace ndt_2d {

struct Cell {
  Cell();

  /** @brief Add a point to this cell */
  void addPoint(const Eigen::Vector2d& p);

  /** @brief Compute the cell values */
  void compute();

  /** @brief Score a point */
  double score(const Eigen::Vector2d& p);

  // Are mean/cov valid;
  bool valid;

  // Technically this should be size_t - but for Eigen math, needs to be double
  double n;
  Eigen::Vector2d mean;
  Eigen::Matrix2d covariance;
  Eigen::Matrix2d correlation;
  Eigen::Matrix2d information;
};

class NDT {
 public:
  /**
   * @brief Create an instance of an NDT distribution.
   * @param cell_size Size of NDT cells in meters.
   * @param size_x Size of NDT in meters.
   * @param size_y Size of NDT in meters.
   * @param origin_x Coordinate of lower left corner in meters.
   * @param origin_y Coordinate of lower left corner in meters.
   */
  NDT(double cell_size, double size_x, double size_y, double origin_x,
      double origin_y);

  virtual ~NDT();

  /**
   * @brief Add a scan to the NDT.
   * @param scan The points from laser scanner to be added.
   */
  void addScan(const ScanPtr& scan);

  /**
   * @brief Add a point to the NDT.
   * @param x The x coordinate of the point in map frame.
   * @param y The y coordinate of the point in map frame.
   */
  void addPoint(double x, double y);

  /**
   * @brief Compute NDT cell values - this must be called after any
   *        scans are added before you can query the cells.
   */
  void compute();

  /**
   * @brief Query the NDT for a given point.
   * @param point The point to score.
   * @returns The probability of the point.
   */
  double likelihood(const Eigen::Vector2d& point);

  /**
   * @brief Query the NDT for a given point.
   * @param point The point to score.
   * @returns The probability of the point.
   */
  double likelihood(const Eigen::Vector3d& point);

  /**
   * @brief Query the NDT.
   * @param points The vector of points to score.
   * @returns The probability of the points.
   */
  double likelihood(const std::vector<Point>& points);

  /**
   * @brief Query the NDT.
   * @param scan The scan to score. Note that scan->pose WILL be used.
   * @returns The probability of the scan.
   */
  double likelihood(const ScanPtr& scan);

 private:
  /**
   * @brief Get the index of a cell within cells_
   * @param x The x coordinate (in meters).
   * @param y The y coordinate (in meters).
   */
  int getIndex(double x, double y);

  double cell_size_;
  size_t size_x_, size_y_;
  double origin_x_, origin_y_;
  std::vector<Cell> cells_;
};

}  // namespace ndt_2d

#endif  // NDT_2D__NDT_MODEL_HPP_
