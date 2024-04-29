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

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <dt_ndt_mcl/conversions.hpp>
#include <dt_ndt_mcl/ndt_model.hpp>

namespace ndt_2d {

Cell::Cell()
    : valid(false),
      n(0),
      mean(Eigen::Vector2d::Zero()),
      covariance(Eigen::Matrix2d::Zero()),
      correlation(Eigen::Matrix2d::Zero()),
      information(Eigen::Matrix2d::Zero()) {}

void Cell::addPoint(const Eigen::Vector2d& point) {
  mean = (mean * n + point) / (n + 1);
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = i; j < 2; ++j) {
      correlation(i, j) =
          (correlation(i, j) * n + point(i) * point(j)) / (n + 1);
    }
  }

  n += 1;
  valid = false;
}

void Cell::compute() {
  // No need to update
  if (valid || n < 3) {
    return;
  }

  const double scale = n / (n - 1);
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = i; j < 2; ++j) {
      covariance(i, j) = (correlation(i, j) - (mean(i) * mean(j))) * scale;
      covariance(j, i) = covariance(i, j);
    }
  }

  // Limit the eigen values so that smaller is always at least 0.001 of larger
  Eigen::EigenSolver<Eigen::Matrix2d> solver(covariance);
  Eigen::Vector2d eigenvalues = solver.eigenvalues().real();
  double small = eigenvalues(0), large = eigenvalues(1);
  if (small > large) std::swap(small, large);
  if (small < 0.001 * large) {
    // Special case
    double determinant = (0.001 * large) * large;
    information(0, 0) = covariance(1, 1) / determinant;
    information(0, 1) = -covariance(1, 0) / determinant;
    information(1, 0) = -covariance(0, 1) / determinant;
    information(1, 1) = covariance(0, 0) / determinant;
  } else {
    information = covariance.inverse();
  }

  valid = true;
}

double Cell::score(const Eigen::Vector2d& point) {
  if (n < 5) {
    // Need at least five points for our mean/cov to be valid
    return 0.0;
  }

  const auto q = point - mean;
  const double exponent = -0.5 * q.transpose() * information * q;
  return std::exp(exponent);
}

NDT::NDT(double cell_size, double size_x, double size_y, double origin_x,
         double origin_y) {
  cell_size_ = cell_size;
  size_x_ = (size_x / cell_size_) + 1;
  size_y_ = (size_y / cell_size_) + 1;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  cells_.resize(size_x_ * size_y_);
}

NDT::~NDT() {}

void NDT::addScan(const ScanPtr& scan) {
  // Precompute transforms
  double cos_th = cos(scan->getPose().theta);
  double sin_th = sin(scan->getPose().theta);

  for (auto& point : scan->getPoints()) {
    // Transform the point by pose
    Eigen::Vector2d p(scan->getPose().x, scan->getPose().y);
    p(0) += point.x * cos_th - point.y * sin_th;
    p(1) += point.x * sin_th + point.y * cos_th;

    // Determine index in NDT grid, add if valid index
    int index = getIndex(p(0), p(1));
    if (index >= 0) {
      cells_[index].addPoint(p);
    }
  }
}

void NDT::compute() {
  for (auto& cell : cells_) {
    cell.compute();
  }
}

double NDT::likelihood(const Eigen::Vector2d& point) {
  int index = getIndex(point(0), point(1));
  if (index >= 0) {
    return cells_[index].score(point);
  }
  return 0.0;
}

double NDT::likelihood(const Eigen::Vector3d& point) {
  Eigen::Vector2d p(point(0), point(1));
  return likelihood(p);
}

double NDT::likelihood(const std::vector<Point>& points) {
  double score = 0.0;
  for (auto& point : points) {
    Eigen::Vector2d p(point.x, point.y);
    score += likelihood(p);
  }
  return score;
}

double NDT::likelihood(const ScanPtr& scan) {
  const Eigen::Isometry3d transform = toEigen(scan->getPose());

  double score = 0.0;
  for (auto& point : scan->getPoints()) {
    Eigen::Vector3d p(point.x, point.y, 1.0);
    p = transform * p;
    score += likelihood(p);
  }
  return score;
}

int NDT::getIndex(double x, double y) {
  if (x < origin_x_ || y < origin_y_) {
    return -1;
  }

  unsigned int grid_x = (x - origin_x_) / cell_size_;
  unsigned int grid_y = (y - origin_y_) / cell_size_;
  if (grid_x >= size_x_ || grid_y >= size_y_) {
    return -1;
  }

  return (grid_y * size_x_) + grid_x;
}

}  // namespace ndt_2d
