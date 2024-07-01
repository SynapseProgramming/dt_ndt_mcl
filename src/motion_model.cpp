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

#include <angles/angles.h>

#include <cmath>
#include <dt_ndt_mcl/motion_model.hpp>
#include <iostream>

namespace ndt_2d
{

#define angle_diff angles::shortest_angular_distance
#define angle_norm angles::normalize_angle

  // Draw randomly from a zero-mean Gaussian distribution, with standard
  // deviation sigma.
  // We use the polar form of the Box-Muller transformation, explained here:
  //   http://www.taygeta.com/random/gaussian.html
  double MotionModel::pf_ran_gaussian(double sigma)
  {
    double x1, x2, w, r;

    do
    {
      do
      {
        r = drand48();
      } while (r == 0.0);
      x1 = 2.0 * r - 1.0;
      do
      {
        r = drand48();
      } while (r == 0.0);
      x2 = 2.0 * r - 1.0;
      w = x1 * x1 + x2 * x2;
    } while (w > 1.0 || w == 0.0);

    return sigma * x2 * sqrt(-2.0 * log(w) / w);
  }

  MotionModel::MotionModel(double a1, double a2, double a3, double a4, double a5)
      : a1_(a1), a2_(a2), a3_(a3), a4_(a4), a5_(a5) {}

  void MotionModel::sample(const double dx, const double dy, const double dth, Pose2d &prev_pose,
                           std::vector<Eigen::Vector3d> &poses)
  {
    // Decompose relative motion
    double trans = std::hypot(dx, dy);
    double rot1 = (trans > 0.01) ? angle_diff(prev_pose.theta, atan2(dy, dx)) : 0.0;
    double rot2 = angle_diff(rot1, dth);

    // Reverse motion should not cause massive errors
    double rot1_ = std::min(std::fabs(angle_diff(rot1, 0.0)),
                            std::fabs(angle_diff(rot1, M_PI)));
    double rot2_ = std::min(std::fabs(angle_diff(rot2, 0.0)),
                            std::fabs(angle_diff(rot2, M_PI)));

    // Determine standard deviation
    double sigma_rot1 = std::sqrt(a1_ * rot1_ * rot1_ + a2_ * trans * trans);
    double sigma_trans = std::sqrt(a3_ * trans * trans + a4_ * rot1_ * rot1_ +
                                   a4_ * rot2_ * rot2_);
    double sigma_rot2 = std::sqrt(a1_ * rot2_ * rot2_ + a2_ * trans * trans);

    for (auto &pose : poses)
    {

      double r1 = angle_diff(pf_ran_gaussian(sigma_rot1), rot1);
      double t = trans - pf_ran_gaussian(sigma_trans);
      double r2 = angle_diff(pf_ran_gaussian(sigma_rot2), rot2);

      pose(0) += t * cos(pose(2) + r1);
      pose(1) += t * sin(pose(2) + r1);
      pose(2) = angle_norm(pose(2) + r1 + r2);
    }
  }

} // namespace ndt_2d
