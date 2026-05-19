// Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
//
// Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
// https://www.openarmx.com
//
// This work is licensed under the Creative Commons Attribution-NonCommercial-
// ShareAlike 4.0 International License (CC BY-NC-SA 4.0).
// To view a copy of this license, visit:
// http://creativecommons.org/licenses/by-nc-sa/4.0/
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
//
// Adapted for openarm_gravity_comp (OpenArm V10) by Enactic, Inc.
// Trimmed to the subset of the API required for gravity feedforward:
//   Init / SetGravityVector / GetGravity / NumJoints / PrintModelSummary.

#pragma once

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>

#include <memory>
#include <string>

namespace openarm_gravity_comp {

class Dynamics {
 public:
  Dynamics(std::string urdf_path, std::string start_link, std::string end_link);
  ~Dynamics() = default;

  bool Init();
  void SetGravityVector(double gx, double gy, double gz);
  void GetGravity(const double* motor_position, double* gravity);

  size_t NumJoints() const { return kdl_chain_.getNrOfJoints(); }
  const std::string& StartLink() const { return start_link_; }
  const std::string& EndLink() const { return end_link_; }

  // Logs each segment and whether it carries inertial data — useful for
  // verifying that KDL pulled mass/com from the URDF before any compensation
  // is applied.
  void PrintModelSummary() const;

 private:
  std::string urdf_path_;
  std::string start_link_;
  std::string end_link_;

  std::shared_ptr<urdf::ModelInterface> urdf_model_;
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::JntArray gravity_forces_;
  KDL::Vector gravity_vec_{0.0, 0.0, -9.81};
  std::unique_ptr<KDL::ChainDynParam> solver_;
};

}  // namespace openarm_gravity_comp
