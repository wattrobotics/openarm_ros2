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

#include "openarm_gravity_comp/dynamics.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

namespace openarm_gravity_comp {

Dynamics::Dynamics(std::string urdf_path, std::string start_link,
                   std::string end_link)
    : urdf_path_(std::move(urdf_path)),
      start_link_(std::move(start_link)),
      end_link_(std::move(end_link)) {}

bool Dynamics::Init() {
  std::ifstream file(urdf_path_);
  if (!file.is_open()) {
    std::fprintf(stderr, "[Dynamics] Failed to open URDF: %s\n",
                 urdf_path_.c_str());
    return false;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  urdf_model_ = urdf::parseURDF(buffer.str());
  if (!urdf_model_) {
    std::fprintf(stderr, "[Dynamics] Failed to parse URDF: %s\n",
                 urdf_path_.c_str());
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(*urdf_model_, kdl_tree_)) {
    std::fprintf(stderr, "[Dynamics] Failed to build KDL tree\n");
    return false;
  }

  if (!kdl_tree_.getChain(start_link_, end_link_, kdl_chain_)) {
    std::fprintf(stderr,
                 "[Dynamics] Failed to extract chain '%s' -> '%s'\n",
                 start_link_.c_str(), end_link_.c_str());
    return false;
  }

  gravity_forces_.resize(kdl_chain_.getNrOfJoints());
  gravity_forces_.data.setZero();

  solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity_vec_);
  return true;
}

void Dynamics::SetGravityVector(double gx, double gy, double gz) {
  gravity_vec_ = KDL::Vector(gx, gy, gz);
  solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity_vec_);
}

void Dynamics::GetGravity(const double* motor_position, double* gravity) {
  const size_t n = kdl_chain_.getNrOfJoints();
  KDL::JntArray q(n);
  for (size_t i = 0; i < n; ++i) {
    q(i) = motor_position[i];
  }
  solver_->JntToGravity(q, gravity_forces_);
  for (size_t i = 0; i < n; ++i) {
    gravity[i] = gravity_forces_(i);
  }
}

void Dynamics::PrintModelSummary() const {
  std::cout << "[Dynamics] URDF: " << urdf_path_ << "\n"
            << "[Dynamics] Chain: '" << start_link_ << "' -> '" << end_link_
            << "'\n"
            << "[Dynamics] DOF (KDL joints): " << kdl_chain_.getNrOfJoints()
            << "\n";

  const auto& link_map = urdf_model_->links_;
  size_t seg_idx = 0;
  size_t inertial_ok = 0;
  for (const auto& seg : kdl_chain_.segments) {
    const auto& joint = seg.getJoint();
    const std::string child_link = seg.getName();

    bool has_inertial = false;
    double mass = -1.0;
    auto it = link_map.find(child_link);
    if (it != link_map.end() && it->second && it->second->inertial) {
      has_inertial = true;
      mass = it->second->inertial->mass;
      ++inertial_ok;
    }

    std::cout << "  [seg " << seg_idx++ << "] link='" << child_link
              << "' joint='" << joint.getName() << "' type=" << joint.getType()
              << " inertial=" << (has_inertial ? "yes" : "NO");
    if (has_inertial) {
      std::cout << ", m=" << mass;
    }
    std::cout << "\n";
  }
  std::cout << "[Dynamics] Inertials present on " << inertial_ok << "/"
            << kdl_chain_.segments.size() << " segments." << std::endl;
}

}  // namespace openarm_gravity_comp
