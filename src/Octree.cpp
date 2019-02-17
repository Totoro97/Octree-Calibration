//
// Created by aska on 19-2-16.
//

#include "Octree.h"
#include <algorithm>

// TreeNode -------------------------------------------------------------------------------

double TreeNode::ToF(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir) {
  double min_t[3], max_t[3];
  // TODO: Hard code here.
  const double eps = 1e-8;
  const double inf = 1e10;
  for (int i = 0; i < 3; i++) {
    if (dir(i) < eps && dir(i) > -eps) {
      if (pos(i) > base_(i) && pos(i) < base_(i) + d_) {
        min_t[i] = -inf;
        max_t[i] = inf;
      } else {
        return -eps;
      }
    }
    else {
      min_t[i] = (base_(i) - pos(i)) / dir(i);
      max_t[i] = min_t[i] + d_ / dir(i);
    }
  }
  return std::min(std::min(max_t[0], max_t[1]), max_t[2]) - std::max(std::max(min_t[0], min_t[1]), min_t[2]);
}

bool TreeNode::Intersect(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir) {
  return ToF(pos, dir) > 0.0;
}

double TreeNode::CalcSingleForce(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir, double fineness) {
  Eigen::Vector3d bias = centroid_ - pos;
  double len = bias.dot(dir);
  Eigen::Vector3d ort_vec = bias - len * dir;
  // TODO: Hard code here;
  return 1.0 / (1.0 + std::exp(-ort_vec.norm())) * num_points_;
}

// Octree ---------------------------------------------------------------------------------

Octree::Octree(Eigen::Vector3d o, double r, double fineness): o_(o), r_(r), fineness_(fineness) {
  root_ = new TreeNode(Eigen::Vector3d(o) - Eigen::Vector3d(r, r, r),
    Eigen::Vector3d(0.0, 0.0, 0.0), r * 2.0, 0);
}


double Octree::CalcForce(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir) {
  ray_pos_ = pos;
  ray_dir_ = dir;
  return CalcForce(root_) / (double) root_->num_points_;
}

double Octree::CalcForce(TreeNode *node) {
  double tof = root_->ToF(ray_pos_, ray_dir_);
  if (tof <= 0.0) {
    return root_->CalcSingleForce(ray_pos_, ray_dir_, fineness_);
  }
  else {
    double ret_force = 0.0;
    int idx = -1;
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          ++idx;
          if (node->sons_[idx] == nullptr) {
            continue;
          }
          ret_force += CalcForce(node->sons_[idx]);
        }
      }
    }
    return ret_force;
  }
}

