//
// Created by aska on 19-2-16.
//

#include "Octree.h"
#include <algorithm>

// TreeNode -------------------------------------------------------------------------------

bool TreeNode::Intersect(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir) {
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
        return false;
      }
    }
    else {
      min_t[i] = (base_(i) - pos(i)) / dir(i);
      max_t[i] = min_t[i] + d_ / dir(i);
    }
  }
  return std::max(std::max(min_t[0], min_t[1]), min_t[2]) < std::min(std::min(max_t[0], max_t[1]), max_t[2]);
}

Eigen::Vector3d TreeNode::CalcSingleForce(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir, double fineness) {
  Eigen::Vector3d bias = centroid_ - pos;
  double len = bias.dot(dir);
  Eigen::Vector3d ort_vec = bias - len * dir;
  // TODO: Hard code here;
  double dis = std::abs(ort_vec.norm() - fineness) / fineness + 1e-2;
  return ort_vec / (ort_vec.norm() * dis * dis) * num_points_;
}

// Octree ---------------------------------------------------------------------------------

Octree::Octree(Eigen::Vector3d o, double r, double fineness): o_(o), r_(r), fineness_(fineness) {
  root_ = new TreeNode(Eigen::Vector3d(o) - Eigen::Vector3d(r, r, r),
    Eigen::Vector3d(0.0, 0.0, 0.0), r * 2.0, 0);
}


Eigen::Vector3d Octree::CalcForce(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir) {
  ray_pos_ = pos;
  ray_dir_ = dir;
  return CalcForce(root_) / (double) root_->num_points_;
}

Eigen::Vector3d Octree::CalcForce(TreeNode *node) {
  if (!root_->Intersect(ray_pos_, ray_dir_)) {
    return root_->CalcSingleForce(ray_pos_, ray_dir_, fineness_);
  }
  else {
    Eigen::Vector3d ret_force(0.0, 0.0, 0.0);
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
