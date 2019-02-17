#pragma once

#include <Eigen>
#include <opencv2/opencv.hpp>

#include <cstring>

class TreeNode {
public:
  TreeNode(Eigen::Vector3d base, Eigen::Vector3d centroid, double d, double num_points):
    base_(base), centroid_(centroid), d_(d), num_points_(num_points) {
    for (int i = 0; i < 8; i++) {
      sons_[i] = nullptr;
    }
  }

  bool Intersect(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir);
  double ToF(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir);
  double CalcSingleForce(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir, double fineness);

  Eigen::Vector3d base_;
  Eigen::Vector3d centroid_;
  double d_;
  double num_points_;
  TreeNode *sons_[8];
};

class Octree {
public:
  Octree(Eigen::Vector3d o, double r, double fineness);
  double CalcForce(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir);
  double CalcForce(TreeNode *node);

  Eigen::Vector3d o_;
  Eigen::Vector3d ray_pos_, ray_dir_;
  double r_, fineness_;
  TreeNode *root_;
};
