#pragma once

#include <Eigen>
#include <opencv2/opencv.hpp>

class TreeNode {
  TreeNode(Eigen::Vector3d base, Eigen::Vector3d centroid, double d, double num_points):
    base_(base), centroid_(centroid), d_(d), num_points_(num_points) {}

  Eigen::Vector3d base_;
  Eigen::Vector3d centroid_;
  double d_;
  double num_points_;
};

class Octree {
public:
  Octree(Eigen::Vector3d o, double r, double fineness);

  Eigen::Vector3d o_;
  double r_, fineness_;
  TreeNode *root_;
};
