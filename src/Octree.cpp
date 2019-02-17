//
// Created by aska on 19-2-16.
//

#include "Octree.h"

Octree::Octree(Eigen::Vector3d o, double r, double fineness): o_(o), r_(r), fineness_(fineness) {
  root_ = new TreeNode(Eigen::Vector3d(o) - Eigen::Vector3d(r, r, r));
}