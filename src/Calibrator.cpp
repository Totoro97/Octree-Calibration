//
// Created by aska on 19-1-29.
//

#include "Calibrator.h"


Calibrator::Calibrator(const cv::Mat &img_gray, Octree* octree, int frame_id) :
  img_gray_(img_gray), octree_(octree), frame_id_(frame_id) {
  height_ = img_gray.rows;
  width_ = img_gray.cols;

  // TODO: Hard code here.
  pos_ = Eigen::Vector3d(10.0, 0.0, 0.0);
  x_axis_ = Eigen::Vector3d(0.0, 1.0, 0.0);
  y_axis_ = Eigen::Vector3d(0.0, 0.0, -1.0);
  z_axis_ = Eigen::Vector3d(-1.0, 0.0, 0.0);

  // TODO: Need Sampling?
  for (int i = 0; i < height_; i++) {
    for (int j = 0; j < width_; j++) {
      if (img_gray_.data[i * width_ + j] == 255) {
        visible_pixs_.emplace_back(j - width_ * 0.5, i - height_ * 0.5);
      }
    }
  }
}

void Calibrator::Calibrate() {

  while (true) {
    Eigen::Vector3d add_trans(0.0, 0.0, 0.0);
    Eigen::Vector3d add_rot(0.0, 0.0, 0.0);

    for (const auto &pix : visible_pixs_) {
      Eigen::Vector3d dir = pix(0) * x_axis_ + pix(1) * y_axis_ + scale_ratio_ * z_axis_;
      dir /= dir.norm();
      Eigen::Vector3d current_f = octree_->CalcForce(pos_, dir);
      Eigen::Vector3d tan_f = dir * current_f.dot(dir);
      Eigen::Vector3d ort_f = current_f - tan_f;
      // TODO: Hard code here.
      add_trans += tan_f + ort_f * 1.0;
      add_rot += dir.cross(ort_f);
    }


    double step_len = 1e-3;
  }
  // Update
}

