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

double Calibrator::CalcCurrentF() {
  double current_f = 0.0;
  for (const auto &pix : visible_pixs_) {
    Eigen::Vector3d dir = pix(0) * x_axis_ + pix(1) * y_axis_ + scale_ratio_ * z_axis_;
    dir /= dir.norm();
    current_f += octree_->CalcForce(pos_, dir);
  }
  return current_f;
}

void Calibrator::Calibrate() {
  double go_len = 0.1;
  while (go_len > 1e-6) {
    double current_f = CalcCurrentF();
    const double step_len = 1.0 / (1 << 10);
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(6);

    for (int i = 0; i < 3; i++) {
      pos_(i) += step_len;
      grad(i) = (CalcCurrentF() - current_f) / step_len;
      pos_(i) -= step_len;
    }

    auto current_x_axis = x_axis_;
    auto current_y_axis = y_axis_;
    auto current_z_axis = z_axis_;

    for (int i = 0; i < 3; i++) {
      Eigen::Vector3d rot(0.0, 0.0, 0.0);
      rot(i) = 1.0;
      Eigen::Matrix3d t;
      t = Eigen::AngleAxisd(step_len, rot);
      x_axis_ = t * current_x_axis;
      y_axis_ = t * current_y_axis;
      z_axis_ = t * current_z_axis;

      grad(i + 3) = (CalcCurrentF() - current_f) / step_len;
    }
    grad *= go_len / grad.norm();
    pos_ += Eigen::Vector3d(grad(0), grad(1), grad(2));
    Eigen::Vector3d rot(grad(3), grad(4), grad(5));
    Eigen::Matrix3d t;
    t = Eigen::AngleAxisd(step_len, rot);
    x_axis_ = t * current_x_axis;
    y_axis_ = t * current_y_axis;
    x_axis_ /= x_axis_.norm();
    y_axis_ = y_axis_ - x_axis_ * x_axis_.dot(y_axis_);
    y_axis_ /= y_axis_.norm();
    z_axis_ = x_axis_.cross(y_axis_);
    go_len *= 0.9;
    std::cout << CalcCurrentF() << std::endl;
  }
  // Update
}

void Calibrator::AddToOctree() {
  for (const auto &pix : visible_pixs_) {
    Eigen::Vector3d dir = pix(0) * x_axis_ + pix(1) * y_axis_ + scale_ratio_ * z_axis_;
    dir /= dir.norm();
    octree_->Add(pos_, dir);
  }
}


void Calibrator::CopyCamParasFrom(Calibrator *another_calibrator) {
  pos_ = another_calibrator->pos_;
  x_axis_ = another_calibrator->x_axis_;
  y_axis_ = another_calibrator->y_axis_;
  z_axis_ = another_calibrator->z_axis_;
}