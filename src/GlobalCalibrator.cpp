#include "GlobalCalibrator.h"
#include "Thinning.h"

GlobalCalibrator::GlobalCalibrator(std::string dir_name, int num_frame): num_frame_(num_frame) {
  octree_ = new Octree();
  for (int i = 0; i < num_frame; i++) {
    auto img = cv::imread(dir_name + "/" + std::to_string(i) + ".png");
    cv::Mat img_gray(img.rows, img.cols, CV_8UC1);
    for (int i = 0; i < img.rows * img.cols; i++) {
      if (img.data[i * 3] == img.data[0] &&
          img.data[i * 3 + 1] == img.data[1] &&
          img.data[i * 3 + 2] == img.data[2]) {
        img_gray.data[i] = 0;
      }
      else
        img_gray.data[i] = (uchar) 255;
    }

    thinning(img_gray, img_gray);
    calibrators_.push_back(new Calibrator(img_gray, octree_, i));
  }
}

GlobalCalibrator::~GlobalCalibrator() {
  for (auto calibrator : calibrators_) {
    delete(calibrator);
  }
  delete(octree_);
}

// No depth parameters (Estimate from Octree)

void GlobalCalibrator::Run() {
  calibrators_[0]->AddToOctree();
  for (auto iter = std::next(calibrators_.begin()); iter != calibrators_.end(); iter++) {
    auto calibrator = *iter;
    calibrator->CopyCamParasFrom(*std::prev(iter));
    calibrator->Calibrate();
    calibrator->AddToOctree();
  }
}