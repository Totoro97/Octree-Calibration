#include "GlobalCalibrator.h"

int main() {
  auto global_calibrator = new GlobalCalibrator("/home/aska/Data", 2);
  global_calibrator->Run();
  delete(global_calibrator);
  return 0;
}