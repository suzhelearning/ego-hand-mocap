#include <cassert>
#include <iostream>
#include <okvis/Frame.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>

class TwoFeatures final : public cv::Feature2D {
 public:
  void detect(cv::InputArray, std::vector<cv::KeyPoint>& points,
              cv::InputArray = cv::noArray()) override {
    points = {cv::KeyPoint(16,16,5), cv::KeyPoint(24,16,5)};
  }
};

int main() {
  using Camera=okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion>;
  auto camera=std::make_shared<Camera>(32,32,20,20,16,16,
      okvis::cameras::RadialTangentialDistortion(0,0,0,0));
  cv::Mat mask(32,32,CV_8UC1,cv::Scalar(255));
  mask.at<unsigned char>(16,16)=0;
  mask.at<unsigned char>(16,24)=1; // Any nonzero value must be valid.
  assert(camera->setMask(mask));
  Eigen::Vector2d pixel;
  assert(camera->project(Eigen::Vector3d(0,0,1),&pixel)==okvis::cameras::ProjectionStatus::Masked);
  assert(camera->project(Eigen::Vector3d(.4,0,1),&pixel)==okvis::cameras::ProjectionStatus::Successful);
  assert(!camera->setMask(cv::Mat(16,16,CV_8UC1)));
  okvis::Frame frame;
  frame.setGeometry(camera);
  frame.setImage(cv::Mat(32,32,CV_8UC1,cv::Scalar(0)));
  frame.setDetector(std::make_shared<TwoFeatures>());
  assert(frame.detect()==1); // The excluded feature must never reach matching.
  std::cout << "PASS: mask polarity, dimensions, projection and feature filtering\n";
}
