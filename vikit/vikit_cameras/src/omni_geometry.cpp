#include "vikit/cameras/omni_geometry.h"

#include <glog/logging.h>

namespace vk {
namespace cameras {

// Scaramuzza
// Verison contains mask
// OmniGeometry::OmniGeometry(
//     const int width, const int height,
//     const Eigen::Matrix<double, 5, 1>& polynomial,
//     const Eigen::Vector2d& principal_point, const Eigen::Vector3d& distortion,
//     const Eigen::Matrix<double, 12, 1>& inverse_polynomial,
//     const Eigen::Vector2d& mask_relative_radii)
// : CameraGeometry(width, height, OmniProjection(
//     polynomial, principal_point, distortion, inverse_polynomial))
// {
//   if (mask_relative_radii != kDontMask)
//   {
//     std::cout<<"omni with mask"<<std::endl;
//     CHECK_GE(mask_relative_radii(0), 0.0);
//     CHECK_LE(mask_relative_radii(1), 1.0);
//     CHECK_LT(mask_relative_radii(0), mask_relative_radii(1));
//     CHECK_GE(width, height) << "Only cameras whose width is larger than its "
//         << "height are supported";

//     const int radius = height/2;
//     const double low_radius_sqaured =
//         std::pow(radius * mask_relative_radii(0), 2);
//     const double high_radius_squared =
//         std::pow(radius * mask_relative_radii(1), 2);

//     // NOTE: opencv accept rows and column
//     cv::Mat mask(height, width, CV_8UC1, cv::Scalar(0));

//     // Loop and set mask: 0 masked / 1 valid.
//     Eigen::Vector2i i;
//     for(i(0) = 0; i(0) < width; ++i(0))
//       for(i(1) = 0; i(1) < height; ++i(1)) {
//         Eigen::Vector2i p_c = i - principal_point.cast<int>();
//         const double dist_squared = p_c.squaredNorm();
//         if (dist_squared > high_radius_squared ||
//             dist_squared < low_radius_sqaured)
//         {
//           mask.at<uint8_t>(i(1), i(0)) = 0;
//         }
//         else
//         {
//           mask.at<uint8_t>(i(1), i(0)) = 1;
//         }
//       }

//     setMask(mask);
//   }
// }

// // Version which does not apply a mask.
// OmniGeometry::OmniGeometry(
//     const int width, const int height,
//     const Eigen::Matrix<double, 5, 1>& polynomial,
//     const Eigen::Vector2d& principal_point, const Eigen::Vector3d& distortion,
//     const Eigen::Matrix<double, 12, 1>& inverse_polynomial)
// : OmniGeometry(width, height, polynomial, principal_point, distortion,
//                inverse_polynomial, kDontMask)
// {
//   std::cout<<"Version which does not apply a mask."<<std::endl;
// }

// directly from intrinsics
// OmniGeometry::OmniGeometry(
//     const int width, const int height,
//     const Eigen::VectorXd &intrinsics)
// : OmniGeometry(width, height,
//                intrinsics.block<5, 1>(0, 0),
//                intrinsics.block<2, 1>(5, 0),
//                intrinsics.block<3, 1>(7, 0),
//                intrinsics.block<12, 1>(10, 0),
//                intrinsics.block<2, 1>(22,0))
// {
//   std::cout<<"omni with directly from intrinsics"<<std::endl;
// }

// Mei omniGeometry
OmniGeometry::OmniGeometry(const int width, const int height,
                           const double &xi,
                           const Eigen::Matrix<double, 4, 1> &distortion,
                           const Eigen::Matrix<double, 4, 1> &projection,
                           const Eigen::Vector2d &principal_point,
                           const Eigen::Vector2d& mask_relative_radii)
:CameraGeometry(width, height, OmniProjection(xi, distortion, projection, principal_point))
{
  std::cout<<"Mei model"<<std::endl;
  if (mask_relative_radii != kDontMask)
  {
    std::cout<<"omni with mask"<<std::endl;
    CHECK_GE(mask_relative_radii(0), 0.0);
    CHECK_LE(mask_relative_radii(1), 1.0);
    CHECK_LT(mask_relative_radii(0), mask_relative_radii(1));
    CHECK_GE(width, height) << "Only cameras whose width is larger than its "
        << "height are supported";

    const int radius = height/2;
    const double low_radius_sqaured =
        std::pow(radius * mask_relative_radii(0), 2);
    const double high_radius_squared =
        std::pow(radius * mask_relative_radii(1), 2);

    // NOTE: opencv accept rows and column
    cv::Mat mask(height, width, CV_8UC1, cv::Scalar(0));

    // Loop and set mask: 0 masked / 1 valid.
    Eigen::Vector2i i;
    for(i(0) = 0; i(0) < width; ++i(0))
      for(i(1) = 0; i(1) < height; ++i(1)) {
        Eigen::Vector2i p_c = i - principal_point.cast<int>();
        const double dist_squared = p_c.squaredNorm();
        if (dist_squared > high_radius_squared ||
            dist_squared < low_radius_sqaured)
        {
          mask.at<uint8_t>(i(1), i(0)) = 0;
        }
        else
        {
          mask.at<uint8_t>(i(1), i(0)) = 1;
        }
      }

    setMask(mask);
  }
}

// Mei omniGeometry w/o mask.
OmniGeometry::OmniGeometry(
    const int width, const int height,
    const double &xi,
    const Eigen::Matrix<double, 4, 1> &distortion,
    const Eigen::Matrix<double, 4, 1> &projection,
    const Eigen::Vector2d &principal_point)
: OmniGeometry(width, height, xi, distortion, projection, principal_point, kDontMask)
{
  std::cout<<"Version which does not apply a mask."<<std::endl;
  // this->printParameters(std::cout);
}

OmniGeometry::OmniGeometry(const int width, const int height,
                           const Eigen::VectorXd &intrinsic)
:OmniGeometry(width, height, 
              intrinsic(0), // xi
              intrinsic.block<4,1>(1,0),  // distortion(k1,k2,p1,p2)
              intrinsic.block<4,1>(5,0),  // projection(f1,f2,u,v)
              intrinsic.block<2,1>(7,0),  // principal_point(u,v)
              intrinsic.block<2,1>(9,0))  // mask_radii
{
  std::cout<<"Omni with directly from intrinsics " <<std::endl;
}


const Eigen::Vector2d OmniGeometry::kDontMask = Eigen::Vector2d::Zero();

}  // namespace cameras
}  // namespace vk
