#include "vikit/cameras/omni_projection.h"

#include <glog/logging.h>
#include <Eigen/Core>
#include <iostream>

namespace vk {
namespace cameras {

namespace omni_projection {
Eigen::Matrix2d distortionToAffineCorrection(const Eigen::Vector3d& distortion)
{
  Eigen::Matrix2d result;
  // The output of the calibration toolbox is row-major (Matlab), therefore we
  // need to transpose here to get col-major.
  result << 1.0, distortion(2), distortion(1), distortion(0);
  return result;
}
}  // namespace omni_projection

// OmniProjection::OmniProjection(
//     const Eigen::Matrix<double, 5, 1>& polynomial,
//     const Eigen::Vector2d& principal_point, const Eigen::Vector3d& distortion,
//     const Eigen::Matrix<double, 12, 1>& inverse_polynomial)
// : polynomial_(polynomial), principal_point_(principal_point),
//   inverse_polynomial_(inverse_polynomial),
//   affine_correction_(omni_projection::distortionToAffineCorrection(distortion)),
//   affine_correction_inverse_(affine_correction_.inverse())
// {}

// bool OmniProjection::backProject3(
//     const Eigen::Ref<const Eigen::Vector2d>& keypoint,
//     Eigen::Vector3d* out_bearing_vector) const
// {
//   CHECK_NOTNULL(out_bearing_vector);
//   const Eigen::Vector2d rectified =
//       affine_correction_inverse_ * (keypoint - principal_point_);
//   const double rho = rectified.norm();

//   out_bearing_vector->head<2>() = rectified;

//   (*out_bearing_vector)(2) = polynomial_(4);
//   (*out_bearing_vector)(2) = polynomial_(3) + (*out_bearing_vector)(2) * rho;
//   (*out_bearing_vector)(2) = polynomial_(2) + (*out_bearing_vector)(2) * rho;
//   (*out_bearing_vector)(2) = polynomial_(1) + (*out_bearing_vector)(2) * rho;
//   (*out_bearing_vector)(2) = polynomial_(0) + (*out_bearing_vector)(2) * rho;
//   (*out_bearing_vector)(2) = (-1.0) * (*out_bearing_vector)(2);

//   out_bearing_vector->normalize();
//   return true;
// }

// void OmniProjection::project3(
//     const Eigen::Ref<const Eigen::Vector3d>& point_3d,
//     Eigen::Vector2d* out_keypoint,
//     Eigen::Matrix<double, 2, 3>* out_jacobian_point) const
// {
//   CHECK_NOTNULL(out_keypoint);

//   const double x = point_3d[0];
//   const double y = point_3d[1];
//   const double z = -point_3d[2];
//   const double xy_norm2 = std::pow(x, 2) + std::pow(y, 2);
//   const double xy_norm = std::sqrt(xy_norm2);
//   const double z_by_xy_norm = z / xy_norm;
//   const double theta = std::atan(z_by_xy_norm);

//   Eigen::Matrix<double, kInversePolynomialOrder, 1> theta_powers;
//   theta_powers[0] = 1.0;
//   for(int i=1; i<theta_powers.size(); ++i)
//   {
//     theta_powers[i] = theta_powers[i-1]*theta;
//   }

//   const double rho = inverse_polynomial_.dot(theta_powers);

//   Eigen::Vector2d raw_uv;
//   raw_uv(0) = x / xy_norm * rho;
//   raw_uv(1) = y / xy_norm * rho;

//   (*out_keypoint) = affine_correction_ * raw_uv + principal_point_;

//   if(out_jacobian_point) {
//     // rho w.r.t theta
//     double drho_dtheta = 0;
//     for(int i=1; i < kInversePolynomialOrder; i++)
//     {
//       drho_dtheta += i * inverse_polynomial_[i] * theta_powers[i-1];
//     }
//     // theta w.r.t x y z
//     const double xyz_norm_sqr = xy_norm2 + std::pow(z,2);
//     const double dtheta_dx = (-1.0*x*z_by_xy_norm) / (xyz_norm_sqr);
//     const double dtheta_dy = (-1.0*y*z_by_xy_norm) / (xyz_norm_sqr);
//     const double dtheta_dz = xy_norm / (xyz_norm_sqr);
//     // rho w.r.t x y z
//     const double drho_dx = drho_dtheta * dtheta_dx;
//     const double drho_dy = drho_dtheta * dtheta_dy;
//     const double drho_dz = drho_dtheta * dtheta_dz;
//     // uv_raw w.r.t x y z
//     const double xy_sqr = xy_norm2;
//     double duraw_dx = (xy_norm-x*x/xy_norm)/xy_sqr*rho + drho_dx*x/xy_norm;
//     double duraw_dy = (-1.0*x*y/xy_norm)/xy_sqr*rho + drho_dy*x/xy_norm;
//     double duraw_dz = drho_dz*x/xy_norm;
//     double dvraw_dx = (-1.0*x*y/xy_norm)/xy_sqr*rho + drho_dx*y/xy_norm;
//     double dvraw_dy = (xy_norm-y*y/xy_norm)/xy_sqr*rho + drho_dy*y/xy_norm;
//     double dvraw_dz = drho_dz*y/xy_norm;
//     (*out_jacobian_point) << duraw_dx, duraw_dy, -duraw_dz,
//         dvraw_dx, dvraw_dy, -dvraw_dz;

//     // uv w.r.t x y z
//     (*out_jacobian_point) = affine_correction_ * (*out_jacobian_point);
//   }
// }

// void OmniProjection::print(std::ostream& out) const
// {
//   out << "  Projection = Omni" << std::endl;
//   out << "  Polynomial = " << polynomial_.transpose() << std::endl;
//   out << "  Principal point = " << principal_point_.transpose() << std::endl;
//   out << "  Inverse polynomial = " << inverse_polynomial_.transpose()
//       << std::endl;
//   out << "  Affine correction = " << std::endl << affine_correction_
//       << std::endl;
//   out << "  Affine correction inverse = " << std::endl
//       << affine_correction_inverse_ << std::endl;
// }

// Mei model
OmniProjection::OmniProjection(const double &xi, 
                 const Eigen::Matrix<double, 4,1> &distortion, 
                 const Eigen::Matrix<double, 4,1> &projection,
                 const Eigen::Vector2d& principal_point)
: principal_point_(principal_point), xi_(xi), distortion_(distortion), projection_(projection)
{
  k1_ = distortion_(0);
  k2_ = distortion_(1);
  p1_ = distortion_(2);
  p2_ = distortion_(3);
}

bool OmniProjection::backProject3(
    const Eigen::Ref<const Eigen::Vector2d>& keypoint,
    Eigen::Vector3d* out_bearing_vector) const
{
  CHECK_NOTNULL(out_bearing_vector);
  // projective transformation
  const Eigen::DiagonalMatrix<double, 2> focal_matrix(projection_[0], projection_[1]);

  // point w/o distortion
  const Eigen::Vector2d pp = focal_matrix.inverse() * (keypoint - principal_point_);
  
  Eigen::Vector2d pu = pp;

  Eigen::Vector3d Xw;

  // remove distortion iteratively
  if(1)
  {
    double mx_u, my_u;
    double rho2_d = pu[0]*pu[0] + pu[1]*pu[1];
    double mx_d = pu[0];
    double my_d = pu[1];
    if(0)
    {
      // Inverse distortion model
      // proposed by Heikkila
      double rho4_d = rho2_d*rho2_d;

      double mx2_d = pu[0]*pu[0];
      double my2_d = pu[1]*pu[1];
      double mxy_d = pu[0]*pu[1];

      double radDist_d = k1_ * rho2_d + k2_ * rho4_d;
      double Dx_d = mx_d * radDist_d + p2_ * (rho2_d + 2 * mx2_d) + 2 * p1_ * mxy_d;
      double Dy_d = my_d * radDist_d + p1_ * (rho2_d + 2 * my2_d) + 2 * p2_ * mxy_d;
      double inv_denom_d = 1 / (1 + 4 * k1_ * rho2_d + 6 * k2_ * rho4_d + 8 * p1_ * my_d + 8 * p2_ * mx_d);

      double mx_u = mx_d - inv_denom_d * Dx_d;
      double my_u = my_d - inv_denom_d * Dy_d;
    }
    else
    {
      // Recursive distortion model
      int n = 8;
      Eigen::Vector2d d_u;
      distortion(Eigen::Vector2d(mx_d, my_d), d_u);
      // Approximate value
      mx_u = mx_d - d_u(0);
      my_u = my_d - d_u(1);

      for (int i = 1; i < n; ++i)
      {
          distortion(Eigen::Vector2d(mx_u, my_u), d_u);
          mx_u = mx_d - d_u(0);
          my_u = my_d - d_u(1);
      }
    }
    // Reuse variable
    rho2_d = mx_u * mx_u + my_u * my_u;
    Xw << mx_u, my_u, 1.0 - xi_ * (rho2_d + 1.0) / (xi_ + sqrt(1.0 + (1.0 - xi_ * xi_) * rho2_d));
  
  }
  else
  {
    for (int j = 0; j < 20; j++)
    {
      double r2 = pu[0]*pu[0] + pu[1]*pu[1];
      double r4 = r2*r2;
      pu[0] = (pp[0]- 2*p1_*pu[0]*pu[1]- p2_*(r2+2*pu[0]*pu[0])) / (1+k1_*r2+k2_*r4);
      pu[1] = (pp[1]- 2*p2_*pu[0]*pu[1]- p1_*(r2+2*pu[1]*pu[1])) / (1+k1_*r2+k2_*r4);
    }
    // project to unit sphere
    double r2 = pu[0]*pu[0] + pu[1]*pu[1];
    double a = (r2 + 1);
    double b = 2*xi_*r2;
    double c = r2*xi_*xi_-1;

    double Zs = (-b + std::sqrt(b*b - 4*a*c)) / (2*a);
    Xw << pu[0]*(Zs+xi_), pu[1]*(Zs+xi_), -Zs;
  }

  if (Xw(2) != Xw(2))
  {
    *out_bearing_vector << 0.0, 0.0, 1.0; 
    return true;
  }
  out_bearing_vector->head<3>() = Xw;
  out_bearing_vector->normalize();
  return true;
}

void OmniProjection::project3(
    const Eigen::Ref<const Eigen::Vector3d>& point_3d,
    Eigen::Vector2d* out_keypoint,
    Eigen::Matrix<double, 2, 3>* out_jacobian_point) const
{
  CHECK_NOTNULL(out_keypoint);

  if (1)
  {
    double xi = xi_;

    Eigen::Vector2d p_u, p_d;
    double norm, inv_denom;
    double dxdmx, dydmx, dxdmy, dydmy;

    const Eigen::Vector3d P = point_3d;
    norm = P.norm();
    // Project points to the normalised plane
    inv_denom = 1.0 / (P(2) + xi * norm);
    p_u << inv_denom * P(0), inv_denom* P(1);
  
    // Apply distortion
    Eigen::Vector2d d_u;
    double k1 = k1_;
    double k2 = k2_;
    double p1 = p1_;
    double p2 = p2_;

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
        p_u(1)* rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
    
    p_d = p_u + d_u;

    double gamma1 = projection_(0);
    double gamma2 = projection_(1);

    // Apply generalised projection matrix
    (*out_keypoint) << gamma1 * p_d(0) + projection_(2),
                    gamma2* p_d(1) + projection_(3);

    
    if(out_jacobian_point) 
    {
      // Calculate jacobian
      inv_denom = inv_denom * inv_denom / norm;
      double dudx = inv_denom * (norm * P(2) + xi * (P(1) * P(1) + P(2) * P(2)));
      double dvdx = -inv_denom * xi * P(0) * P(1);
      double dudy = dvdx;
      double dvdy = inv_denom * (norm * P(2) + xi * (P(0) * P(0) + P(2) * P(2)));
      inv_denom = inv_denom * (-xi * P(2) - norm); // reuse variable
      double dudz = P(0) * inv_denom;
      double dvdz = P(1) * inv_denom;


      // Distortion model Jacobian
      double dxdmx = 1.0 + rad_dist_u + k1 * 2.0 * mx2_u + k2 * rho2_u * 4.0 * mx2_u + 2.0 * p1 * p_u(1) + 6.0 * p2 * p_u(0);
      double dydmx = k1 * 2.0 * p_u(0) * p_u(1) + k2 * 4.0 * rho2_u * p_u(0) * p_u(1) + p1 * 2.0 * p_u(0) + 2.0 * p2 * p_u(1);
      double dxdmy = dydmx;
      double dydmy = 1.0 + rad_dist_u + k1 * 2.0 * my2_u + k2 * rho2_u * 4.0 * my2_u + 6.0 * p1 * p_u(1) + 2.0 * p2 * p_u(0);

      // Make the product of the jacobians
      // and add projection matrix jacobian
      inv_denom = gamma1 * (dudx * dxdmx + dvdx * dxdmy); // reuse
      dvdx = gamma2 * (dudx * dydmx + dvdx * dydmy);
      dudx = inv_denom;

      inv_denom = gamma1 * (dudy * dxdmx + dvdy * dxdmy); // reuse
      dvdy = gamma2 * (dudy * dydmx + dvdy * dydmy);
      dudy = inv_denom;

      inv_denom = gamma1 * (dudz * dxdmx + dvdz * dxdmy); // reuse
      dvdz = gamma2 * (dudz * dydmx + dvdz * dydmy);
      dudz = inv_denom;

      (*out_jacobian_point) << dudx, dudy, dudz,
                                dvdx, dvdy, dvdz;
    }
  }
  else 
  {
    const double x = point_3d[0];
    const double y = point_3d[1];
    const double z = point_3d[2];
    const double xyz_norm2 = std::pow(x, 2) + std::pow(y, 2) + std::pow(z,2);
    const double xyz_norm = std::sqrt(xyz_norm2);
    // points on unit sphere
    const Eigen::Vector3d Xs = point_3d.head<3>() / xyz_norm;

    // convert to normalized plane
    const double z_plus_xi_norm = 1 / (Xs(2) + xi_);
    const Eigen::Vector2d Xu = Xs.head<2>() * z_plus_xi_norm;

    // add distortion
    const double r2 = std::pow(Xu[0], 2) + std::pow(Xu[1], 2);
    const double r4 = std::pow(r2, 2);
    const double a = 1 + k1_*r2 + k2_*r4;

    Eigen::Vector2d Xd;
    Xd[0] = Xu[0]*a + 2*p1_*Xu[0]*Xu[1] + p2_*(r2+2*Xu[0]*Xu[0]);
    Xd[1] = Xu[1]*a + p1_*(r2+2*Xu[1]*Xu[1]) + 2*p2_*Xu[0]*Xu[1];

    // projective transformation
    const Eigen::DiagonalMatrix<double, 2> focal_matrix(projection_[0], projection_[1]);

    (*out_keypoint) = focal_matrix * Xd + principal_point_;

    if(out_jacobian_point) 
    {
      // Xd wrt Xu
      double temp1 = 2*k1_*Xu[0] + 4*k2_*Xu[0]*r2;
      double temp2 = 2*k1_*Xu[1] + 4*k2_*Xu[1]*r2;
      Eigen::Matrix<double, 2, 2> dXd_dXu;
      dXd_dXu << a+6*p2_*Xu[0]+2*p1_*Xu[1]+Xu[0]*temp1, 2*p1_*Xu[0]+2*p2_*Xu[1]+Xu[0]*temp2,
                                          2*p1_*Xu[0]+2*p2_*Xu[1]+Xu[0]*temp2, a+6*p1_*Xu[1]+2*p2_*Xu[0]+Xu[1]*temp2;
      // dXd_dXu << a+2*p1_*Xu[1]+4*p2_*Xu[0], 2*p1_*Xu[0], 2*p2_*Xu[1], a+4*p1_*Xu[1]+2*p2_*Xu[0];

      // Xu wrt Xs
      Eigen::Matrix<double, 2, 3> dXu_dXs;
      dXu_dXs.leftCols<2>() = Eigen::Matrix2d::Identity() * z_plus_xi_norm;
      dXu_dXs.rightCols<1>() = - Xs.head<2>() * z_plus_xi_norm * z_plus_xi_norm;

      // Xs wrt x y z
      Eigen::Matrix<double, 3, 3> dXs_dx;
      const double xyz_norm3 = std::pow(xyz_norm,3);
      double dxs_dx = (y*y+z*z) / xyz_norm3;
      double dxs_dy = -x*y / xyz_norm3;
      double dxs_dz = -x*z / xyz_norm3;
      double dys_dx = -x*y / xyz_norm3;
      double dys_dy = (x*x+z*z) / xyz_norm3;
      double dys_dz = -y*z / xyz_norm3;
      double dzs_dx = -x*z / xyz_norm3;
      double dzs_dy = -y*z / xyz_norm3;
      double dzs_dz = (y*y+x*x) / xyz_norm3;
      dXs_dx << dxs_dx, dxs_dy, dxs_dz, dys_dx, dys_dy, dys_dz, dzs_dx, dzs_dy, dzs_dz;
      // uv w.r.t x y z
      (*out_jacobian_point) = focal_matrix * dXd_dXu * dXu_dXs * dXs_dx;
    }
  }
}

void OmniProjection::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const
    {
        double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

        mx2_u = p_u(0) * p_u(0);
        my2_u = p_u(1) * p_u(1);
        mxy_u = p_u(0) * p_u(1);
        rho2_u = mx2_u + my2_u;
        rad_dist_u = k1_ * rho2_u + k2_ * rho2_u * rho2_u;
        d_u << p_u(0) * rad_dist_u + 2.0 * p1_ * mxy_u + p2_ * (rho2_u + 2.0 * mx2_u),
            p_u(1)* rad_dist_u + 2.0 * p2_ * mxy_u + p1_ * (rho2_u + 2.0 * my2_u);
    }


void OmniProjection::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u,
            Eigen::Matrix2d& J) const
    {
        double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

        mx2_u = p_u(0) * p_u(0);
        my2_u = p_u(1) * p_u(1);
        mxy_u = p_u(0) * p_u(1);
        rho2_u = mx2_u + my2_u;
        rad_dist_u = k1_ * rho2_u + k2_ * rho2_u * rho2_u;
        d_u << p_u(0) * rad_dist_u + 2.0 * p1_ * mxy_u + p2_ * (rho2_u + 2.0 * mx2_u),
            p_u(1)* rad_dist_u + 2.0 * p2_ * mxy_u + p1_ * (rho2_u + 2.0 * my2_u);

        double dxdmx = 1.0 + rad_dist_u + k1_ * 2.0 * mx2_u + k2_ * rho2_u * 4.0 * mx2_u + 2.0 * p1_ * p_u(1) + 6.0 * p2_ * p_u(0);
        double dydmx = k1_ * 2.0 * p_u(0) * p_u(1) + k2_ * 4.0 * rho2_u * p_u(0) * p_u(1) + p1_ * 2.0 * p_u(0) + 2.0 * p2_ * p_u(1);
        double dxdmy = dydmx;
        double dydmy = 1.0 + rad_dist_u + k1_ * 2.0 * my2_u + k2_ * rho2_u * 4.0 * my2_u + 6.0 * p1_ * p_u(1) + 2.0 * p2_ * p_u(0);

        J << dxdmx, dxdmy,
            dydmx, dydmy;
    }


double OmniProjection::errorMultiplier() const
{
  return 1.0;
}

/* It will be called while processing the 1st and 2nd frame */
double OmniProjection::getAngleError(double img_err) const
{
  Eigen::Vector3d center_f;
  backProject3(principal_point_, &center_f);
  const Eigen::Vector2d offset_x(
      principal_point_(0) + img_err, principal_point_(1));
  Eigen::Vector3d offset_x_f;
  backProject3(offset_x, &offset_x_f);
  const Eigen::Vector2d offset_y(
      principal_point_(0), principal_point_(1) + img_err);
  Eigen::Vector3d offset_y_f;
  backProject3(offset_y, &offset_y_f);

  const double theta_x = std::acos(center_f.dot(offset_x_f));
  const double theta_y = std::acos(center_f.dot(offset_y_f));
  // std::cout << "theta_x = " << theta_x << " " << "theta_y = "<<theta_y << std::endl;

  return (theta_x+theta_y) / 2.0;
}

void OmniProjection::print(std::ostream& out) const
{
  out << "  Projection = Omni" << std::endl;
  out << "  xi = " << xi_ << std::endl;
  out << "  k1 = " << k1_ 
      << "  k2 = " << k2_ 
      << "  p1 = " << p1_ 
      << "  p2 = " << p2_  << std::endl;
  out << "  Projection (fx,fy) = " << projection_.head<2>().transpose() << std::endl;
  out << "  Principal_point (cx,cy) = " << principal_point_.transpose() << std::endl;
}

Eigen::VectorXd OmniProjection::getIntrinsicParameters() const
{
  Eigen::VectorXd intrinsics(7);

  // intrinsics.resize(7);
  // for(int idx = 0; idx < 5; idx++)
  // {
  //   intrinsics(idx) = polynomial_(idx);
  // }

  // intrinsics(5) = principal_point_(0);
  // intrinsics(6) = principal_point_(1);

  intrinsics.resize(4);
  for (int i = 0; i <4; i++)
  {
    intrinsics(i) = projection_(i);
  }
  
  return intrinsics;
}

Eigen::VectorXd OmniProjection::getDistortionParameters() const
{
  Eigen::VectorXd distortion(4);

  // distortion(0) = affine_correction_(0,0);
  // distortion(1) = affine_correction_(0,1);
  // distortion(2) = affine_correction_(1,0);
  // distortion(3) = affine_correction_(1,1);
  for (int i =0; i <4; i++)
  {
    distortion(i) = distortion_(i);
  }
  
  return distortion;
}

}  // namespace cameras
}  // namespace vk
