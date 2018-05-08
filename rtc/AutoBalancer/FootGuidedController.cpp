/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "FootGuidedController.h"


void foot_guided_control_base::set_mat(const double dz)
{
  xi = std::sqrt(g / dz);
  h = 1 + xi * dt;
  h_ = 1 - xi * dt;
  A <<
    1.0, dt,
    xi * xi * dt, 1.0;
  b <<
    0.0,
    -xi * xi * dt;
  Phi <<
    1.0, 1.0 / xi,
    1.0, -1.0 / xi;
  Phi_inv <<
    1.0, 1.0,
    xi, -xi;
  Phi_inv = 2.0 * Phi_inv;
}

// Discrete ver.
// void foot_guided_control_base::calc_u(const std::size_t N, const double ref_dcm, const double ref_zmp)
// {
//   if ( N > 0 ) {
//     double hn = std::pow(h, N);
//     double hgain =  (hn / h) * (1 + h) / ( 1 - hn * hn );
//     w_k = Phi * x_k;
//     double dxsp = ref_dcm - ref_zmp, xcp = w_k(0) - ref_zmp + w_k_offset;
//     u_k = ref_zmp - hgain * (hn * xcp - dxsp);
//   } else {
//     u_k = ref_zmp;
//   }
//   truncate_u();
// }

void foot_guided_control_base::calc_u(const std::size_t N, const double ref_dcm, const double ref_zmp)
{
  if ( N > 0 ) {
    w_k = Phi * x_k;
    double dxsp = ref_dcm - ref_zmp, xcp = w_k(0) - ref_zmp + w_k_offset, T = N * dt;
    T = std::max(50e-3, T); // lower limit, refer ; Bipedal walking control based on capture point dynamics
    u_k = ref_zmp + 2 * (xcp - std::exp(- xi * T) * dxsp) / (1 - std::exp(-2 * xi * T));
  } else {
    u_k = ref_zmp;
  }
}


void foot_guided_control_base::truncate_u()
{
  double acc;
  get_acc(acc);
  if (std::abs(acc) > mu * g) {
    acc = mu * g * sgn(acc);
    u_k = x_k(0) - acc / (xi * xi);
  }
}

// assumed after calc_u
void foot_guided_control_base::calc_x_k()
{
  x_k = A * x_k + b * u_k;
}

void foot_guided_control_base::update_control(double& zmp, const std::size_t N, const double ref_dcm, const double ref_zmp)
{
  calc_u(N, ref_dcm, ref_zmp);
  zmp = u_k;
}

void foot_guided_control_base::update_state(double& pos)
{
  calc_x_k();
  pos = x_k(0);
}

void foot_guided_control_base::update(double& zmp, double& pos, const std::size_t N, const double ref_dcm, const double ref_zmp)
{
  update_control(zmp, N, ref_dcm, ref_zmp);
  update_state(pos);
}