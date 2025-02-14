/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_CONSTANTS_
#define ROBOWFLEX_CONSTANTS_

#include <Eigen/Core>
#include <boost/math/constants/constants.hpp>

namespace robowflex
{
    namespace constants
    {
        // common
        static const double half = boost::math::constants::half<double>();
        static const double third = boost::math::constants::third<double>();
        static const double eps = std::numeric_limits<double>::epsilon();
        static const double inf = std::numeric_limits<double>::infinity();
        static const double nan = std::numeric_limits<double>::quiet_NaN();

        // pi
        static const double pi = boost::math::constants::pi<double>();
        static const double half_pi = boost::math::constants::half_pi<double>();
        static const double quarter_pi = half_pi * half;
        static const double two_pi = boost::math::constants::two_pi<double>();

        // tolerances
        static const double ik_tolerance = 0.001;
        static const unsigned int ik_attempts = 50;
        static const Eigen::Vector3d ik_vec_tolerance = {ik_tolerance, ik_tolerance, ik_tolerance};
        static const double cart_rot_step_size = 0.01;
        static const double cart_pos_step_size = 0.01;
        static const double cart_rot_jump_tol = 0.25;
        static const double cart_pos_jump_tol = 0.25;

    }  // namespace constants
}  // namespace robowflex

#endif
