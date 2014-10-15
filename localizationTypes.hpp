#ifndef LOCALIZATION_TYPES_H
#define LOCALIZATION_TYPES_H

#include <vector>
#include <string>

#include <base/time.h>
#include <base/eigen.h>
#include <base/samples/RigidBodyState.hpp>

namespace localization
{
    struct OutputPortsConfiguration
    {
        /** Name of the input port */
        std::string output_name;

        /**  This is a four element vector **/

        /* 1. Name of the input ports with the delta displacement, */
        std::string delta_pose_name;

        /* 2. Name of the input Jacobian with respect to the exteroceptive measurements at time k,*/
        std::string jacobian_k_name;

        /* 3. Names of the input Jacobian with respect to the exteroceptive measurements at time k+m,*/
        std::string jacobian_k_m_name;

        /* 4. Names of the input covariance. Inaccuracies in the displacement-estimation. **/
        std::string covariance_name;
    };

    struct ExteroceptiveSample
    {
        base::samples::RigidBodyState delta_pose; // Relative displacement
        base::MatrixXd jacobian_k; // Displacement Jacobian with respect of the point/samples at time k
        base::MatrixXd jacobian_k_m; // Displacement Jacobian with respect of the point/samples at time k+m
        base::MatrixXd covariance; // Uncertainty of the points/samples uses to compute the relative measurement
    };
}

#endif


