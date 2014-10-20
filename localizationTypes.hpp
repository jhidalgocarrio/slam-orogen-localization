#ifndef LOCALIZATION_TYPES_H
#define LOCALIZATION_TYPES_H

#include <vector>
#include <string>

#include <base/time.h>
#include <base/eigen.h>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Pointcloud.hpp>

namespace localization
{
    struct OutputPortsConfiguration
    {
        /** Name of the input port */
        std::string output_name;

        /**  This is a five element vector **/

        /* 1. Name of the input ports with the delta displacement, */
        std::string delta_pose_name;

        /* 2. Names of the input covariance. Inaccuracies in the displacement-estimation. **/
        std::string pointcloud_name;

        /* 3. Names of the input covariance. Inaccuracies in the displacement-estimation. **/
        std::string covariance_name;

        /* 4. Name of the input Jacobian with respect to the exteroceptive measurements at time k,*/
        std::string jacobian_k_name;

        /* 5. Names of the input Jacobian with respect to the exteroceptive measurements at time k+m,*/
        std::string jacobian_k_m_name;

    };

    struct ExteroceptiveSample
    {
        base::samples::RigidBodyState delta_pose; // Relative displacement
        base::samples::Pointcloud point_cloud; // Point cloud used for the delta displacement
        std::vector<base::Matrix3d> covariance; // Uncertainty of the points/samples uses to compute the relative measurement
        base::MatrixXd jacobian_k; // Displacement Jacobian with respect of the point/samples at time k
        base::MatrixXd jacobian_k_m; // Displacement Jacobian with respect of the point/samples at time k+m
    };
}

#endif

