#ifndef LOCALIZATION_TYPES_H
#define LOCALIZATION_TYPES_H

#include <vector>
#include <string>

#include <boost/uuid/uuid.hpp>

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

        /* Names of the input index. Index previous to next feature. **/
        std::string index_name;

        /* Names of the input covariance. Inaccuracies in the displacement-estimation. **/
        std::string pointcloud_name;

        /* Names of the input covariance. Inaccuracies in the displacement-estimation. **/
        std::string covariance_name;
    };

    struct Feature
    {
        boost::uuids::uuid index; // Indexes of the points/samples uses to compute the relative measurement
        base::Vector3d point; // Point cloud used for the delta displacement
        base::Matrix3d cov; // Covariance of the points/samples uses to compute the relative measurement
    };

    struct ExteroFeatures
    {
        base::Time time;
        std::vector<Feature> features;
    };

    typedef boost::uuids::uuid samples_uuid;
}

#endif

