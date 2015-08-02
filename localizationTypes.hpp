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
    class FeatureMeasurement
    {
    public:

        boost::uuids::uuid index; // Indexes
        base::Vector2d point; // Point in camera frame

        FeatureMeasurement(){}

        FeatureMeasurement(boost::uuids::uuid _index):index(_index){}

        FeatureMeasurement(boost::uuids::uuid _index,
                            base::Vector2d &_point):index(_index), point(_point){}
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
        unsigned int img_idx;
        std::vector<Feature> features;
    };


    typedef boost::uuids::uuid samples_uuid;
}

#endif

