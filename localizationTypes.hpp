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

    struct FilterInfo
    {
        base::Time time; //time-stamp
        base::Time predict_execution_time;
        base::Time update_execution_time;
        base::Time add_features_execution_time;
        std::vector< base::samples::RigidBodyState > sensors_rbs; //Rbs with the orientation and position of the contact point
        base::MatrixXd Pk; //filter covariance matrix
    };


    typedef boost::uuids::uuid samples_uuid;
}

#endif

