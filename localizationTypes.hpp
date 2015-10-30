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

    enum UpdateType
    {
        EKF, // EKF update
        EKF_OB, // EKF with Observability analysis
        UKF, //Unscented update
        NO_UPDATE //NO UPDATE
    };

    typedef boost::uuids::uuid samples_uuid;

    /** Envire graph types **/
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


    /** Output port type **/
    struct FilterInfo
    {
        base::Time time; //time-stamp
        base::Time predict_execution_time;
        base::Time update_execution_time;
        base::Time add_features_execution_time;
        base::Time remove_features_execution_time;
        unsigned int number_features_added;
        unsigned int number_features_removed;
        unsigned int number_outliers;
        std::vector< base::samples::RigidBodyState > sensors_rbs; //Rbs with the orientation and position of the contact point
        base::MatrixXd Pk; //filter covariance matrix
    };
}

namespace visual_stereo
{
    typedef boost::uuids::uuid samples_uuid;

    /** Input port types **/
    struct Feature
    {
        boost::uuids::uuid index; // Indexes of the points/samples uses to compute the relative measurement
        base::Vector3d stereo_point; // 2D stereo point (u_left, u_right, v)
        base::Vector3d point_3d; // 3D point (x, y, z)
        base::Matrix3d cov_3d; // Covariance of the points/samples uses to compute the relative measurement
    };

    struct ExteroFeatures
    {
        base::Time time;
        unsigned int img_idx;
        std::vector<Feature> features;
    };
}

#endif

