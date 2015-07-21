/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

#define DEBUG_PRINTS 1

using namespace localization;

/** Process model when accumulating delta poses **/
WSingleState processModel (const WSingleState &state,  const Eigen::Vector3d &delta_position, const localization::SO3 &delta_orientation,
                            const Eigen::Vector3d &velocity, const Eigen::Vector3d &angular_velocity)
{
    WSingleState s2; /** Propagated state */

    /** Apply Rotation **/
    s2.orient = state.orient * delta_orientation;
    s2.angvelo = angular_velocity;

    /** Apply Translation **/
    s2.pos = state.pos + (s2.orient * delta_position);
    s2.velo = velocity;

    return s2;
};


MeasurementType measurementModelK_K_i (const WMultiState &wastate)
{
    MeasurementType z_hat;
//    std::cout<<"z_hat "<<z_hat<<"\n";

    return z_hat;
};



Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{

    /******************************/
    /*** Control Flow Variables ***/
    /******************************/
    initFilter = false;

    /**************************/
    /** Input port variables **/
    /**************************/
    delta_pose.invalidate();
}

Task::~Task()
{
}

void Task::delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::BodyState &delta_pose_samples_sample)
{

    if(!initFilter)
    {
        /***************************/
        /** Filter Initialization **/
        /***************************/

        Eigen::Affine3d tf; /** Transformer transformation **/

        /** Get the transformation **/
        if (_navigation_frame.value().compare(_world_frame.value()) == 0)
        {
            tf.setIdentity();
        }
        else if (!_navigation2world.get(ts, tf, false))
        {
           RTT::log(RTT::Fatal)<<"[LOCALIZATION FATAL ERROR]  No transformation provided."<<RTT::endlog();
           return;
        }

        #ifdef DEBUG_PRINTS
        std::cout<<"[LOCALIZATION POSE_SAMPLES] - Initializing Filter...";
        #endif

        /** Initialization of the filter **/
        this->initMultiStateFilter (filter, tf);

        #ifdef DEBUG_PRINTS
        std::cout<<"[DONE]\n";
        #endif

        initFilter = true;
    }

    #ifdef DEBUG_PRINTS
    base::Time delta_t = delta_pose_samples_sample.time - this->delta_pose.time;
    //base::Time delta_t = base::Time::fromSeconds(_pose_samples_period.get());
    std::cout<<"[LOCALIZATION POSE_SAMPLES] Received new samples at "<<delta_pose_samples_sample.time.toString()<<"\n";
    std::cout<<"[LOCALIZATION POSE_SAMPLES] delta_t: "<<delta_t.toSeconds()<<"\n";
    #endif

    /** A new sample arrived to the input port **/
    this->delta_pose = delta_pose_samples_sample;

    /********************/
    /** Filter Predict **/
    /********************/

    /** Process Model Uncertainty **/
    typedef MultiStateFilter::SingleStateCovariance SingleStateCovariance;
    SingleStateCovariance cov_process; cov_process.setZero();
    MTK::subblock (cov_process, &WSingleState::pos, &WSingleState::pos) = this->delta_pose.cov_position();
    MTK::subblock (cov_process, &WSingleState::orient, &WSingleState::orient) = this->delta_pose.cov_orientation();
    MTK::subblock (cov_process, &WSingleState::velo, &WSingleState::velo) = this->delta_pose.cov_linear_velocity();
    MTK::subblock (cov_process, &WSingleState::angvelo, &WSingleState::angvelo) = this->delta_pose.cov_angular_velocity();

    /** Predict the filter state **/
    filter->predict(boost::bind(processModel, _1 ,
                            static_cast<const Eigen::Vector3d>(delta_pose.position()),
                            static_cast<const localization::SO3>(Eigen::Quaterniond(delta_pose.orientation())),
                            static_cast<const Eigen::Vector3d>(delta_pose.linear_velocity()),
                            static_cast<const Eigen::Vector3d>(delta_pose.angular_velocity())),
                            cov_process);

    this->outputPortSamples(delta_pose.time);
}

void Task::visual_features_samplesTransformerCallback(const base::Time &ts, const ::localization::ExteroFeatures &visual_features_samples_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/

    /** Get the transformation (transformation) Tbody_left_camera **/
    if (_body_frame.value().compare(_sensor_frame.value()) == 0)
    {
        tf.setIdentity();
    }
    else if (!_sensor2body.get(ts, tf, false))
    {
        throw std::runtime_error("[THREED_ODOMETRY] Transformation from imu to body is not provided.");
        return;
    }


    /** Perform Measurements Update **/
    #ifdef DEBUG_PRINTS
    std::cout<<"[LOCALIZATION VISUAL_FEATURES] Received sample at time "<< visual_features_samples_sample.time.toString()<<"\n";
    std::cout<<"[LOCALIZATION VISUAL_FEATURES] Received Measurements Number "<< visual_features_samples_sample.features.size()<<"\n";
    #endif

    if (initFilter)
    {
        if (filter->muState().sensorsk.size() == _maximum_number_sensor_poses.value())
        {
            /** Perform an update when reached maximum number of camera sensor poses **/
            std::cout<<"[LOCALIZATION VISUAL_FEATURES] mu_state\n"<<filter->muState()<<"\n";
            std::cout<<"[LOCALIZATION VISUAL_FEATURES] Pk is of size "<<filter->getPk().rows()<<" x "<<filter->getPk().cols()<<"\n";
            std::cout<<"[LOCALIZATION VISUAL_FEATURES] Pk\n"<<filter->getPk()<<"\n";


            /** Remove sensor pose from filter **/
            std::string sensor_pose = removeSensorPoseFromFilter(filter);

            /** Remove sensor pose from envire **/

        }

        /** New sensor camera pose in the filter**/
        this->addSensorPoseToFilter(filter, tf);

        /** New measurement to envire **/
        //this->addMeasurementToEnvire(filter, visual_features_samples_sample);

    }

    std::cout<<"[LOCALIZATION VISUAL_FEATURES] **** END ****\n";

    /** Get the measurement vector and uncertainty **/
    MeasurementType measurement;
    Eigen::Matrix<MultiStateFilter::ScalarType, Eigen::Dynamic, Eigen::Dynamic> measurementCov;


}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /***********************/
    /** Configure Values  **/
    /***********************/

    initFilter = false;

    /** Output port **/
    pose_out.invalidate();
    pose_out.sourceFrame = _localization_source_frame.value();

    /** Relative Frame to port out the samples **/
    pose_out.targetFrame = _world_frame.value();

    /** Initialize environment tree **/

    /***********************/
    /** Info and Warnings **/
    /***********************/
    RTT::log(RTT::Warning)<<"[LOCALIZATION TASK] Desired Target Frame is "<<pose_out.targetFrame<<RTT::endlog();

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    /** Liberate the memory of the shared_ptr **/
    filter.reset();

    /** Remove the content in the envire tree **/
    envire_tree.clear();
}

void Task::initMultiStateFilter(boost::shared_ptr<MultiStateFilter> &filter, Eigen::Affine3d &tf)
{
    /** The filter vector state variables for the navigation quantities **/
    WMultiState statek_0;
    WSingleState single_state;

    /** Set the current pose to initialize the filter structure **/
    single_state.pos = tf.translation(); //!Initial position
    single_state.orient = Eigen::Quaternion<double>(tf.rotation());

    /** Set the initial velocities in the state vector **/
    single_state.velo.setZero(); //!Initial linear velocity
    single_state.angvelo.setZero(); //!Initial angular velocity

    /** Store the single state into the state vector **/
    statek_0.statek = single_state;

    /************************************/
    /** Initialize the Back-End Filter **/
    /************************************/

    /** Initial covariance matrix **/
    MultiStateFilter::SingleStateCovariance P0_single; /** Initial P(0) for the state **/
    MultiStateCovariance P0;
    P0_single.setZero();
    P0.resize(WSingleState::DOF, WSingleState::DOF);
    P0.setIdentity();

    MTK::setDiagonal (P0_single, &WSingleState::pos, 1e-06);
    MTK::setDiagonal (P0_single, &WSingleState::orient, 1e-06);
    MTK::setDiagonal (P0_single, &WSingleState::velo, 1e-10);
    MTK::setDiagonal (P0_single, &WSingleState::angvelo, 1e-10);

    P0.block(0, 0, WSingleState::DOF, WSingleState::DOF) = P0_single;

    /** Create the filter **/
    filter.reset (new MultiStateFilter (statek_0, P0));

    #ifdef DEBUG_PRINTS
    WMultiState vstate = filter->muState();
    std::cout<<"\n";
    std::cout<<"[LOCALIZATION INIT] State P0|0 is of size " <<P0_single.rows()<<" x "<<P0_single.cols()<<"\n";
    std::cout<<"[LOCALIZATION INIT] State P0|0:\n"<<P0_single<<"\n";
    std::cout<<"[LOCALIZATION INIT] Multi State P0|0 is of size " <<filter->getPk().rows()<<" x "<<filter->getPk().cols()<<"\n";
    std::cout<<"[LOCALIZATION INIT] Multi State P0|0:\n"<<filter->getPk()<<"\n";
    std::cout<<"[LOCALIZATION INIT] state:\n"<<vstate.getVectorizedState()<<"\n";
    std::cout<<"[LOCALIZATION INIT] position:\n"<<vstate.statek.pos<<"\n";
    Eigen::Vector3d euler; /** In Euler angles **/
    euler[2] = vstate.statek.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
    euler[1] = vstate.statek.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
    euler[0] = vstate.statek.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
    std::cout<<"[LOCALIZATION INIT] orientation Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
    std::cout<<"[LOCALIZATION INIT] velocity:\n"<<vstate.statek.velo<<"\n";
    std::cout<<"[LOCALIZATION INIT] angular velocity:\n"<<vstate.statek.angvelo<<"\n";
    std::cout<<"\n";
    #endif

    return;
}


void Task::outputPortSamples(const base::Time &timestamp)
{
    WSingleState statek_i = filter->muState().statek;

    pose_out.time = timestamp;
    pose_out.position = statek_i.pos;
    pose_out.cov_position = filter->getPkSingleState().block<3,3>(0,0);
    pose_out.orientation = statek_i.orient;
    pose_out.cov_orientation = filter->getPkSingleState().block<3,3>(3,3);
    pose_out.velocity = statek_i.velo;
    pose_out.cov_velocity =  filter->getPkSingleState().block<3,3>(6,6);
    pose_out.angular_velocity = statek_i.angvelo;
    pose_out.cov_angular_velocity =  filter->getPkSingleState().block<3,3>(9,9);
    _pose_samples_out.write(pose_out);

}


void Task::addSensorPoseToFilter(boost::shared_ptr<MultiStateFilter> filter, Eigen::Affine3d &tf)
{
    Eigen::Quaternion <double> qtf = Eigen::Quaternion <double> (tf.rotation());

    /** Create the camera pose **/
    localization::SensorState camera_pose;
    camera_pose.orient = filter->muSingleState().orient * qtf;//Tnav_camera = Tnav_body * Tbody_camera
    Eigen::Vector3d pbody_camera = camera_pose.orient * tf.translation(); //pbody_camera in navigation frame
    camera_pose.pos = filter->muSingleState().pos + pbody_camera; //pnav_camera = pnav_body + Tnav_camera * pbody_camera

    /** Jacobian matrix for new sensor camera pose estimation **/
    MultiStateCovariance J (filter->muState().getDOF() + localization::SensorState::DOF, filter->muState().getDOF()); J.setZero();
    J.block(0, 0, filter->muState().getDOF(), filter->muState().getDOF()) = Eigen::MatrixXd::Identity(filter->muState().getDOF(), filter->muState().getDOF());

    /** Identity in position **/
    J.block(filter->muState().getDOF(), 0, localization::vec3::DOF, localization::vec3::DOF) = Eigen::MatrixXd::Identity(localization::vec3::DOF, localization::vec3::DOF);

    /** Skew symmetric matrix with orientation of the state **/
    J.block(filter->muState().getDOF(), localization::vec3::DOF, localization::vec3::DOF, localization::vec3::DOF) = Task::makeSkewSymmetric(pbody_camera);

    /* Sensor orientation **/
    J.block(filter->muState().getDOF()+localization::vec3::DOF, localization::vec3::DOF, localization::vec3::DOF, localization::vec3::DOF) = tf.rotation();

    /** Extend state by new camera pose **/
    filter->muState().sensorsk.push_back(camera_pose);

    /** Extend filter covariance matrix **/
    MultiStateCovariance Pk_i (filter->muState().getDOF(), filter->muState().getDOF());

    /** Multiplication **/
    Pk_i = J * filter->getPk() * J.transpose();

    /** Store the new Pk in the filter **/
    filter->setPk(Pk_i);

    #ifdef DEBUG_PRINTS
    std::cout<<"[LOCALIZATION ADD_SENSOR_POSE] J is of size "<<J.rows()<<" x "<<J.cols()<<"\n";
    std::cout<<"[LOCALIZATION ADD_SENSOR_POSE] J\n"<<J<<"\n";
    std::cout<<"[LOCALIZATION ADD_SENSOR_POSE] Pk is of size "<<filter->getPk().rows()<<" x "<<filter->getPk().cols()<<"\n";
    #endif

}


std::string Task::removeSensorPoseFromFilter(boost::shared_ptr<MultiStateFilter> filter)
{
    MultiStateCovariance Pk_i (filter->muState().getDOF(), filter->muState().getDOF());
    Pk_i = filter->getPk();

    #ifdef DEBUG_PRINTS
    std::cout<<"[LOCALIZATION REMOVE_SENSOR_POSE] Sensor size: "<<filter->muState().sensorsk.size()<<" == maximum("<<_maximum_number_sensor_poses.value()<<")\n";
    std::cout<<"[LOCALIZATION REMOVE_SENSOR_POSE] Pk_i is of size "<<Pk_i.rows()<<" x "<<Pk_i.cols()<<"\n";
    #endif

    /**Remove in the state **/
    std::srand( time( NULL ) );  //  using the time seed from srand explanation
    unsigned int it_dice = std::rand() % (filter->muState().sensorsk.size());
    filter->muState().sensorsk.erase(filter->muState().sensorsk.begin() + it_dice);

    #ifdef DEBUG_PRINTS
    std::cout<<"[LOCALIZATION REMOVE_SENSOR_POSE] it_dice: "<<it_dice<<"\n";
    #endif

    /**Remove in the covariance matrix **/
    unsigned int i_remove = WSingleState::DOF + (localization::SensorState::DOF * it_dice);
    for (register size_t i = 0; i < localization::SensorState::DOF; ++i)
    {
        //std::cout<<"i -> "<<i<<" i_remove: "<<i_remove<<"\n";

        /** Remove row and column **/
        Task::removeRow(Pk_i, i_remove);
        Task::removeColumn(Pk_i, i_remove);
        //std::cout<<"Pk_i is of size "<<Pk_i.rows()<<" x "<<Pk_i.cols()<<"\n";
    }

    filter->setPk(Pk_i);

    return std::string("pose_" + boost::lexical_cast<std::string>(it_dice));
}

