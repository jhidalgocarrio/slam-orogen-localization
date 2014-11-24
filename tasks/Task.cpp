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

WSingleState processModel (const WSingleState &state,  const Eigen::Vector3d &velocity, const Eigen::Vector3d &angular_velocity, double dt)
{
    WSingleState s2; /** Propagated state */

    /** Apply Rotation **/
    Eigen::Vector3d scaled_axis = angular_velocity * dt;
    SO3 rot = SO3::exp (scaled_axis);
    s2.orient = state.orient * rot ;
    s2.angvelo = angular_velocity;

    /** Apply Translation **/
    s2.velo = velocity;
    s2.pos = state.pos + state.velo * dt;

    return s2;
};

WSingleState processModel (const WSingleState &state,  const Eigen::Vector3d &delta_position, const localization::SO3 &delta_orientation,
                            const Eigen::Vector3d &velocity, const Eigen::Vector3d &angular_velocity)
{
    WSingleState s2; /** Propagated state */

    /** Apply Rotation **/
    s2.orient = state.orient * delta_orientation;
    s2.angvelo = angular_velocity;

    /** Apply Translation **/
    s2.velo = velocity;
    s2.pos = state.pos + delta_position;

    return s2;
};


localization::AugmentedState<Eigen::Dynamic>::MeasurementType measurementModelK_K_i (const WAugmentedState &wastate)
{
    WSingleState delta_state, statek, statek_i; /** Propagated state */
    localization::AugmentedState<Eigen::Dynamic>::MeasurementType z_hat;
    z_hat = wastate.featuresk;
    statek = wastate.statek;
    statek_i = wastate.statek_i;

    delta_state = statek - statek_i;
    Eigen::Affine3d delta_transform (delta_state.orient);
    delta_transform.translation() = delta_state.pos;

    for (register unsigned int i = 0; i < z_hat.size(); i+=3)
    {
        Eigen::Vector3d coord;
        coord<<wastate.featuresk[i], wastate.featuresk[i+1], wastate.featuresk[i+2];
        coord = delta_transform * coord;
        z_hat[i] = coord[0];
        z_hat[i+1] = coord[1];
        z_hat[i+2] = coord[2];
    }
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
    pose_sample.invalidate();
}

Task::~Task()
{
}

void Task::pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{

    if(!initFilter)
    {
        /** Set an initial pose samples **/
        pose_sample = pose_samples_sample;

        /***************************/
        /** Filter Initialization **/
        /***************************/

        Eigen::Affine3d tf; /** Transformer transformation **/

        /** Get the transformation **/
        if (boost::iequals(_localization_target_frame.value(), _world_frame.value()))
        {
            if (!_navigation2world.get(ts, tf, false))
            {
               RTT::log(RTT::Fatal)<<"[LOCALIZATION FATAL ERROR]  No transformation provided."<<RTT::endlog();
               return;
            }
        }
        else
        {
            tf.setIdentity();
        }

        #ifdef DEBUG_PRINTS
        std::cout<<"[LOCALIZATION POSE_SAMPLES] - Initializing Filter...";
        #endif

        /** Initialization of the filter with the first measurements **/
        this->initStateFilter (filter, tf);

        #ifdef DEBUG_PRINTS
        std::cout<<"[DONE]\n";
        #endif

        initFilter = true;
    }

    base::Time delta_t = pose_samples_sample.time - pose_sample.time;
    //base::Time delta_t = base::Time::fromSeconds(_pose_samples_period.get());

    /** A new sample arrived to the input port **/
    pose_sample = pose_samples_sample;

    #ifdef DEBUG_PRINTS
    std::cout<<"[LOCALIZATION POSE_SAMPLES] Received new samples at "<<pose_samples_sample.time.toString()<<"\n";
    std::cout<<"[LOCALIZATION POSE_SAMPLES] delta_t: "<<delta_t.toSeconds()<<"\n";
    #endif

    /********************/
    /** Filter Predict **/
    /********************/

    /** Process Model Uncertainty **/
    typedef StateFilter::SingleStateCovariance SingleStateCovariance;
    SingleStateCovariance processCovQ; processCovQ.setZero();
    MTK::subblock (processCovQ, &WSingleState::velo, &WSingleState::velo) = pose_sample.cov_velocity;
    MTK::subblock (processCovQ, &WSingleState::angvelo, &WSingleState::angvelo) = pose_sample.cov_angular_velocity;

    /** Predict the filter state **/
    filter->predict(boost::bind(processModel, _1 ,
                            static_cast<const Eigen::Vector3d>(pose_sample.velocity),
                            static_cast<const Eigen::Vector3d>(pose_sample.angular_velocity),
                            delta_t.toSeconds()), processCovQ);

    this->outputPortSamples(pose_sample.time);
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
    if (!boost::iequals(_localization_target_frame.value(), _world_frame.value()))
    {
        _localization_target_frame.set(_navigation_frame.value());
    }

    pose_out.targetFrame = _localization_target_frame.value();

    /***********************************/
    /** Dynamic Exteroceptive Inputs  **/
    /***********************************/
    std::vector<std::string> exteroceptive_config (_exteroceptive_inputs.value());
    clearPorts(); // make sure all created ports are removed first, in case we aborted one configureHook already
    for (size_t i = 0; i < exteroceptive_config.size(); ++i)
    {
        /** Create the input ports **/
        std::string const& name(exteroceptive_config[i]);
        if (!getPort(name))
        {
            InputPortExtero* port = new InputPortExtero(name);
            mInputExtero.push_back(port);
            addEventPort(*port);
        }
    }

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

    for (register size_t i = 0; i < mInputExtero.size(); ++i)
    {
        /** Exteroceptive sample **/
        localization::ExteroceptiveSample extero_sample;

        if (mInputExtero[i]->read(extero_sample, false) == RTT::NewData)
        {
            /** Perform Measurements Update **/
            #ifdef DEBUG_PRINTS
            std::cout<<"[LOCALIZATION TASK] Received Exteroceptive "<<mInputExtero[i]->getName()<<"sample at time "<< extero_sample.delta_pose.time.toString()<<"\n";
            std::cout<<"[LOCALIZATION TASK] Received Measurements Number "<< extero_sample.point_cloud.points.size()<<"\n";
            std::cout<<"[LOCALIZATION TASK] Received Jacobian k size "<< extero_sample.jacobian_k.rows()<<" x "<<extero_sample.jacobian_k.cols()<<"\n";
            std::cout<<"[LOCALIZATION TASK] Received Jacobian k+m size "<< extero_sample.jacobian_k_m.rows()<<" x "<<extero_sample.jacobian_k_m.cols()<<"\n";
            #endif

            /** Get the measurement vector and uncertainty **/
            localization::AugmentedState<Eigen::Dynamic>::MeasurementType measurement;
            Eigen::Matrix<StateFilter::ScalarType, Eigen::Dynamic, Eigen::Dynamic> measurementCov;
//            size_t number_measurements = std::max (NUMBER_MEASUREMENTS, extero_sample.point_cloud.points.size());
//            measurement.resize(3*NUMBER_MEASUREMENTS, 1);
            measurement.resize(3*extero_sample.point_cloud.points.size());
            measurementCov.resize(measurement.size(), measurement.size());


            /** Measurement for state k **/
            if (i == 0)
            {
                register size_t k = 0;
                measurement.setZero();
                measurementCov.setZero();

                /** First measurement **/
                if (filter->muState().featuresk.size() == 0 )
                {

                    for (register size_t j = 0; j < extero_sample.point_cloud.points.size(); ++j)
                    {
                        measurement.block(0+k, 0, 3, 1) = extero_sample.point_cloud.points[j];
                        measurementCov.block(0+k, 0+k, 3, 3) = extero_sample.covariance[j];
                        k=k+3;
                        if (k >= static_cast<size_t>(measurement.size()))
                            break;

                    }

                    /** Set the first features samples in the vector state **/
                    filter->setMeasurement<localization::AugmentedState<Eigen::Dynamic>::MeasurementType,
                        Eigen::Matrix<StateFilter::ScalarType, Eigen::Dynamic, Eigen::Dynamic> >(localization::STATEK, measurement, measurementCov);

                    #ifdef DEBUG_PRINTS
                    std::cout<<"[LOCALIZATION TASK] FIRST_FEATURES_SAMPLES "<<std::endl;
                    std::cout<<"[LOCALIZATION TASK] Measurement\n"<<measurement<<"\n";
                    std::cout<<"[LOCALIZATION TASK] Filter featuresk size "<<filter->muState().featuresk.size() <<"\n";
                    std::cout<<"[LOCALIZATION TASK] Pk is of size "<<filter->PkAugmentedState().rows() <<" x "<<filter->PkAugmentedState().cols()<<std::endl;
                    std::cout<<"[LOCALIZATION TASK] Filter State "<<filter->muState()<<"\n";
                    #endif
                }
                else
                {
                    #ifdef DEBUG_PRINTS
                    std::cout<<"[LOCALIZATION TASK] UPDATE "<<std::endl;
                    std::cout<<"[LOCALIZATION TASK] Measurement\n"<<measurement<<"\n";
                    std::cout<<"[LOCALIZATION TASK] Measurement Covariance size "<<measurementCov.rows() <<" x "<< measurementCov.cols()<<"\n";
                    std::cout<<"[LOCALIZATION TASK] Filter featuresk size "<<filter->muState().featuresk.size() <<"\n";
                    #endif

                    /************/
                    /** UPDATE **/
                    /************/
                    filter->updateEKF(static_cast< Eigen::Matrix<StateFilter::ScalarType, Eigen::Dynamic, 1> > (measurement), measurementCov, measurementCov);
                    //filter->update(static_cast< Eigen::Matrix<StateFilter::ScalarType, Eigen::Dynamic, 1> > (measurement), boost::bind(measurementModelK_K_i, _1), measurementCov);

                }
            }
            else if (i == 1) /** Measurement for state k+l **/
            {
                /** NO IMPLEMENTED **/

            }
        }
    }
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

    /** Clean ports **/
    clearPorts();

    /** Liberate the memory of the shared_ptr **/
    filter.reset();
}

void Task::initStateFilter(boost::shared_ptr<StateFilter> &filter, Eigen::Affine3d &tf)
{
    /** The filter vector state variables for the navigation quantities **/
    WSingleState single_state;

    /************************************/
    /** Initialize the Back-End Filter **/
    /************************************/

    /** Initial covariance matrix **/
    StateFilter::SingleStateCovariance P0_single; /** Initial P(0) for one state **/
    P0_single.setZero();

    MTK::setDiagonal (P0_single, &WSingleState::pos, 1e-06);
    MTK::setDiagonal (P0_single, &WSingleState::orient, 1e-06);
    MTK::setDiagonal (P0_single, &WSingleState::velo, 1e-10);
    MTK::setDiagonal (P0_single, &WSingleState::angvelo, 1e-10);

    /** Set the current pose to initialize the filter structure **/
    single_state.pos = tf.translation(); //!Initial position
    single_state.orient = Eigen::Quaternion<double>(tf.rotation());

    /** Set the initial velocities in the state vector **/
    single_state.velo.setZero(); //!Initial linear velocity
    single_state.angvelo.setZero(); //!Initial angular velocity

    /** Create the filter **/
    filter.reset (new StateFilter (static_cast<const WSingleState> (single_state),
                        static_cast<const StateFilter::SingleStateCovariance> (P0_single)));

    #ifdef DEBUG_PRINTS
    WAugmentedState vstate = filter->muState();
    std::cout<<"\n";
    std::cout<<"[LOCALIZATION INIT] State P0|0 is of size " <<P0_single.rows()<<" x "<<P0_single.cols()<<"\n";
    std::cout<<"[LOCALIZATION INIT] State P0|0:\n"<<P0_single<<"\n";
    std::cout<<"[LOCALIZATION INIT] Augmented P0|0 is of size " <<filter->PkAugmentedState().rows()<<" x "<<filter->PkAugmentedState().cols()<<"\n";
    std::cout<<"[LOCALIZATION INIT] Augmented P0|0:\n"<<filter->PkAugmentedState()<<"\n";
    std::cout<<"[LOCALIZATION INIT] state:\n"<<vstate.getVectorizedState()<<"\n";
    std::cout<<"[LOCALIZATION INIT] position:\n"<<vstate.statek_i.pos<<"\n";
    Eigen::Matrix <double,localization::NUMAXIS,1> euler; /** In Euler angles **/
    euler[2] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
    euler[1] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
    euler[0] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
    std::cout<<"[LOCALIZATION INIT] orientation Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
    std::cout<<"[LOCALIZATION INIT] velocity:\n"<<vstate.statek_i.velo<<"\n";
    std::cout<<"[LOCALIZATION INIT] angular velocity:\n"<<vstate.statek_i.angvelo<<"\n";
    std::cout<<"\n";
    #endif

    return;
}


void Task::outputPortSamples(const base::Time &timestamp)
{
    WSingleState statek_i = filter->muState().statek_i;

    pose_out.time = timestamp;
    pose_out.position = statek_i.pos;
    pose_out.cov_position = filter->PkSingleState().block<3,3>(0,0);
    pose_out.orientation = statek_i.orient;
    pose_out.cov_orientation = filter->PkSingleState().block<3,3>(3,3);
    pose_out.velocity = statek_i.velo;
    pose_out.cov_velocity =  filter->PkSingleState().block<3,3>(6,6);
    pose_out.angular_velocity = statek_i.angvelo;
    pose_out.cov_angular_velocity =  filter->PkSingleState().block<3,3>(9,9);
    _pose_samples_out.write(pose_out);

}

void Task::clearPorts()
{
    /** Input ports delta displacements **/
    for (register size_t i = 0; i < mInputExtero.size(); ++i)
    {
        ports()->removePort(mInputExtero[i]->getName());
        delete mInputExtero[i];
    }
    mInputExtero.clear();
}

