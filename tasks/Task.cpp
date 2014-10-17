/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

//#define DEBUG_PRINTS 1

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
    SingleStateCovariance processCovQ; processCovQ.setIdentity();

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
            //#ifdef DEBUG_PRINTS
            std::cout<<"[LOCALIZATION TASK] Received Exteroceptive sample at time "<< extero_sample.delta_pose.time.toString()<<"\n";
            //#endif
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
    /** The filter vector state variables one for the navigation quantities the other for the error **/
    WAugmentedState vstate;

    /************************************/
    /** Initialize the Back-End Filter **/
    /************************************/

    /** Initial covariance matrix **/
    StateFilter::SingleStateCovariance P0single; /** Initial P(0) for one state **/
    P0single.setZero();

    MTK::setDiagonal (P0single, &WSingleState::pos, 1e-03);
    MTK::setDiagonal (P0single, &WSingleState::orient, 1e-03);
    MTK::setDiagonal (P0single, &WSingleState::velo, 1e-10);
    MTK::setDiagonal (P0single, &WSingleState::angvelo, 1e-10);


    /** Initial covariance matrix for the Vector of States **/
    StateFilter::AugmentedStateCovariance P0; /** Initial P(0) for the whole Vector State **/

    MTK::subblock (P0, &WAugmentedState::statek, &WAugmentedState::statek) = P0single;
    MTK::subblock (P0, &WAugmentedState::statek_l, &WAugmentedState::statek_l) = P0single;
    MTK::subblock (P0, &WAugmentedState::statek_i, &WAugmentedState::statek_i) = P0single;

    MTK::subblock (P0, &WAugmentedState::statek, &WAugmentedState::statek_l) = P0single;
    MTK::subblock (P0, &WAugmentedState::statek, &WAugmentedState::statek_i) = P0single;
    MTK::subblock (P0, &WAugmentedState::statek_l, &WAugmentedState::statek) = P0single;
    MTK::subblock (P0, &WAugmentedState::statek_i, &WAugmentedState::statek) = P0single;

    MTK::subblock (P0, &WAugmentedState::statek_l, &WAugmentedState::statek_i) = P0single;
    MTK::subblock (P0, &WAugmentedState::statek_i, &WAugmentedState::statek_l) = P0single;

    /** Set the current pose to initialize the filter structure **/
    vstate.statek_i.pos = tf.translation(); //!Initial position
    vstate.statek_i.orient = Eigen::Quaternion<double>(tf.rotation());

    /** Set the initial velocities in the state vector **/
    vstate.statek_i.velo.setZero(); //!Initial linear velocity
    vstate.statek_i.angvelo.setZero(); //!Initial angular velocity

    /** Copy the state in the vector of states **/
    vstate.statek = vstate.statek_i;
    vstate.statek_l = vstate.statek_i;

    /** Create the filter **/
    filter.reset (new StateFilter (static_cast<const WAugmentedState> (vstate),
                        static_cast<const StateFilter::AugmentedStateCovariance> (P0)));

    #ifdef DEBUG_PRINTS
    std::cout<<"\n";
    std::cout<<"[LOCALIZATION INIT] State P0|0 is of size " <<P0single.rows()<<" x "<<P0single.cols()<<"\n";
    std::cout<<"[LOCALIZATION INIT] State P0|0:\n"<<P0single<<"\n";
    std::cout<<"[LOCALIZATION INIT] Augmented P0|0 is of size " <<P0.rows()<<" x "<<P0.cols()<<"\n";
    std::cout<<"[LOCALIZATION INIT] Augmented P0|0:\n"<<P0<<"\n";
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

