/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "StateOptimize.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

//#define DEBUG_PRINTS 1

using namespace rover_localization;

StateOptimize::StateOptimize(std::string const& name)
    : StateOptimizeBase(name)
{
    /******************************/
    /*** Control Flow Variables ***/
    /******************************/
    initFilter = false;
    number.reset();
    counter.reset();
    flag.reset();

    /******************************************/
    /*** General Internal Storage Variables ***/
    /******************************************/
    slipVector.data = base::NaN<double>() * Eigen::Matrix<double, 3, 1>::Ones();
    slipVector.Cov = base::NaN<double>() * Eigen::Matrix<double, 3, 3>::Ones();

    /**************************/
    /** Input port variables **/
    /**************************/
    poseSamples =  boost::circular_buffer< ::base::samples::RigidBodyState > (DEFAULT_CIRCULAR_BUFFER_SIZE);
    inertialSamples = boost::circular_buffer< ::base::samples::IMUSensors >(DEFAULT_CIRCULAR_BUFFER_SIZE);
    inertialState = boost::circular_buffer< rover_localization::InertialState >(DEFAULT_CIRCULAR_BUFFER_SIZE);

}

StateOptimize::StateOptimize(std::string const& name, RTT::ExecutionEngine* engine)
    : StateOptimizeBase(name, engine)
{
}

StateOptimize::~StateOptimize()
{
}
void StateOptimize::pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{
    /** A new sample arrived to the input port **/
    poseSamples.push_front(pose_samples_sample);
    counter.poseSamples++;

    #ifdef DEBUG_PRINTS
    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] Received new samples at "<<poseSamples[0].time.toMicroseconds()<<"\n";
    #endif

    /***************************/
    /** Filter Initialization **/
    /***************************/
    if(!initFilter)
    {
        /** Initialize if inertial state samples  **/
        if (counter.inertialState > 0)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"[STATE_OPTIMIZE INERTIAL_STATE] - "<<inertialState[0].time.toMicroseconds()<<" - Initializing Filter...";
            #endif

            /** Initialization of the filter with the first measurements **/
//            this->initStateOptimizeFilter (filter, poseSamples, inertialState);

            #ifdef DEBUG_PRINTS
            std::cout<<"[DONE]\n";
            #endif

            initFilter = true;
        }
    }

    /** Set the flag of the Pose values valid to true **/
    if (!flag.poseSamples && (poseSamples.size() == poseSamples.capacity()))
        flag.poseSamples = true;

    /** Reset counter in case of inconsistency **/
    if (counter.poseSamples > poseSamples.size())
        counter.poseSamples = 0;
}

void StateOptimize::inertial_stateTransformerCallback(const base::Time &ts, const ::rover_localization::InertialState &inertial_state_sample)
{
    /** A new sample arrived to the input port **/
    inertialState.push_front(inertial_state_sample);
    counter.inertialState++;

    #ifdef DEBUG_PRINTS
    std::cout<<"[STATE_OPTIMIZE INERTIAL-SAMPLES] Received new samples at "<<inertialState[0].time.toMicroseconds()<<"\n";
    std::cout<<"[STATE_OPTIMIZE INERTIAL-SAMPLES] counter.inertialState "<< counter.inertialState<<"\n";
    std::cout<<"[STATE_OPTIMIZE INERTIAL-SAMPLES] counter.poseSamples "<< counter.poseSamples<<"\n";
    #endif

    /***************************/
    /** Filter Initialization **/
    /***************************/
    if(!initFilter)
    {
        /** Initialize if pose samples  **/
        if (counter.poseSamples > 0)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] - "<<poseSamples[0].time.toMicroseconds()<<" - Initializing Filter...";
            #endif

            /** Initialization of the filter with the first measurements **/
//            this->initStateOptimizeFilter (filter, poseSamples, inertialState);

            #ifdef DEBUG_PRINTS
            std::cout<<"[DONE]\n";
            #endif

            initFilter = true;
        }
    }

    /** Set the flag to true **/
    if (!flag.inertialState && (inertialState.size() == inertialState.capacity()))
        flag.inertialState = true;

    /** Reset counter in case of inconsistency **/
    if (counter.inertialState > inertialState.size())
        counter.inertialState = 0;

}

void StateOptimize::inertial_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample)
{
    /** A new sample arrived to the input port **/
    inertialSamples.push_front(inertial_samples_sample);
    counter.inertialSamples++;

    if (counter.inertialSamples == number.inertialSamples)
    {
        flag.inertialSamples = true;
        counter.inertialSamples = 0;
    }
    else
        flag.inertialSamples = false;

    if (initFilter)
    {
        WSingleState statek_i; /** Current robot state (copy from the filter object) */
        WSingleState deltaStatek_i; /** Delta in robot state to propagate the state (fill with info coming from FrontEnd) */

        /**********************************************/
        /** Propagation of the navigation quantities **/
        /**********************************************/

        /** Get the current state **/
        statek_i = filter->muState().statek_i;

        #ifdef DEBUG_PRINTS
        std::cout<<"\n[STATE_OPTIMIZE POSE_SAMPLES] BEFORE_PROPAGATION \n";
        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] position:\n"<<statek_i.pos<<"\n";
        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] velocity:\n"<<statek_i.vel<<"\n";
        Eigen::Matrix <double,localization::NUMAXIS,1> eulerb; /** In Euler angles **/
        eulerb[2] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
        eulerb[1] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
        eulerb[0] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] Roll:"
            <<eulerb[0]*localization::R2D<<" Pitch:"
            <<eulerb[1]*localization::R2D<<" Yaw:"
            <<eulerb[2]*localization::R2D<<"\n";
        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] gbias:\n"<<statek_i.gbias<<"\n";
        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] abias:\n"<<statek_i.abias<<"\n\n";
        #endif

        /** Get the delta state **/
//        deltaStatek_i = this->deltaState (delta_t, statek_i, poseSamples, inertialSamples);

        /** Propagate the rover state (navigation quantities) **/
        statek_i = statek_i + deltaStatek_i.getVectorizedState(); filter->setStatek_i(statek_i);

        /** Get the current state (after propagation) **/
        statek_i = filter->muState().statek_i;

        #ifdef DEBUG_PRINTS
        std::cout<<"\n[STATE_OPTIMIZE POSE_SAMPLES] AFTER_PROPAGATION \n";
        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] position:\n"<<statek_i.pos<<"\n";
        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] velocity:\n"<<statek_i.vel<<"\n";
        Eigen::Matrix <double,localization::NUMAXIS,1> euler; /** In Euler angles **/
        euler[2] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
        euler[1] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
        euler[0] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] Roll:"
            <<euler[0]*localization::R2D<<" Pitch:"
            <<euler[1]*localization::R2D<<" Yaw:"
            <<euler[2]*localization::R2D<<"\n";
        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] gbias:\n"<<statek_i.gbias<<"\n";
        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] abias:\n"<<statek_i.abias<<"\n\n";
        #endif

        /*******************/
        /** Predict State **/
        /*******************/
//        this->statePredict(delta_t, statek_i, poseSamples, inertialState);

        /*******************/
        /** Update State **/
        /*******************/
//        this->attitudeAndVelocityUpdate(delta_t, statek_i, poseSamples, inertialState);

        /** Out port the information of the Back-End **/
//        this->outputPortSamples (filter, accModel, accInertial);

        /** Reset the error vector state and perform cloning */
//        filter->muErrorSingleReset();
//
//        if (flag.poseSamples && flag.inertialState)
//        {
//            double delta_t = (1.0/config.backend_frequency); /** Position correction delta time */
//
//            #ifdef DEBUG_PRINTS
//            std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] - "<<inertialState[0].time.toMicroseconds()<<" - Position correction@"<< delta_t <<" seconds\n";
//            std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] acc:\n"<<inertialState[0].acc<<"\n";
//            std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] gyro:\n"<<inertialState[0].gyro<<"\n";
//            #endif
//
//            /***************************/
//            /** Delay Position Update **/
//            /***************************/
//
//            /** Observation matrix H (measurement model) **/
//            Eigen::Matrix<double, 3, WAugmentedState::DOF> H;/** Observation matrix H (measurement model) */
//            H = localization::delayPositionMeasurementMatrix<WAugmentedState, WSingleState>();
//
//            /** Create the measurement and the covariance **/
//            typedef Eigen::Matrix<double, 3, 1> MeasurementVector;
//            localization::DataModel<double, 3> relativePos = this->relativePosition(frontEndPose);
//            MeasurementVector z = relativePos.data; /** Relative position measurement vector */
//
//            /** Update the filter **/
//            slipVector.data += filter->ekfUpdate <MeasurementVector,
//                       Eigen::Matrix<double, 3, WAugmentedState::DOF>,
//                       Eigen::Matrix<double, 3, 3> > (z, H, relativePos.Cov);
//
//            /** Calculate the uncertainty of the slip vector **/
//            slipVector.Cov += H * filter->PkAugmentedState() * H.transpose() +  relativePos.Cov;
//
//            #ifdef DEBUG_PRINTS
//            std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] SlipVector:\n"<<slipVector.data<<"\n";
//            std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] SlipVectorCov:\n"<<slipVector.Cov<<"\n";
//            #endif
//
//            filter->cloning();

    //        /** Get the corrected state **/
    //        statek_i = filter->muState().statek_i;
    //
    //        #ifdef DEBUG_PRINTS
    //        std::cout<<"\n[STATE_OPTIMIZE POSE_SAMPLES] AFTER_CORRECTION \n";
    //        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] position:\n"<<statek_i.pos<<"\n";
    //        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] velocity:\n"<<statek_i.vel<<"\n";
    //        euler[2] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
    //        euler[1] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
    //        euler[0] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
    //        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] Roll: "<<euler[0]*localization::R2D<<" Pitch: "<<euler[1]*localization::R2D<<" Yaw: "<<euler[2]*localization::R2D<<"\n";
    //        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] gbias:\n"<<statek_i.gbias<<"\n";
    //        std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] abias:\n"<<statek_i.abias<<"\n\n";
    //        #endif


            flag.reset();
        //}
    }

    /** CAUTION: check how if the rover velocity comes in the body or in the navigation frame **/

    return;
}

void StateOptimize::exteroceptive_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &exteroceptive_samples_sample)
{
    throw std::runtime_error("Transformer callback for exteroceptive_samples not implemented");
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See StateOptimize.hpp for more detailed
// documentation about them.

bool StateOptimize::configureHook()
{
    if (! StateOptimizeBase::configureHook())
        return false;

    /************************/
    /** Read configuration **/
    /************************/
    config = _configuration.value();
    inertialNoise = _inertial_noise.value();
    adaptiveConfig = _adaptive_config.value();

    /***********************/
    /** Configure Values  **/
    /***********************/

    /** Set the number of samples between each sensor input */
    if (config.input_frequency != 0.00 && config.delay_state_frequency != 0.00)
    {
        number.poseSamples = config.input_frequency/config.delay_state_frequency;
        number.inertialSamples =  config.input_frequency/config.delay_state_frequency;
        number.inertialState =  config.input_frequency/config.delay_state_frequency;
    }
    else
    {
        RTT::log(RTT::Warning)<<"[STATE_OPTIMIZE FATAL ERROR] Configuration frequency cannot be zero."<<RTT::endlog();
        return false;
    }

    /** Set the capacity of the circular_buffer according to the sampling rate **/
    poseSamples.set_capacity(number.poseSamples);
    inertialSamples.set_capacity(number.inertialSamples);
    inertialState.set_capacity(number.inertialState);

    #ifdef DEBUG_PRINTS
    std::cout<<"[STATE_OPTIMIZE CONFIGURE] poseSamples has capacity "<<poseSamples.capacity()<<" and size "<<poseSamples.size()<<"\n";
    std::cout<<"[STATE_OPTIMIZE CONFIGURE] inertialSamples has capacity "<<inertialSamples.capacity()<<" and size "<<inertialSamples.size()<<"\n";
    std::cout<<"[STATE_OPTIMIZE CONFIGURE] inertialState has capacity "<<inertialState.capacity()<<" and size "<<inertialState.size()<<"\n";
    #endif

    /** Create the class for the adaptive measurement update of the orientation **/
    adapAtt.reset (new localization::AdaptiveAttitudeCov (adaptiveConfig.M1, adaptiveConfig.M2, adaptiveConfig.gamma, adaptiveConfig.r2count));

    /** Create the class for the adaptive measurement update of the orientation **/
    adapAcc.reset (new localization::AdaptiveAttitudeCov (adaptiveConfig.M1, adaptiveConfig.M2, adaptiveConfig.gamma/3.0, adaptiveConfig.r2count));

    /** Set Slip vector to zero **/
    slipVector.data.setZero();
    slipVector.Cov.setZero();

    /***********************/
    /** Info and Warnings **/
    /***********************/
    RTT::log(RTT::Warning)<<"[Info] Frequency[Hertz]: "<<config.input_frequency<<RTT::endlog();

    if (config.input_frequency < config.delay_state_frequency)
    {
        RTT::log(RTT::Warning)<<"[STATE_OPTIMIZE FATAL] State delay frequency cannot be higher than Input frequency."<<RTT::endlog();
        return false;
    }

    return true;
}
bool StateOptimize::startHook()
{
    if (! StateOptimizeBase::startHook())
        return false;
    return true;
}
void StateOptimize::updateHook()
{
    StateOptimizeBase::updateHook();
}
void StateOptimize::errorHook()
{
    StateOptimizeBase::errorHook();
}
void StateOptimize::stopHook()
{
    StateOptimizeBase::stopHook();
}
void StateOptimize::cleanupHook()
{
    StateOptimizeBase::cleanupHook();

    /** Liberate the memory of the shared_ptr **/
    filter.reset();
    adapAtt.reset();
    adapAcc.reset();
}

inline WSingleState StateOptimize::deltaState (const double delta_t, const WSingleState &currentState,
        base::samples::RigidBodyState &pose,
        base::samples::RigidBodyState &delta_pose, base::samples::IMUSensors &inertialSamples)
{

    WSingleState delta_s;
    Eigen::Vector3d deltaVelocity;
    deltaVelocity = inertialSamples.acc * delta_t;
    deltaVelocity[2] = delta_pose.velocity[2];

    /** Robot's state estimate is propagated using the non-linear equation **/
    delta_s.pos = (pose.orientation * (currentState.vel + deltaVelocity)) * delta_t; //! Increment in position
    delta_s.vel = deltaVelocity; //! Increment in velocity using inertial sensors.
    delta_s.orient = static_cast< MTK::SO3<double> > (delta_pose.orientation); //! Delta in orientation

    #ifdef DEBUG_PRINTS
    std::cout<<"\n[STATE_OPTIMIZE POSE_SAMPLES] DELTA_STATE \n";
    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] position:\n"<<delta_s.pos<<"\n";
    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] velocity:\n"<<delta_s.vel<<"\n";
    Eigen::Matrix <double,localization::NUMAXIS,1> eulerdelta; /** In Euler angles **/
    eulerdelta[2] = delta_s.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
    eulerdelta[1] = delta_s.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
    eulerdelta[0] = delta_s.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] Roll:"
        <<eulerdelta[0]*localization::R2D<<" Pitch:"
        <<eulerdelta[1]*localization::R2D<<" Yaw:"
        <<eulerdelta[2]*localization::R2D<<"\n";
    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] Roll(rad): "<<eulerdelta[0]<<" Pitch(rad): "<<eulerdelta[1]<<" Yaw(rad): "<<eulerdelta[2]<<"\n";
    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] w: "<<delta_s.orient.w()<<" x: "<<delta_s.orient.x()<<" y: "<<delta_s.orient.y()<<" z:"<<delta_s.orient.z()<<"\n";
    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] gbias:\n"<<delta_s.gbias<<"\n";
    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] abias:\n"<<delta_s.abias<<"\n\n";
    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] DELTA_STATE in Vectorized_form:\n"<<delta_s.getVectorizedState(::localization::State::ERROR_QUATERNION)<<"\n\n";
    #endif

    return delta_s;

}

//inline void StateOptimize::statePredict(const double delta_t, const WSingleState &statek_i,
//        boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
//        boost::circular_buffer<rover_localization::InertialState> &inertialState)
//{
//    /** Form the process model matrix **/
//    typedef Eigen::Matrix<double, WSingleState::DOF, WSingleState::DOF> SingleStateProcessModelMatrix;
//    SingleStateProcessModelMatrix processModelF = localization::processModel
//                                            < WSingleState, SingleStateProcessModelMatrix>
//                                            (inertialState[0].gyro, inertialState[0].acc,
//                                            static_cast<const Eigen::Quaterniond&>(statek_i.orient), delta_t);
//
//    /** Form the process covariance matrix **/
//    typedef StateOptimizeFilter::SingleStateCovariance SingleStateCovariance;//! Typedef for the statek_i covariance type
//    SingleStateCovariance processCovQ;
//    processCovQ = localization::processNoiseCov
//                        <WSingleState, SingleStateCovariance > (processModelF, sensornoise.accrw, sensornoise.aresolut,
//                                                                sensornoise.gyrorw, sensornoise.gbiasins, sensornoise.abiasins,
//                                                                static_cast<const Eigen::Quaterniond&>(statek_i.orient), delta_t) ;
//
//    /** Robot's error state is propagated using the non-linear noise dynamic model (processModel) **/
//    filter->ekfPredict(processModelF, processCovQ);
//
//    return;
//}
//
//inline void StateOptimize::attitudeAndVelocityUpdate(const double delta_t, const WSingleState &statek_i,
//                boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
//                boost::circular_buffer<rover_localization::InertialState> &inertialState)
//{
//    /** Observation matrix H (measurement model) **/
//    Eigen::Matrix<double, 6, WSingleState::DOF> H;/** Observation matrix H (measurement model) */
//    H = ::localization::proprioceptiveMeasurementMatrix<WSingleState::DOF>(statek_i.orient, inertialState[0].theoretical_g);
//
//    /** Create the measurement **/
//    typedef Eigen::Matrix<double, 6, 1> MeasurementVector;
//    MeasurementVector z; /** Unified measurement vector (velocity and acceleration) */
//    localization::DataModel<double, 3> veloError = this->velocityError(frontEndPose, filter);
//
//    /** Fill the measurement vector **/
//    z.block<3,1>(0,0) = veloError.data;
//    z.block<3,1>(3,0) = inertialState[0].acc;
//
//    /** Form the measurement covariance matrix **/
//    Eigen::Matrix<double, 6, 6> measuCovR;
//    measuCovR = ::localization::proprioceptiveMeasurementNoiseCov(veloError.Cov,
//                                        sensornoise.accrw, sensornoise.aresolut, delta_t);
//
//    /** Adaptive part of the measurement covariance needs the covariance of the process  **/
//    StateOptimizeFilter::SingleStateCovariance Pksingle = filter->PkSingleState(); /** covariance of the single state */
//
//    /** Get the error state **/
//    WSingleState errork_i = filter->muError().statek_i;
//
//    /** Adaptive covariance matrix of the measurement (3x3 bottomRight matrix part) **/
//    measuCovR.bottomRightCorner<3,3>() = adapAtt->matrix<WSingleState::DOF>
//        (errork_i.getVectorizedState(::localization::State::ERROR_QUATERNION),
//         Pksingle, inertialState[0].acc,
//         H.block<3, WSingleState::DOF>(3,0),
//         measuCovR.bottomRightCorner<3,3>());
//
//
//    /** Perform the update in an EKF form **/
//    filter->ekfSingleUpdate < MeasurementVector,
//                        Eigen::Matrix<double,6, WSingleState::DOF>,
//                        Eigen::Matrix<double,6,6> > (z, H, measuCovR);
//
//    return;
//}
//
//void StateOptimize::initStateOptimizeFilter(boost::shared_ptr<StateOptimizeFilter> &filter, boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
//                boost::circular_buffer<rover_localization::InertialState> &inertialState)
//{
//    /** The filter vector state variables one for the navigation quantities the other for the error **/
//    WAugmentedState vstate;
//    WAugmentedState verror;
//
//    /************************************/
//    /** Initialize the Back-End Filter **/
//    /************************************/
//
//    /** Initial covariance matrix **/
//    StateOptimizeFilter::SingleStateCovariance P0single; /** Initial P(0) for one state **/
//    P0single.setZero();
//
//    MTK::setDiagonal (P0single, &WSingleState::pos, 1e-03);
//    MTK::setDiagonal (P0single, &WSingleState::vel, 1e-03);
//    MTK::setDiagonal (P0single, &WSingleState::orient, 1e-03);
//    MTK::setDiagonal (P0single, &WSingleState::gbias, 1e-10);
//    MTK::setDiagonal (P0single, &WSingleState::abias, 1e-10);
//
//
//    /** Initial covariance matrix for the Vector of States **/
//    StateOptimizeFilter::AugmentedStateCovariance P0; /** Initial P(0) for the whole Vector State **/
//
//    MTK::subblock (P0, &WAugmentedState::statek, &WAugmentedState::statek) = P0single;
//    MTK::subblock (P0, &WAugmentedState::statek_l, &WAugmentedState::statek_l) = P0single;
//    MTK::subblock (P0, &WAugmentedState::statek_i, &WAugmentedState::statek_i) = P0single;
//
//    MTK::subblock (P0, &WAugmentedState::statek, &WAugmentedState::statek_l) = P0single;
//    MTK::subblock (P0, &WAugmentedState::statek, &WAugmentedState::statek_i) = P0single;
//    MTK::subblock (P0, &WAugmentedState::statek_l, &WAugmentedState::statek) = P0single;
//    MTK::subblock (P0, &WAugmentedState::statek_i, &WAugmentedState::statek) = P0single;
//
//    MTK::subblock (P0, &WAugmentedState::statek_l, &WAugmentedState::statek_i) = P0single;
//    MTK::subblock (P0, &WAugmentedState::statek_i, &WAugmentedState::statek_l) = P0single;
//
//    /** Set the current pose to initialize the filter structure **/
//    vstate.statek_i.pos = frontEndPose[0].position; //!Initial position from kinematics
//    vstate.statek_i.vel = frontEndPose[0].velocity; //!Initial velocity from kinematics
//    vstate.statek_i.orient = static_cast< Eigen::Quaternion<double> >(frontEndPose[0].orientation);
//
//    /** Set the initial accelerometers and gyros bias offset in the state vector **/
//    vstate.statek_i.gbias = sensornoise.gbiasoff + inertialState[0].gbias_onoff; //!Initial gyros bias offset
//    vstate.statek_i.abias = sensornoise.abiasoff + inertialState[0].abias_onoff; //!Initial accelerometers bias offset
//
//    /** Copy the state in the vector of states **/
//    vstate.statek = vstate.statek_i;
//    vstate.statek_l = vstate.statek_i;
//
//    /** Set the initial error of the verror state (by default pos, vel are zero and orient is identity) **/
//    /**TO-DO: No needed in the vector state **/
//    //verror.statek_i.gbias = vstate.statek_i.gbias;
//    //verror.statek_i.abias = vstate.statek_i.abias;
//
//    /** Copy the error state in the vector of states **/
//    verror.statek = verror.statek_i;
//    verror.statek_l = verror.statek_i;
//
//    /** Create the filter **/
//    filter.reset (new StateOptimizeFilter (static_cast<const WAugmentedState> (vstate),
//                                    static_cast<const WAugmentedState> (verror),
//                                    static_cast<const StateOptimizeFilter::AugmentedStateCovariance> (P0)));
//
//    #ifdef DEBUG_PRINTS
//    std::cout<<"\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] Single P0|0 is of size " <<P0single.rows()<<" x "<<P0single.cols()<<"\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] Single P0|0:\n"<<P0single<<"\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] P0|0 is of size " <<P0.rows()<<" x "<<P0.cols()<<"\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] P0|0:\n"<<P0<<"\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] state:\n"<<vstate.getVectorizedState()<<"\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] position:\n"<<vstate.statek_i.pos<<"\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] velocity:\n"<<vstate.statek_i.vel<<"\n";
//    Eigen::Matrix <double,localization::NUMAXIS,1> euler; /** In Euler angles **/
//    euler[2] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
//    euler[1] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
//    euler[0] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] Roll: "<<euler[0]*localization::R2D<<" Pitch: "<<euler[1]*localization::R2D<<" Yaw: "<<euler[2]*localization::R2D<<"\n";
//    std::cout<<"\n";
//    #endif
//
//    return;
//}
//
//localization::DataModel<double, 3> StateOptimize::relativePosition(const boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose)
//{
//    localization::DataModel<double, 3> deltaPosition; /** Relative position **/
//
//    if (frontEndPose.size() > 1)
//    {
//        deltaPosition.data = frontEndPose[0].position - frontEndPose[1].position;
//        deltaPosition.Cov = frontEndPose[0].cov_position - frontEndPose[1].cov_position;
//    }
//    else
//    {
//        deltaPosition.data.setZero();
//        deltaPosition.Cov = frontEndPose[0].cov_position;
//    }
//
//    std::cout<<"frontEndPose[0].position\n"<<frontEndPose[0].position<<"\n";
//    std::cout<<"frontEndPose[0].cov_position\n"<<frontEndPose[0].cov_position<<"\n";
//
//    return deltaPosition;
//}
//
//localization::DataModel<double, 3> StateOptimize::velocityError(const boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
//                                                        const boost::shared_ptr< StateOptimizeFilter > filter)
//{
//    localization::DataModel<double, 3> veloError, stateVelocity, frontEndVelocity; /** Velocity **/
//
//    /** Store the velocity in the Data Model types **/
//    stateVelocity.data = filter->muState().statek_i.vel;
//    stateVelocity.Cov = filter->PkSingleState().block<3,3>(3,3);
//    if (localization::Util::isnotnan(frontEndPose[0].velocity))
//        frontEndVelocity.data = frontEndPose[0].velocity;
//    if (localization::Util::isnotnan(frontEndPose[0].cov_velocity))
//        frontEndVelocity.Cov = frontEndPose[0].cov_velocity;
//    else
//        frontEndVelocity.Cov.setZero();
//
//    /** Compute the error in velocity **/
//    veloError = frontEndVelocity - stateVelocity;
//
//    return veloError;
//}
//
//void StateOptimize::outputPortSamples (const boost::shared_ptr< localization::Usckf<WAugmentedState, WSingleState> > filter,
//                            const localization::DataModel<double, 3> &deltaVeloModel, const localization::DataModel<double, 3> &deltaVeloInertial)
//{
//    base::samples::RigidBodyState poseOut; /** Robot pose to port out **/
//    WSingleState statek_i; /** Current robot state */
//    rover_localization::StateOptimizeEstimation backEndEstimationSamples; /** Filter information **/
//    WAugmentedState errorAugmentedState = filter->muError();
//
//    //localization::DataModel<double, 3> deltaVeloCommon = deltaVeloInertial;
//
//    /** Fusion of deltaVeloModel with deltaVeloInertial **/
//    //deltaVeloCommon.Cov = deltaVeloCommon.Cov;
//    //deltaVeloCommon.fusion(deltaVeloModel);
//
//    /** Init **/
//    poseOut.invalidate();
//
//    /** Get the current robot pose from the filter **/
//    statek_i = filter->muState().statek_i;
//
//    /*******************************************/
//    /** Port out the Output Ports information **/
//    /*******************************************/
//
//    /** The Back-End Estimated pose **/
//    poseOut.time = poseSamples[0].time;
//    poseOut.position = statek_i.pos;
//    poseOut.cov_position = filter->PkSingleState().block<3,3>(0,0);
//    poseOut.velocity = statek_i.vel;
//    poseOut.cov_velocity = filter->PkSingleState().block<3,3>(3,3);
//    poseOut.orientation = statek_i.orient;
//    poseOut.cov_orientation = filter->PkSingleState().block<3,3>(6,6);
//    _pose_samples_out.write(poseOut);
//
//    /** Port out the estimated error in this sample interval **/
//
//
//    /** Port out the filter information **/
//    backEndEstimationSamples.time = poseSamples[0].time;
//    backEndEstimationSamples.statek_i = static_cast<WAugmentedState>(filter->muState()).statek_i.getVectorizedState();
//    backEndEstimationSamples.errork_i = static_cast<WAugmentedState>(errorAugmentedState).statek_i.getVectorizedState();
//    backEndEstimationSamples.orientation = filter->muState().statek_i.orient;
//    backEndEstimationSamples.Pki = filter->PkAugmentedState();
//    backEndEstimationSamples.gbias = filter->muState().statek_i.gbias;
//    backEndEstimationSamples.abias = filter->muState().statek_i.abias;
//    backEndEstimationSamples.accModel = deltaVeloModel.data;
//    backEndEstimationSamples.accModelCov = deltaVeloModel.Cov;
//    backEndEstimationSamples.accInertial = deltaVeloInertial.data;
//    backEndEstimationSamples.accInertialCov = deltaVeloInertial.Cov;
////    backEndEstimationSamples.accError = deltaVeloError.data;
////    backEndEstimationSamples.accErrorCov = deltaVeloError.Cov;
////    backEndEstimationSamples.Hellinger = Hellinger;
//    backEndEstimationSamples.mahalanobis = localization::Util::mahalanobis<double, 3> (deltaVeloModel, deltaVeloInertial);
//    backEndEstimationSamples.Threshold = (deltaVeloModel.Cov + deltaVeloInertial.Cov).inverse();
////    backEndEstimationSamples.deltaVeloCommon = deltaVeloCommon.data;
////    backEndEstimationSamples.deltaVeloCommonCov = deltaVeloCommon.Cov;
//    _backend_estimation_samples_out.write(backEndEstimationSamples);
//
//    return;
//}


