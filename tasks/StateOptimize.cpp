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

WSingleState processModel (const WSingleState &state,
                        const Eigen::Matrix<double, 3, 1> &delta_position,
                        const Eigen::Quaterniond &delta_orient)
{
    WSingleState s2;
    s2.orient = state.orient * delta_orient;
    s2.pos = state.pos + (state.orient * delta_position);
    s2.gbias = state.gbias;
    s2.abias = state.abias;

    return s2;
}

Eigen::Vector3d attitudeObservationModel (const WSingleState &state)
{
    Eigen::AngleAxis<double> orientation (state.orient);
    return orientation.angle() * orientation.axis();
}

Eigen::Vector3d delayObservationModel (const WAugmentedState &state)
{
    Eigen::Vector3d deltaPos = state.statek_i.pos - state.statek_l.pos;
    deltaPos[2] = 0.00; // Only X-Y plane observation
    #ifdef DEBUG_PRINTS
    std::cout<<"[DELAY_OBSERVATION] Position at statek_i: "<<state.statek_i.pos[0]<<" "<<state.statek_i.pos[1]<<" "<<state.statek_i.pos[2]<<"\n";
    std::cout<<"[DELAY_OBSERVATION] Position at statek_l: "<<state.statek_l.pos[0]<<" "<<state.statek_l.pos[1]<<" "<<state.statek_l.pos[2]<<"\n";
    std::cout<<"[DELAY_OBSERVATION] Delta Position: "<<deltaPos[0]<<" "<<deltaPos[1]<<" "<<deltaPos[2]<<"\n";
    #endif

    return deltaPos;
}

StateOptimize::StateOptimize(std::string const& name)
    : StateOptimizeBase(name)
{
    /******************************/
    /*** Control Flow Variables ***/
    /******************************/
    initFilter = false;

    /******************************************/
    /*** General Internal Storage Variables ***/
    /******************************************/
    slipVector.data = base::NaN<double>() * Eigen::Matrix<double, 3, 1>::Ones();
    slipVector.Cov = base::NaN<double>() * Eigen::Matrix<double, 3, 3>::Ones();

    /**************************/
    /** Input port variables **/
    /**************************/
    deltaPoseSamples.invalidate();
    inertialState.time.fromMicroseconds(base::NaN<uint64_t>());
    inertialState.abias_onoff =  base::NaN<uint64_t>() * Eigen::Vector3d::Ones();
    inertialState.gbias_onoff =  base::NaN<uint64_t>() * Eigen::Vector3d::Ones();
    inertialSamples.acc  = base::NaN<uint64_t>() * Eigen::Vector3d::Ones();
    inertialSamples.gyro  = base::NaN<uint64_t>() * Eigen::Vector3d::Ones();
    inertialSamples.mag  = base::NaN<uint64_t>() * Eigen::Vector3d::Ones();
    meanInertialSamples.acc.setZero();
    meanInertialSamples.gyro.setZero();
    meanInertialSamples.mag.setZero();
}

StateOptimize::StateOptimize(std::string const& name, RTT::ExecutionEngine* engine)
    : StateOptimizeBase(name, engine)
{
}

StateOptimize::~StateOptimize()
{
}

void StateOptimize::delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample)
{
    base::Time deltaTime = delta_pose_samples_sample.time - deltaPoseSamples.time;

    #ifdef DEBUG_PRINTS
    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] Received new samples at "<<delta_pose_samples_sample.time.toMicroseconds()<<"\n";
    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] deltaTime: "<<deltaTime.toSeconds()<<"\n";
    #endif

    /** A new sample arrived to the input port **/
    deltaPoseSamples = delta_pose_samples_sample;

    if (initFilter)
    {
        /*******************/
        /** Predict State **/
        /*******************/

        /** Process Model Uncertainty **/
        typedef StateOptimizeFilter::SingleStateCovariance SingleStateCovariance;
        SingleStateCovariance processCovQ;
        processCovQ = localization::processNoiseCov <WSingleState, SingleStateCovariance > (static_cast<const Eigen::Matrix3d>(deltaPoseSamples.cov_position),
                                                                                        static_cast<const Eigen::Matrix3d>(deltaPoseSamples.cov_orientation),
                                                                                        static_cast<const Eigen::Vector3d>(inertialnoise.gbiasins),
                                                                                        static_cast<const Eigen::Vector3d>(inertialnoise.abiasins));

        /** Predict the filter state **/
        filter->predict(boost::bind(processModel, _1 , static_cast<const Eigen::Vector3d>(deltaPoseSamples.position),
                                static_cast<const Eigen::Quaterniond>(deltaPoseSamples.orientation)),
                                processCovQ);

    }
}


void StateOptimize::inertial_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation in quaternion form **/

    /** Get the transformation **/
    if (!_world2navigation.get(ts, tf, false))
    {
       RTT::log(RTT::Fatal)<<"[FATAL ERROR]  No transformation provided."<<RTT::endlog();
       return;
    }

    qtf = Eigen::Quaternion <double> (Eigen::AngleAxisd(0.00,  Eigen::Vector3d::UnitZ())*
                                        Eigen::AngleAxisd(tf.rotation().eulerAngles(2,1,0)[1],  Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(tf.rotation().eulerAngles(2,1,0)[2],  Eigen::Vector3d::UnitX()));//! Pitch and Roll

    base::Time deltaTime = inertial_samples_sample.time - inertialSamples.time;

    #ifdef DEBUG_PRINTS
    std::cout<<"[STATE_OPTIMIZE INERTIAL_SAMPLES] Received new samples at "<<inertial_samples_sample.time.toMicroseconds()<<"\n";
    std::cout<<"[STATE_OPTIMIZE INERTIAL_SAMPLES] deltaTime: "<<deltaTime.toSeconds()<<"\n";
    std::cout<<"[STATE_OPTIMIZE INERTIAL_SAMPLES] acc(body_frame):\n"<<inertialSamples.acc<<"\n";
    std::cout<<"[STATE_OPTIMIZE INERTIAL_SAMPLES] gyro(body_frame):\n"<<inertialSamples.gyro<<"\n";
    std::cout<<"[STATE_OPTIMIZE INERTIAL_SAMPLES] incl(body_frame):\n"<<inertialSamples.mag<<"\n";
    #endif

    /** A new sample arrived to the input port **/
    inertialSamples = inertial_samples_sample;

    if (initFilter)
    {
        /** Get current state **/
        WSingleState statek_i = filter->muState().statek_i;

        /** Integrate acceleration and transform to the relative Frame **/
        meanInertialSamples.acc += statek_i.orient * inertialSamples.acc; //acc_nav = Tnav_body * acc_body

        /** Iterator **/
        delayIterations--;
        #ifdef DEBUG_PRINTS
        std::cout<<"[STATE_OPTIMIZE INERTIAL_SAMPLES] delayIterations["<<delayIterations<<"]\n";
        #endif

        this->attitudeUpdate(_inertial_samples_period, qtf);

        //if (delayIterations == 0)
        //{
        //    #ifdef DEBUG_PRINTS
        //    std::cout<<"[STATE_OPTIMIZE INERTIAL_SAMPLES] DELAY UPDATE\n";
        //    #endif

        //    double iterations = _delay_state_period.value()/_inertial_samples_period.value();
        //    meanInertialSamples.time = inertialSamples.time;
        //    meanInertialSamples.acc /= iterations;
        //    deltaPosMeasurement = meanInertialSamples.acc * _delay_state_period * _delay_state_period;
        //    deltaPosMeasurement[2] = 0.00;

        //    /******************/
        //    /** Update State **/
        //    /******************/
        //   // this->delayUpdate(deltaPosMeasurement, _delay_state_period);

        //    /** Debug ports **/
        //    this->outputDebugPortSamples(meanInertialSamples, inertialSamples.time);

        //    /** Perform the cloning statek_i -> statek_l **/
        //    filter->cloning(localization::STATEK_I);

        //    /** Reset the values **/
        //    meanInertialSamples.acc.setZero();
        //    meanInertialSamples.gyro.setZero();
        //    meanInertialSamples.mag.setZero();
        //    delayIterations = _delay_state_period.value()/_inertial_samples_period.value();

        //}

        this->outputPortSamples(inertialSamples.time);
    }

    return;
}

void StateOptimize::inertial_stateTransformerCallback(const base::Time &ts, const ::localization::InertialState &inertial_state_sample)
{
    /** A new sample arrived to the input port **/
    inertialState = inertial_state_sample;

    #ifdef DEBUG_PRINTS
    //std::cout<<"[STATE_OPTIMIZE INERTIAL_STATE] Received new samples at "<<inertialState.time.toMicroseconds()<<"\n";
    #endif

    /***************************/
    /** Filter Initialization **/
    /***************************/
    if(!initFilter)
    {
        Eigen::Affine3d tf; /** Transformer transformation **/

        /** Get the transformation **/
        if (absoluteFrame)
        {
            if (!_world2navigation.get(ts, tf, false))
            {
               RTT::log(RTT::Fatal)<<"[FATAL ERROR]  No transformation provided."<<RTT::endlog();
               return;
            }
        }
        else
        {
            tf.setIdentity();
        }

        #ifdef DEBUG_PRINTS
        std::cout<<"[STATE_OPTIMIZE INERTIAL_STATE] - Initializing Filter...";
        #endif

        /** Initialization of the filter with the first measurements **/
        this->initStateOptimizeFilter (filter, inertialState, tf);

        #ifdef DEBUG_PRINTS
        std::cout<<"[DONE]\n";
        #endif

        initFilter = true;
    }
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
    inertialnoise = _inertial_noise.value();
    adaptiveconfig = _adaptive_config.value();

    /***********************/
    /** Configure Values  **/
    /***********************/

    /** Create the class for the adaptive measurement update of the orientation **/
    adapAtt.reset (new localization::AdaptiveAttitudeCov (adaptiveconfig.M1, adaptiveconfig.M2, adaptiveconfig.gamma, adaptiveconfig.r2count));

    /** Set Slip vector to zero **/
    slipVector.data.setZero();
    slipVector.Cov.setZero();

    /** Delay iterations **/
    delayIterations = _delay_state_period/_inertial_samples_period;
    deltaPosMeasurement.setZero();
    deltaPosMeasurementCov.setZero();

    /** Relative Frame to port out the samples **/
    if (boost::iequals(_pose_samples_out_relative_frame.value(), "world"))
    {
        _pose_samples_out_relative_frame.set(_world_frame.value());
        absoluteFrame = true;
    }
    else
    {
        _pose_samples_out_relative_frame.set(_navigation_frame.value());
        absoluteFrame = false;
    }


    /***********************/
    /** Info and Warnings **/
    /***********************/
    if (_delay_state_period.value() < _delta_pose_samples_period.value())
    {
        throw std::runtime_error("[FATAL ERROR]: Delay State period cannot be smaller than delta pose samples period.");
    }

    if (_delay_state_period.value() < _inertial_samples_period.value())
    {
        throw std::runtime_error("[FATAL ERROR]: Delay State period cannot be smaller than inertial samples period.");
    }

    RTT::log(RTT::Warning)<<"[Info] Delay state period[seconds]: "<<_delay_state_period.value()<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Number of inertial_samples iterations: "<<delayIterations<<RTT::endlog();

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
}

//inline WSingleState StateOptimize::deltaState (const double delta_t, const WSingleState &currentState,
//        base::samples::RigidBodyState &pose,
//        base::samples::RigidBodyState &delta_pose, base::samples::IMUSensors &inertialSamples)
//{
//
//    WSingleState delta_s;
//    Eigen::Vector3d deltaVelocity;
//    deltaVelocity = inertialSamples.acc * delta_t;
//    deltaVelocity[2] = delta_pose.velocity[2];
//
//    /** Robot's state estimate is propagated using the non-linear equation **/
//    delta_s.pos = (pose.orientation * (currentState.vel + deltaVelocity)) * delta_t; //! Increment in position
//    delta_s.vel = deltaVelocity; //! Increment in velocity using inertial sensors.
//    delta_s.orient = static_cast< MTK::SO3<double> > (delta_pose.orientation); //! Delta in orientation
//
//    #ifdef DEBUG_PRINTS
//    std::cout<<"\n[STATE_OPTIMIZE POSE_SAMPLES] DELTA_STATE \n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] position:\n"<<delta_s.pos<<"\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] velocity:\n"<<delta_s.vel<<"\n";
//    Eigen::Matrix <double,localization::NUMAXIS,1> eulerdelta; /** In Euler angles **/
//    eulerdelta[2] = delta_s.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
//    eulerdelta[1] = delta_s.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
//    eulerdelta[0] = delta_s.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] Roll:"
//        <<eulerdelta[0]*localization::R2D<<" Pitch:"
//        <<eulerdelta[1]*localization::R2D<<" Yaw:"
//        <<eulerdelta[2]*localization::R2D<<"\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] Roll(rad): "<<eulerdelta[0]<<" Pitch(rad): "<<eulerdelta[1]<<" Yaw(rad): "<<eulerdelta[2]<<"\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] w: "<<delta_s.orient.w()<<" x: "<<delta_s.orient.x()<<" y: "<<delta_s.orient.y()<<" z:"<<delta_s.orient.z()<<"\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] gbias:\n"<<delta_s.gbias<<"\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] abias:\n"<<delta_s.abias<<"\n\n";
//    std::cout<<"[STATE_OPTIMIZE POSE_SAMPLES] DELTA_STATE in Vectorized_form:\n"<<delta_s.getVectorizedState(::localization::State::ERROR_QUATERNION)<<"\n\n";
//    #endif
//
//    return delta_s;
//
//}

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
//                        <WSingleState, SingleStateCovariance > (processModelF, inertialnoise.accrw, inertialnoise.aresolut,
//                                                                inertialnoise.gyrorw, inertialnoise.gbiasins, inertialnoise.abiasins,
//                                                                static_cast<const Eigen::Quaterniond&>(statek_i.orient), delta_t) ;
//
//    /** Robot's error state is propagated using the non-linear noise dynamic model (processModel) **/
//    filter->ekfPredict(processModelF, processCovQ);
//
//    return;
//}

inline void StateOptimize::delayUpdate(const Eigen::Vector3d &measurement, const double &delta_t)
{
    typedef Eigen::Matrix<double, WAugmentedState::DOF, 3> MeasurementCrossCovariance;
    typedef Eigen::Matrix<double, 3, WAugmentedState::DOF> ObservationMatrix;
    typedef Eigen::Matrix3d MeasurementNoiseCovariance;
    typedef Eigen::Matrix3d MeasurementModelCovariance;

    /** Create the measurement **/
    std::cout<<"Measurement(delta_pose):\n" <<measurement<<std::endl;

    /** Form the measurement covariance matrix **/
    MeasurementNoiseCovariance measuCovR = ::localization::deltaPosMeasurementNoiseCov(inertialnoise.accrw, inertialnoise.accresolut, delta_t);

    /** Copy to the global variable **/
    deltaPosMeasurementCov = measuCovR;

    /** Measurement observation covariance matrix **/
    StateOptimizeFilter::SingleStateCovariance Pk_i, Pk_l;
    Pk_i = filter->PkSingleState(localization::STATEK_I);
    Pk_l = filter->PkSingleState(localization::STATEK_L);
    std::cout<<"Pk_i is of size:" <<Pk_i.rows()<<"x"<<Pk_i.cols()<<std::endl;
    std::cout<<"Pk_i\n"<<Pk_i<<"\n";
    std::cout<<"Pk_l is of size:" <<Pk_l.rows()<<"x"<<Pk_l.cols()<<std::endl;
    std::cout<<"Pk_l\n"<<Pk_l<<"\n";
   MeasurementModelCovariance measuCovPzz = MTK::subblock (Pk_i, &WSingleState::pos) - MTK::subblock (Pk_l, &WSingleState::pos);

    /** Riccati-free version needs the covariance of the process  **/
    StateOptimizeFilter::AugmentedStateCovariance Pk = filter->PkAugmentedState();
    ObservationMatrix Hk = ::localization::delayPositionMeasurementMatrix<localization::AugmentedState, localization::State>();
    MeasurementCrossCovariance measuCovPxz = (Hk * Pk).transpose();


    /** Perform the update **/
    filter->delayUpdate(measurement, boost::bind(delayObservationModel, _1), measuCovPzz, measuCovPxz, measuCovR);

    return;
}

inline void StateOptimize::attitudeUpdate(const double &delta_t, const Eigen::Quaterniond &world2nav)
{
    if (inertialSamples.mag.norm() < 10.1)
    {

    /** Current filter state **/
    WSingleState statek_i = filter->muState().statek_i;

    /** Create the measurement **/
    typedef Eigen::Vector3d MeasurementVector;
    Eigen::Quaterniond world2body = attitudeMeasurement(static_cast<Eigen::Quaterniond>(statek_i.orient),
                                        static_cast<Eigen::Vector3d>(inertialSamples.mag)); //mag has inclinometers

    Eigen::AngleAxis<double> nav2body;
    if (absoluteFrame)
        nav2body = world2body;
    else
        nav2body = world2nav.inverse() * world2body;

    MeasurementVector z = nav2body.angle() * nav2body.axis();

    /** Form the measurement covariance matrix **/
    Eigen::Matrix<double, 3, 3> measuCovR;
    measuCovR = ::localization::proprioceptiveMeasurementNoiseCov(inertialnoise.accrw, inertialnoise.accresolut, delta_t);

    /** Adaptive part of the measurement covariance needs the covariance of the process  **/
    StateOptimizeFilter::SingleStateCovariance Pksingle = filter->PkSingleState(); /** covariance of the single state */

    /** Adaptive covariance matrix of the measurement **/
    //measuCovR = adapAtt->matrix<WSingleState::DOF> (errork_i, Pksingle, inertialSamples.acc, H, measuCovR);

    /** Perform the update **/
    filter->singleUpdate(z, boost::bind(attitudeObservationModel, _1), measuCovR);

    }
}

void StateOptimize::initStateOptimizeFilter(boost::shared_ptr<StateOptimizeFilter> &filter, localization::InertialState &inertialState, Eigen::Affine3d &tf)
{
    /** The filter vector state variables one for the navigation quantities the other for the error **/
    WAugmentedState vstate;

    /************************************/
    /** Initialize the Back-End Filter **/
    /************************************/

    /** Initial covariance matrix **/
    StateOptimizeFilter::SingleStateCovariance P0single; /** Initial P(0) for one state **/
    P0single.setZero();

    MTK::setDiagonal (P0single, &WSingleState::pos, 1e-03);
    MTK::setDiagonal (P0single, &WSingleState::orient, 1e-03);
    MTK::setDiagonal (P0single, &WSingleState::gbias, 1e-10);
    MTK::setDiagonal (P0single, &WSingleState::abias, 1e-10);


    /** Initial covariance matrix for the Vector of States **/
    StateOptimizeFilter::AugmentedStateCovariance P0; /** Initial P(0) for the whole Vector State **/

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

    /** Set the initial accelerometers and gyros bias offset in the state vector **/
    vstate.statek_i.gbias = inertialnoise.gbiasoff + inertialState.gbias_onoff; //!Initial gyros bias offset
    vstate.statek_i.abias = inertialnoise.abiasoff + inertialState.abias_onoff; //!Initial accelerometers bias offset

    /** Copy the state in the vector of states **/
    vstate.statek = vstate.statek_i;
    vstate.statek_l = vstate.statek_i;

    /** Create the filter **/
    filter.reset (new StateOptimizeFilter (static_cast<const WAugmentedState> (vstate),
                        static_cast<const StateOptimizeFilter::AugmentedStateCovariance> (P0)));

    #ifdef DEBUG_PRINTS
    std::cout<<"\n";
    std::cout<<"[STATE_OPTIMIZE INIT] Single P0|0 is of size " <<P0single.rows()<<" x "<<P0single.cols()<<"\n";
    std::cout<<"[STATE_OPTIMIZE INIT] Single P0|0:\n"<<P0single<<"\n";
    std::cout<<"[STATE_OPTIMIZE INIT] P0|0 is of size " <<P0.rows()<<" x "<<P0.cols()<<"\n";
    std::cout<<"[STATE_OPTIMIZE INIT] P0|0:\n"<<P0<<"\n";
    std::cout<<"[STATE_OPTIMIZE INIT] state:\n"<<vstate.getVectorizedState()<<"\n";
    std::cout<<"[STATE_OPTIMIZE INIT] position:\n"<<vstate.statek_i.pos<<"\n";
    Eigen::Matrix <double,localization::NUMAXIS,1> euler; /** In Euler angles **/
    euler[2] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
    euler[1] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
    euler[0] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
    std::cout<<"[STATE_OPTIMIZE INIT] Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
    std::cout<<"\n";
    #endif

    return;
}

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

inline Eigen::Quaterniond StateOptimize::attitudeMeasurement (const Eigen::Quaterniond &orient, const Eigen::Vector3d &acc)
{
    Eigen::Quaterniond attitude;
    Eigen::Vector3d euler;

    euler[0] = (double) asin((double)acc[1]/ (double)acc.norm()); // Roll
    euler[1] = (double) -atan(acc[0]/acc[2]); //Pitch
    euler[2] = orient.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW

    attitude = Eigen::AngleAxis<double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));

    std::cout<< "******** [ATTITUDE_MEASUREMENT]\n";
    std::cout<< " Roll: "<<euler[0]*180.00/M_PI<<" Pitch: "<<euler[1]*180.00/M_PI<<" Yaw: "<<euler[2]*180.00/M_PI<<"\n";

    return attitude;
}


void StateOptimize::outputPortSamples(const base::Time &timestamp)
{
    base::samples::RigidBodyState poseOut; poseOut.invalidate();
    WSingleState statek_i = filter->muState().statek_i;

    /** Estimated Pose **/
    poseOut.time = timestamp;
    if (absoluteFrame)
        poseOut.sourceFrame = "world";
    else
        poseOut.sourceFrame = "navigation";
    poseOut.targetFrame = "body";
    poseOut.position = statek_i.pos;
    poseOut.cov_position = filter->PkSingleState().block<3,3>(0,0);
    poseOut.orientation = statek_i.orient;
    poseOut.cov_orientation = filter->PkSingleState().block<3,3>(3,3);
    poseOut.velocity = deltaPoseSamples.velocity;
    poseOut.cov_velocity = deltaPoseSamples.cov_velocity;
    _pose_samples_out.write(poseOut);

    /** Gyroscopes bias **/
    base::Vector3d gyrobias = static_cast<base::Vector3d>(statek_i.gbias);
    _gyroscopes_bias_out.write(gyrobias);

    /** Accelerometers bias **/
    base::Vector3d accbias = static_cast<base::Vector3d>(statek_i.abias);
    _accelerometers_bias_out.write(accbias);

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

}

void StateOptimize::outputDebugPortSamples(base::samples::IMUSensors &inertial, const base::Time &timestamp)
{
    base::samples::RigidBodyState odo_deltaPoseOut, acc_deltaPoseOut;
    odo_deltaPoseOut.invalidate();
    WSingleState statek_i = filter->muState().statek_i;
    WSingleState statek_l = filter->muState().statek_l;
    StateOptimizeFilter::SingleStateCovariance Pk_i, Pk_l;
    Pk_i = filter->PkSingleState(localization::STATEK_I);
    Pk_l = filter->PkSingleState(localization::STATEK_L);

    /** Set Body State delta Pose **/
    odo_deltaPoseOut.time = timestamp;
    odo_deltaPoseOut.sourceFrame = "body_k-1";
    odo_deltaPoseOut.targetFrame = "body";
    acc_deltaPoseOut = odo_deltaPoseOut;

    /** Delta Pose delta **/
    odo_deltaPoseOut.position = statek_i.pos - statek_l.pos;
    std::cout<<"[STATE_OPTIMIZE DEBUG_SAMPLES] DELTA POSITION (State):\n"<< odo_deltaPoseOut.position<<"\n";
    odo_deltaPoseOut.cov_position = MTK::subblock (Pk_i, &WSingleState::pos) - MTK::subblock (Pk_l, &WSingleState::pos);
    _odo_delta_pose_samples_out.write(odo_deltaPoseOut);

    /** Delta Pose delta **/
    acc_deltaPoseOut.position = deltaPosMeasurement;
    std::cout<<"[STATE_OPTIMIZE DEBUG_SAMPLES] DELTA POSITION (Acc):\n"<< deltaPosMeasurement<<"\n";
    acc_deltaPoseOut.cov_position = deltaPosMeasurementCov;
    _acc_delta_pose_samples_out.write(acc_deltaPoseOut);

    /** Inertial Samples in the relative frame **/
    base::samples::IMUSensors inertialOut = inertial;
    std::cout<<"[STATE_OPTIMIZE DEBUG_SAMPLES] MEAN Acc:\n"<< inertial.acc<<"\n";
    _inertial_samples_out.write(inertialOut);
}



