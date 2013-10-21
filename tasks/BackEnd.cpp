/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BackEnd.hpp"

//#define DEBUG_PRINTS 1

using namespace rover_localization;

BackEnd::BackEnd(std::string const& name)
    : BackEndBase(name)
{
    /******************************/
    /*** Control Flow Variables ***/
    /******************************/
    initFilter = false;
    number.reset();
    counter.reset();
    flag.reset();
    adapValues.reset();

    /******************************************/
    /*** General Internal Storage Variables ***/
    /******************************************/
    frontEndPose =  boost::circular_buffer<base::samples::RigidBodyState> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    inertialState = boost::circular_buffer<rover_localization::InertialState>(DEFAULT_CIRCULAR_BUFFER_SIZE);
    slipVector.data = base::NaN<double>() * Eigen::Matrix<double, 3, 1>::Ones();
    slipVector.Cov = base::NaN<double>() * Eigen::Matrix<double, 3, 3>::Ones();

    /**************************/
    /** Input port variables **/
    /**************************/
    frontEndPoseSamples = boost::circular_buffer<base::samples::RigidBodyState> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    inertialStateSamples = boost::circular_buffer<rover_localization::InertialState> (DEFAULT_CIRCULAR_BUFFER_SIZE);

}

BackEnd::BackEnd(std::string const& name, RTT::ExecutionEngine* engine)
    : BackEndBase(name, engine)
{
}

BackEnd::~BackEnd()
{
}
void BackEnd::pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{
    /** A new sample arrived to the input port **/
    frontEndPoseSamples.push_front(pose_samples_sample);
    counter.frontEndPoseSamples++;

    #ifdef DEBUG_PRINTS
    std::cout<<"[BACK-END POSE-SAMPLES] Received new samples at "<<frontEndPoseSamples[0].time.toMicroseconds()<<"\n";
    #endif

    /** Set the flag of Front-End Pose values valid to true **/
    if (!flag.frontEndPoseSamples && (frontEndPoseSamples.size() == frontEndPoseSamples.capacity()))
        flag.frontEndPoseSamples = true;

    /** Reset counter in case of inconsistency **/
    if (counter.frontEndPoseSamples > frontEndPoseSamples.size())
        counter.frontEndPoseSamples = 0;
}

void BackEnd::inertial_samplesTransformerCallback(const base::Time &ts, const ::rover_localization::InertialState &inertial_samples_sample)
{
    /** A new sample arrived to the input port **/
    inertialStateSamples.push_front(inertial_samples_sample);
    counter.inertialStateSamples++;

    #ifdef DEBUG_PRINTS
    std::cout<<"[BACK-END INERTIAL-SAMPLES] Received new samples at "<<inertialStateSamples[0].time.toMicroseconds()<<"\n";
    std::cout<<"[BACK-END INERTIAL-SAMPLES] counter.inertialStateSamples "<< counter.inertialStateSamples<<"\n";
    std::cout<<"[BACK-END INERTIAL-SAMPLES] counter.frontEndPoseSamples "<< counter.frontEndPoseSamples<<"\n";
    #endif

    if (counter.inertialStateSamples == number.inertialStateSamples)
    {
        flag.inertialStateSamples = true;
        counter.inertialStateSamples = 0;
    }
    else
        flag.inertialStateSamples = false;


    /***************************/
    /** Filter Initialization **/
    /***************************/
    if(!initFilter)
    {
        /** It also needs at least one sample from the Front-End Odometry **/
        if (counter.frontEndPoseSamples > 0)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"[BACK-END POSE-SAMPLES] - "<<inertialState[0].time.toMicroseconds()<<" - Initializing Filter...";
            #endif

            /** Initialization of the Back-End Filter with the first measurements **/
            this->initBackEndFilter (filter, frontEndPoseSamples, inertialStateSamples);

            #ifdef DEBUG_PRINTS
            std::cout<<"[DONE]\n";
            #endif

            initFilter = true;
        }
    }
    else
    {
        WSingleState statek_i; /** Current robot state (copy from the filter object) */
        WSingleState deltaStatek_i; /** Delta in robot state to propagate the state (fill with info coming from FrontEnd) */
        double delta_t = (1.0/framework.frontend_frequency); /** Delta integration time */

        std::cout<<"[BACK-END POSE-SAMPLES] - Performing Filter@"<< delta_t <<" seconds\n";

        /**********************************************/
        /** Propagation of the navigation quantities **/
        /**********************************************/

        /** Get the current state **/
        statek_i = filter->muState().statek_i;

        #ifdef DEBUG_PRINTS
        std::cout<<"\n[BACK-END POSE-SAMPLES] BEFORE_PROPAGATION \n";
        std::cout<<"[BACK-END POSE-SAMPLES] position:\n"<<statek_i.pos<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] velocity:\n"<<statek_i.vel<<"\n";
        Eigen::Matrix <double,localization::NUMAXIS,1> eulerb; /** In euler angles **/
        eulerb[2] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
        eulerb[1] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
        eulerb[0] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
        std::cout<<"[BACK-END POSE-SAMPLES] Roll:"
            <<eulerb[0]*localization::R2D<<" Pitch:"
            <<eulerb[1]*localization::R2D<<" Yaw:"
            <<eulerb[2]*localization::R2D<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] gbias:\n"<<statek_i.gbias<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] abias:\n"<<statek_i.abias<<"\n\n";
        #endif

        /** Get the delta with the information coming from the Front-End **/
        deltaStatek_i = this->deltaState (delta_t, statek_i, frontEndPoseSamples, inertialStateSamples);

        /** Propagate the rover state (navigation quantities) **/
        statek_i = statek_i + deltaStatek_i.getVectorizedState(); filter->setStatek_i(statek_i);

        /** Get the current state (after propagation) **/
        statek_i = filter->muState().statek_i;

        #ifdef DEBUG_PRINTS
        std::cout<<"\n[BACK-END POSE-SAMPLES] AFTER_PROPAGATION \n";
        std::cout<<"[BACK-END POSE-SAMPLES] position:\n"<<statek_i.pos<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] velocity:\n"<<statek_i.vel<<"\n";
        Eigen::Matrix <double,localization::NUMAXIS,1> euler; /** In Euler angles **/
        euler[2] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
        euler[1] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
        euler[0] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
        std::cout<<"[BACK-END POSE-SAMPLES] Roll:"
            <<euler[0]*localization::R2D<<" Pitch:"
            <<euler[1]*localization::R2D<<" Yaw:"
            <<euler[2]*localization::R2D<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] gbias:\n"<<statek_i.gbias<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] abias:\n"<<statek_i.abias<<"\n\n";
        #endif

        /*******************/
        /** Predict State **/
        /*******************/
        this->statePredict(delta_t, statek_i, frontEndPoseSamples, inertialStateSamples);

        /*******************/
        /** Update State **/
        /*******************/
        this->attitudeAndVelocityUpdate(delta_t, statek_i, frontEndPoseSamples, inertialStateSamples);

        /** Out port the information of the Back-End **/
        this->outputPortSamples (filter, accModel, accInertial);

        /** Reset the error vector state and perform cloning */
        filter->muErrorSingleReset();

    }


    if (flag.frontEndPoseSamples && flag.inertialStateSamples)
    {
        double delta_t = (1.0/framework.backend_frequency); /** Position correction delta time */

        /** Get the correct values from the input port at the desired Back-End frequency **/
        this->inputPortSamples(frontEndPose, inertialState);

        #ifdef DEBUG_PRINTS
        std::cout<<"[BACK-END POSE-SAMPLES] - "<<inertialState[0].time.toMicroseconds()<<" - Position correction@"<< delta_t <<" seconds\n";
        std::cout<<"[BACK-END POSE-SAMPLES] acc:\n"<<inertialState[0].acc<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] gyro:\n"<<inertialState[0].gyro<<"\n";
        #endif

        /***************************/
        /** Delay Position Update **/
        /***************************/

        /** Observation matrix H (measurement model) **/
        Eigen::Matrix<double, 3, WAugmentedState::DOF> H;/** Observation matrix H (measurement model) */
        H = localization::delayPositionMeasurementMatrix<WAugmentedState, WSingleState>();
        /** Create the measurement and the covariance **/
        typedef Eigen::Matrix<double, 3, 1> MeasurementVector;
        localization::DataModel<double, 3> relativePos = this->relativePosition(frontEndPose);
        MeasurementVector z = relativePos.data; /** Relative position measurement vector */

        /** Update the filter **/
        slipVector.data += filter->ekfUpdate <MeasurementVector,
                   Eigen::Matrix<double, 3, WAugmentedState::DOF>,
                   Eigen::Matrix<double, 3, 3> > (z, H, relativePos.Cov);

        /** Calculate the uncertainty of the slip vector **/
        slipVector.Cov += H * filter->PkAugmentedState() * H.transpose() +  relativePos.Cov;

        #ifdef DEBUG_PRINTS
        std::cout<<"[BACK-END POSE-SAMPLES] SlipVector:\n"<<slipVector.data<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] SlipVectorCov:\n"<<slipVector.Cov<<"\n";
        #endif

        filter->cloning();

//        /** Get the corrected state **/
//        statek_i = filter->muState().statek_i;
//
//        #ifdef DEBUG_PRINTS
//        std::cout<<"\n[BACK-END POSE-SAMPLES] AFTER_CORRECTION \n";
//        std::cout<<"[BACK-END POSE-SAMPLES] position:\n"<<statek_i.pos<<"\n";
//        std::cout<<"[BACK-END POSE-SAMPLES] velocity:\n"<<statek_i.vel<<"\n";
//        euler[2] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
//        euler[1] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
//        euler[0] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
//        std::cout<<"[BACK-END POSE-SAMPLES] Roll: "<<euler[0]*localization::R2D<<" Pitch: "<<euler[1]*localization::R2D<<" Yaw: "<<euler[2]*localization::R2D<<"\n";
//        std::cout<<"[BACK-END POSE-SAMPLES] gbias:\n"<<statek_i.gbias<<"\n";
//        std::cout<<"[BACK-END POSE-SAMPLES] abias:\n"<<statek_i.abias<<"\n\n";
//        #endif


        flag.reset();
    }

    /** Reset counter in case of inconsistency **/
    if (counter.inertialStateSamples > inertialStateSamples.size())
        counter.inertialStateSamples = 0;

    /** CAUTION: check how if the rover velocity  comes in the body or in the world frame **/

}

void BackEnd::exteroceptive_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &exteroceptive_samples_sample)
{
    throw std::runtime_error("Transformer callback for exteroceptive_samples not implemented");
}

void BackEnd::update_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &update_samples_sample)
{
    throw std::runtime_error("Transformer callback for update_samples not implemented");
}

void BackEnd::visual_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &visual_samples_sample)
{
    throw std::runtime_error("Transformer callback for visual_samples not implemented");
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BackEnd.hpp for more detailed
// documentation about them.

bool BackEnd::configureHook()
{
    if (! BackEndBase::configureHook())
        return false;

    /************************/
    /** Read configuration **/
    /************************/
    sensornoise = _proprioceptive_sensors.value();
    framework = _framework.value();
    adapValues = _adaptive_filter.value();

    /***********************/
    /** Configure Values  **/
    /***********************/

    /** Set the number of samples between each sensor input */
    /** It depends on the different between frontend and backend frequency **/
    if (framework.frontend_frequency != 0.00 && framework.backend_frequency != 0.00)
    {
        number.frontEndPoseSamples = framework.frontend_frequency/framework.backend_frequency;
        number.inertialStateSamples =  framework.frontend_frequency/framework.backend_frequency;
    }
    else
    {
        RTT::log(RTT::Warning)<<"[BACK-END FATAL ERROR]  Front-End/Back-End frequency cannot be zero."<<RTT::endlog();
        return false;
    }

    /** Set the capacity of the circular_buffer according to the sampling rate **/
    frontEndPoseSamples.set_capacity(number.frontEndPoseSamples);
    inertialStateSamples.set_capacity(number.inertialStateSamples);

    #ifdef DEBUG_PRINTS
    std::cout<<"[BACK-END CONFIGURE] frontEndPoseSamples has capacity "<<frontEndPoseSamples.capacity()<<" and size "<<frontEndPoseSamples.size()<<"\n";
    std::cout<<"[BACK-END CONFIGURE] inertialStateSamples has capacity "<<inertialStateSamples.capacity()<<" and size "<<inertialStateSamples.size()<<"\n";
    std::cout<<"[BACK-END CONFIGURE] frontEndPose has capacity "<<frontEndPose.capacity()<<" and size "<<frontEndPose.size()<<"\n";
    std::cout<<"[BACK-END CONFIGURE] inertialState has capacity "<<inertialState.capacity()<<" and size "<<inertialState.size()<<"\n";
    #endif

    /** Set the delta time for the noise calculation */
    /*  To choose the maximum between sensor bandwidth and backend delta time **/
    double delta_bandwidth = (1.0/sensornoise.bandwidth); /** Bandwidth delta interval */
    double delta_backend = (1.0/framework.backend_frequency); /** Back-End delta interval */

    if (delta_backend > delta_bandwidth)
        this->delta_noise = sqrt(delta_backend);
    else
        this->delta_noise = sqrt(delta_bandwidth);

    /** Create the class for the adaptive measurement update of the orientation **/
    adapAtt.reset (new localization::AdaptiveAttitudeCov (adapValues.M1, adapValues.M2, adapValues.gamma, adapValues.r2count));

    /** Create the class for the adaptive measurement update of the orientation **/
    adapAcc.reset (new localization::AdaptiveAttitudeCov (adapValues.M1, adapValues.M2, adapValues.gamma/3.0, adapValues.r2count));

    /** Set Slip vector to zero **/
    slipVector.data.setZero();
    slipVector.Cov.setZero();

    /*******************************************/
    /** Info and Warnings about the Framework **/
    /*******************************************/
    RTT::log(RTT::Warning)<<"[Info Back-End] Back-End running at Frequency[Hertz]: "<<framework.backend_frequency<<RTT::endlog();

    if (framework.frontend_frequency < framework.backend_frequency)
    {
        RTT::log(RTT::Warning)<<"[BACK-END FATAL ERROR]  Back-End frequency cannot be higher than Front-End frequency."<<RTT::endlog();
        return false;
    }

    return true;
}
bool BackEnd::startHook()
{
    if (! BackEndBase::startHook())
        return false;
    return true;
}
void BackEnd::updateHook()
{
    BackEndBase::updateHook();
}
void BackEnd::errorHook()
{
    BackEndBase::errorHook();
}
void BackEnd::stopHook()
{
    BackEndBase::stopHook();
}
void BackEnd::cleanupHook()
{
    BackEndBase::cleanupHook();

    /** Liberate the memory of the shared_ptr **/
    filter.reset();
    adapAtt.reset();
    adapAcc.reset();
}

inline WSingleState BackEnd::deltaState (const double delta_t, const WSingleState &currentState,
        boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
        boost::circular_buffer<rover_localization::InertialState> &inertialState)
{

    WSingleState delta_s;
    Eigen::Vector3d deltaVelocity;
    deltaVelocity = inertialState[0].delta_vel;
    deltaVelocity[2] = frontEndPose[0].velocity[2] - frontEndPose[1].velocity[2];

    /** Robot's state estimate is propagated using the non-linear equation **/
    delta_s.pos = (frontEndPose[0].orientation * (currentState.vel + inertialState[0].delta_vel)) * delta_t; //! Increment in position
    delta_s.vel = deltaVelocity; //! Increment in velocity using inertial sensors.
    delta_s.orient = static_cast< MTK::SO3<double> > (inertialState[0].delta_orientation); //! Delta in orientation coming from Front End

    #ifdef DEBUG_PRINTS
    std::cout<<"\n[BACK-END POSE-SAMPLES] DELTA_STATE \n";
    std::cout<<"[BACK-END POSE-SAMPLES] position:\n"<<delta_s.pos<<"\n";
    std::cout<<"[BACK-END POSE-SAMPLES] velocity:\n"<<delta_s.vel<<"\n";
    Eigen::Matrix <double,localization::NUMAXIS,1> eulerdelta; /** In Euler angles **/
    eulerdelta[2] = delta_s.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
    eulerdelta[1] = delta_s.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
    eulerdelta[0] = delta_s.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
    std::cout<<"[BACK-END POSE-SAMPLES] Roll:"
        <<eulerdelta[0]*localization::R2D<<" Pitch:"
        <<eulerdelta[1]*localization::R2D<<" Yaw:"
        <<eulerdelta[2]*localization::R2D<<"\n";
    std::cout<<"[BACK-END POSE-SAMPLES] Roll(rad): "<<eulerdelta[0]<<" Pitch(rad): "<<eulerdelta[1]<<" Yaw(rad): "<<eulerdelta[2]<<"\n";
    std::cout<<"[BACK-END POSE-SAMPLES] w: "<<delta_s.orient.w()<<" x: "<<delta_s.orient.x()<<" y: "<<delta_s.orient.y()<<" z:"<<delta_s.orient.z()<<"\n";
    std::cout<<"[BACK-END POSE-SAMPLES] gbias:\n"<<delta_s.gbias<<"\n";
    std::cout<<"[BACK-END POSE-SAMPLES] abias:\n"<<delta_s.abias<<"\n\n";
    std::cout<<"[BACK-END POSE-SAMPLES] DELTA_STATE in Vectorized_form:\n"<<delta_s.getVectorizedState(::localization::State::ERROR_QUATERNION)<<"\n\n";
    #endif

    return delta_s;

}

inline void BackEnd::statePredict(const double delta_t, const WSingleState &statek_i,
        boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
        boost::circular_buffer<rover_localization::InertialState> &inertialState)
{
    /** Form the process model matrix **/
    typedef Eigen::Matrix<double, WSingleState::DOF, WSingleState::DOF> SingleStateProcessModelMatrix;
    SingleStateProcessModelMatrix processModelF = localization::processModel
                                            < WSingleState, SingleStateProcessModelMatrix>
                                            (inertialState[0].gyro, inertialState[0].acc,
                                            static_cast<const Eigen::Quaterniond&>(statek_i.orient), delta_t);

    /** Form the process covariance matrix **/
    typedef BackEndFilter::SingleStateCovariance SingleStateCovariance;//! Typedef for the statek_i covariance type
    SingleStateCovariance processCovQ;
    processCovQ = localization::processNoiseCov
                        <WSingleState, SingleStateCovariance > (processModelF, sensornoise.accrw, sensornoise.aresolut,
                                                                sensornoise.gyrorw, sensornoise.gbiasins, sensornoise.abiasins,
                                                                static_cast<const Eigen::Quaterniond&>(statek_i.orient), delta_t) ;

    /** Robot's error state is propagated using the non-linear noise dynamic model (processModel) **/
    filter->ekfPredict(processModelF, processCovQ);

    return;
}

inline void BackEnd::attitudeAndVelocityUpdate(const double delta_t, const WSingleState &statek_i,
                boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
                boost::circular_buffer<rover_localization::InertialState> &inertialState)
{
    /** Observation matrix H (measurement model) **/
    Eigen::Matrix<double, 6, WSingleState::DOF> H;/** Observation matrix H (measurement model) */
    H = ::localization::proprioceptiveMeasurementMatrix<WSingleState::DOF>(statek_i.orient, inertialState[0].theoretical_g);

    /** Create the measurement **/
    typedef Eigen::Matrix<double, 6, 1> MeasurementVector;
    MeasurementVector z; /** Unified measurement vector (velocity and acceleration) */
    localization::DataModel<double, 3> veloError = this->velocityError(frontEndPose, filter);

    /** Fill the measurement vector **/
    z.block<3,1>(0,0) = veloError.data;
    z.block<3,1>(3,0) = inertialState[0].acc;

    /** Form the measurement covariance matrix **/
    Eigen::Matrix<double, 6, 6> measuCovR;
    measuCovR = ::localization::proprioceptiveMeasurementNoiseCov(veloError.Cov,
                                        sensornoise.accrw, sensornoise.aresolut, delta_t);

    /** Adaptive part of the measurement covariance needs the covariance of the process  **/
    BackEndFilter::SingleStateCovariance Pksingle = filter->PkSingleState(); /** covariance of the single state */

    /** Get the error state **/
    WSingleState errork_i = filter->muError().statek_i;

    /** Adaptive covariance matrix of the measurement (3x3 bottomRight matrix part) **/
    measuCovR.bottomRightCorner<3,3>() = adapAtt->matrix<WSingleState::DOF>
        (errork_i.getVectorizedState(::localization::State::ERROR_QUATERNION),
         Pksingle, inertialState[0].acc,
         H.block<3, WSingleState::DOF>(3,0),
         measuCovR.bottomRightCorner<3,3>());


    /** Perform the update in an EKF form **/
    filter->ekfSingleUpdate < MeasurementVector,
                        Eigen::Matrix<double,6, WSingleState::DOF>,
                        Eigen::Matrix<double,6,6> > (z, H, measuCovR);

    return;
}

void BackEnd::inputPortSamples(boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
                               boost::circular_buffer<rover_localization::InertialState> &inertialState)
{
    unsigned int frontEndPoseSamplesSize = frontEndPoseSamples.size();
    unsigned int inertialStateSamplesSize = inertialStateSamples.size();
    base::samples::RigidBodyState pose; /** Local variable for frontEndPose */
    rover_localization::InertialState inertial;/** Local variable for inertial sensor sample */

    Eigen::Quaterniond delta_quaternion; /** Accumulation of delta in orientation along the buffers */

    /** Delta quaternion to identity **/
    delta_quaternion.setIdentity();

    #ifdef DEBUG_PRINTS
    std::cout<<"[BACK-END GET_INPORT] frontEndPoseSamples has capacity "<<frontEndPoseSamples.capacity()<<" and size "<<frontEndPoseSamples.size()<<"\n";
    std::cout<<"[BACK-END GET_INPORT] inertialStateSamples has capacity "<<inertialStateSamples.capacity()<<" and size "<<inertialStateSamples.size()<<"\n";
    #endif

    /** Set the position coming from the Front-End **/
    pose = frontEndPoseSamples[0];
    pose.velocity.setZero();
    pose.cov_velocity.setZero();

    /** Process the buffer for velocity average **/
    for (register unsigned int i = 0; i<frontEndPoseSamplesSize; ++i)
    {
        pose.velocity += frontEndPoseSamples[i].velocity;
        pose.cov_velocity += frontEndPoseSamples[i].cov_velocity;
    }

    /** Velocity average over the Back-End time interval **/
    pose.velocity /= frontEndPoseSamplesSize;
    pose.cov_velocity /= frontEndPoseSamplesSize;

    /** Push the pose into the buffer of Front-End poses **/
    frontEndPose.push_front(pose);

    /** Set the inertial measurements coming from the Front-End **/
    inertial = inertialStateSamples[0];
    inertial.acc.setZero();
    inertial.gyro.setZero();
    inertial.incl.setZero();
    inertial.delta_vel.setZero();

    /** Process the buffer **/
    for (register unsigned int i=0; i<inertialStateSamplesSize; ++i)
    {
        delta_quaternion = delta_quaternion * inertialStateSamples[i].delta_orientation;
	inertial.acc += inertialStateSamples[i].acc;
	inertial.gyro += inertialStateSamples[i].gyro;
	inertial.incl += inertialStateSamples[i].incl;
        inertial.delta_vel += inertialStateSamples[i].delta_vel;
    }

    /** Set the delta in orientation along the time interval **/
    inertial.delta_orientation = delta_quaternion;

    /** Set the mean of the values for this time interval **/
    inertial.acc /= inertialStateSamplesSize;
    inertial.gyro /= inertialStateSamplesSize;
    inertial.incl /= inertialStateSamplesSize;
    inertial.delta_vel /= inertialStateSamplesSize;

    /** Push the sample into the buffer of inertial samples **/
    inertialState.push_front(inertial);

    /** Set all counters to zero **/
    counter.reset();

    return;
}

void BackEnd::initBackEndFilter(boost::shared_ptr<BackEndFilter> &filter, boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
                boost::circular_buffer<rover_localization::InertialState> &inertialState)
{
    /** The filter vector state variables one for the navigation quantities the other for the error **/
    WAugmentedState vstate;
    WAugmentedState verror;

    /************************************/
    /** Initialize the Back-End Filter **/
    /************************************/

    /** Initial covariance matrix **/
    BackEndFilter::SingleStateCovariance P0single; /** Initial P(0) for one state **/
    P0single.setZero();

    MTK::setDiagonal (P0single, &WSingleState::pos, 1e-03);
    MTK::setDiagonal (P0single, &WSingleState::vel, 1e-03);
    MTK::setDiagonal (P0single, &WSingleState::orient, 1e-03);
    MTK::setDiagonal (P0single, &WSingleState::gbias, 1e-10);
    MTK::setDiagonal (P0single, &WSingleState::abias, 1e-10);


    /** Initial covariance matrix for the Vector of States **/
    BackEndFilter::AugmentedStateCovariance P0; /** Initial P(0) for the whole Vector State **/

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
    vstate.statek_i.pos = frontEndPose[0].position; //!Initial position from kinematics
    vstate.statek_i.vel = frontEndPose[0].velocity; //!Initial velocity from kinematics
    vstate.statek_i.orient = static_cast< Eigen::Quaternion<double> >(frontEndPose[0].orientation);

    /** Set the initial accelerometers and gyros bias offset in the state vector **/
    vstate.statek_i.gbias = sensornoise.gbiasoff + inertialState[0].gbias_onoff; //!Initial gyros bias offset
    vstate.statek_i.abias = sensornoise.abiasoff + inertialState[0].abias_onoff; //!Initial accelerometers bias offset

    /** Copy the state in the vector of states **/
    vstate.statek = vstate.statek_i;
    vstate.statek_l = vstate.statek_i;

    /** Set the initial error of the verror state (by default pos, vel are zero and orient is identity) **/
    /**TO-DO: No needed in the vector state **/
    //verror.statek_i.gbias = vstate.statek_i.gbias;
    //verror.statek_i.abias = vstate.statek_i.abias;

    /** Copy the error state in the vector of states **/
    verror.statek = verror.statek_i;
    verror.statek_l = verror.statek_i;

    /** Create the filter **/
    filter.reset (new BackEndFilter (static_cast<const WAugmentedState> (vstate),
                                    static_cast<const WAugmentedState> (verror),
                                    static_cast<const BackEndFilter::AugmentedStateCovariance> (P0)));

    #ifdef DEBUG_PRINTS
    std::cout<<"\n";
    std::cout<<"[BACK-END POSE-SAMPLES] Single P0|0 is of size " <<P0single.rows()<<" x "<<P0single.cols()<<"\n";
    std::cout<<"[BACK-END POSE-SAMPLES] Single P0|0:\n"<<P0single<<"\n";
    std::cout<<"[BACK-END POSE-SAMPLES] P0|0 is of size " <<P0.rows()<<" x "<<P0.cols()<<"\n";
    std::cout<<"[BACK-END POSE-SAMPLES] P0|0:\n"<<P0<<"\n";
    std::cout<<"[BACK-END POSE-SAMPLES] state:\n"<<vstate.getVectorizedState()<<"\n";
    std::cout<<"[BACK-END POSE-SAMPLES] position:\n"<<vstate.statek_i.pos<<"\n";
    std::cout<<"[BACK-END POSE-SAMPLES] velocity:\n"<<vstate.statek_i.vel<<"\n";
    Eigen::Matrix <double,localization::NUMAXIS,1> euler; /** In Euler angles **/
    euler[2] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
    euler[1] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
    euler[0] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
    std::cout<<"[BACK-END POSE-SAMPLES] Roll: "<<euler[0]*localization::R2D<<" Pitch: "<<euler[1]*localization::R2D<<" Yaw: "<<euler[2]*localization::R2D<<"\n";
    std::cout<<"\n";
    #endif

    return;
}

localization::DataModel<double, 3> BackEnd::relativePosition(const boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose)
{
    localization::DataModel<double, 3> deltaPosition; /** Relative position **/

    if (frontEndPose.size() > 1)
    {
        deltaPosition.data = frontEndPose[0].position - frontEndPose[1].position;
        deltaPosition.Cov = frontEndPose[0].cov_position - frontEndPose[1].cov_position;
    }
    else
    {
        deltaPosition.data.setZero();
        deltaPosition.Cov = frontEndPose[0].cov_position;
    }

    std::cout<<"frontEndPose[0].position\n"<<frontEndPose[0].position<<"\n";
    std::cout<<"frontEndPose[0].cov_position\n"<<frontEndPose[0].cov_position<<"\n";

    return deltaPosition;
}

localization::DataModel<double, 3> BackEnd::velocityError(const boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
                                                        const boost::shared_ptr< BackEndFilter > filter)
{
    localization::DataModel<double, 3> veloError, stateVelocity, frontEndVelocity; /** Velocity **/

    /** Store the velocity in the Data Model types **/
    stateVelocity.data = filter->muState().statek_i.vel;
    stateVelocity.Cov = filter->PkSingleState().block<3,3>(3,3);
    if (localization::Util::isnotnan(frontEndPose[0].velocity))
        frontEndVelocity.data = frontEndPose[0].velocity;
    if (localization::Util::isnotnan(frontEndPose[0].cov_velocity))
        frontEndVelocity.Cov = frontEndPose[0].cov_velocity;
    else
        frontEndVelocity.Cov.setZero();

    /** Compute the error in velocity **/
    veloError = frontEndVelocity - stateVelocity;

    return veloError;
}

void BackEnd::outputPortSamples (const boost::shared_ptr< localization::Usckf<WAugmentedState, WSingleState> > filter,
                            const localization::DataModel<double, 3> &deltaVeloModel, const localization::DataModel<double, 3> &deltaVeloInertial)
{
    base::samples::RigidBodyState poseOut; /** Robot pose to port out **/
    WSingleState statek_i; /** Current robot state */
    rover_localization::BackEndEstimation backEndEstimationSamples; /** Filter information **/
    WAugmentedState errorAugmentedState = filter->muError();

    //localization::DataModel<double, 3> deltaVeloCommon = deltaVeloInertial;

    /** Fusion of deltaVeloModel with deltaVeloInertial **/
    //deltaVeloCommon.Cov = deltaVeloCommon.Cov;
    //deltaVeloCommon.fusion(deltaVeloModel);

    /** Init **/
    poseOut.invalidate();

    /** Get the current robot pose from the filter **/
    statek_i = filter->muState().statek_i;

    /***************************************/
    /** Port out the OutPorts information **/
    /***************************************/

    /** The Back-End Estimated pose **/
    poseOut.time = frontEndPoseSamples[0].time;
    poseOut.position = statek_i.pos;
    poseOut.cov_position = filter->PkSingleState().block<3,3>(0,0);
    poseOut.velocity = statek_i.vel;
    poseOut.cov_velocity = filter->PkSingleState().block<3,3>(3,3);
    poseOut.orientation = statek_i.orient;
    poseOut.cov_orientation = filter->PkSingleState().block<3,3>(6,6);
    _pose_samples_out.write(poseOut);

    /** Port out the estimated error in this sample interval **/


    /** Port out the filter information **/
    backEndEstimationSamples.time = frontEndPoseSamples[0].time;
    backEndEstimationSamples.statek_i = static_cast<WAugmentedState>(filter->muState()).statek_i.getVectorizedState();
    backEndEstimationSamples.errork_i = static_cast<WAugmentedState>(errorAugmentedState).statek_i.getVectorizedState();
    backEndEstimationSamples.orientation = filter->muState().statek_i.orient;
    backEndEstimationSamples.Pki = filter->PkAugmentedState();
    backEndEstimationSamples.gbias = filter->muState().statek_i.gbias;
    backEndEstimationSamples.abias = filter->muState().statek_i.abias;
    backEndEstimationSamples.accModel = deltaVeloModel.data;
    backEndEstimationSamples.accModelCov = deltaVeloModel.Cov;
    backEndEstimationSamples.accInertial = deltaVeloInertial.data;
    backEndEstimationSamples.accInertialCov = deltaVeloInertial.Cov;
//    backEndEstimationSamples.accError = deltaVeloError.data;
//    backEndEstimationSamples.accErrorCov = deltaVeloError.Cov;
//    backEndEstimationSamples.Hellinger = Hellinger;
    backEndEstimationSamples.mahalanobis = localization::Util::mahalanobis<double, 3> (deltaVeloModel, deltaVeloInertial);
    backEndEstimationSamples.Threshold = (deltaVeloModel.Cov + deltaVeloInertial.Cov).inverse();
//    backEndEstimationSamples.deltaVeloCommon = deltaVeloCommon.data;
//    backEndEstimationSamples.deltaVeloCommonCov = deltaVeloCommon.Cov;
    _backend_estimation_samples_out.write(backEndEstimationSamples);

    return;
}


