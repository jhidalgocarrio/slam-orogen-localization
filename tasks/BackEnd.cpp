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

    /******************************************/
    /*** General Internal Storage Variables ***/
    /******************************************/
    frontEndPose.invalidate();
    inertialState.acc = base::NaN<double>() * base::Vector3d::Zero();
    inertialState.gyro = base::NaN<double>() * base::Vector3d::Zero();
    inertialState.incl = base::NaN<double>() * base::Vector3d::Zero();

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
    std::cout<<"[BE POSE-SAMPLES] Received new samples at "<<frontEndPoseSamples[0].time.toMicroseconds()<<"\n";
    #endif

    /** Set the flag of FrontEnd Pose values valid to true **/
    if (!flag.frontEndPoseSamples && (frontEndPoseSamples.size() == frontEndPoseSamples.capacity()))
        flag.frontEndPoseSamples = true;


}

void BackEnd::inertial_samplesTransformerCallback(const base::Time &ts, const ::rover_localization::InertialState &inertial_samples_sample)
{
    /** A new sample arrived to the input port **/
    inertialStateSamples.push_front(inertial_samples_sample);
    counter.inertialStateSamples++;

    #ifdef DEBUG_PRINTS
    std::cout<<"[BE PROPRIO-SAMPLES] Received new samples at "<<inertialStateSamples[0].time.toMicroseconds()<<"\n";
    #endif

    if (counter.inertialStateSamples == number.inertialStateSamples)
        flag.inertialStateSamples = true;
    else
        flag.inertialStateSamples = false;

    if (flag.frontEndPoseSamples && flag.inertialStateSamples)
    {
        /** Get the correct values from the input port at the desired BackEnd frequency **/
        this->inputPortSamples(frontEndPose, inertialState);

        /** If filter is not initialized do it right now **/
        if(!initFilter)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"[BE POSE-SAMPLES] Initializing Filter...";
            #endif

            /** The filter vector state variables one for the nav. quantities the other for the error **/
            WVectorState vstate;
            WVectorState verror;

            /************************************/
            /** Initialize the Back-End Filter **/
            /************************************/

            /** Initial covariance matrix **/
            localization::Usckf<WVectorState, WSingleState>::SingleStateCovariance P0single; /** Init P(0) for one state **/

            MTK::setDiagonal (P0single, &WSingleState::pos, 1e-06);
            MTK::setDiagonal (P0single, &WSingleState::vel, 1e-06);
            MTK::setDiagonal (P0single, &WSingleState::orient, 1e-06);
            MTK::setDiagonal (P0single, &WSingleState::gbias, 1e-10);
            MTK::setDiagonal (P0single, &WSingleState::abias, 1e-10);


            /** Initial covariance matrix for the Vector of States **/
            localization::Usckf<WVectorState, WSingleState>::VectorStateCovariance P0; /** Init P(0) for the whole Vectro State **/

            MTK::subblock (P0, &WVectorState::statek, &WVectorState::statek) = P0single;
            MTK::subblock (P0, &WVectorState::statek_l, &WVectorState::statek_l) = P0single;
            MTK::subblock (P0, &WVectorState::statek_i, &WVectorState::statek_i) = P0single;

            MTK::subblock (P0, &WVectorState::statek, &WVectorState::statek_l) = P0single;
            MTK::subblock (P0, &WVectorState::statek, &WVectorState::statek_i) = P0single;
            MTK::subblock (P0, &WVectorState::statek_l, &WVectorState::statek) = P0single;
            MTK::subblock (P0, &WVectorState::statek_i, &WVectorState::statek) = P0single;

            MTK::subblock (P0, &WVectorState::statek_l, &WVectorState::statek_i) = P0single;
            MTK::subblock (P0, &WVectorState::statek_i, &WVectorState::statek_l) = P0single;

            /** Set the current pose to initialize the filter structure **/
            vstate.statek_i.pos = frontEndPose.position;
            vstate.statek_i.vel = frontEndPose.velocity;
            vstate.statek_i.orient = static_cast< Eigen::Quaternion<double> >(frontEndPose.orientation);

            /** Set the initial acc and gyros bias offset in the state vector **/
            vstate.statek_i.gbias = sensornoise.gbiasoff;
            vstate.statek_i.abias = sensornoise.abiasoff;

            /** Copy the state in the vector of states **/
            vstate.statek = vstate.statek_i;
            vstate.statek_l = vstate.statek_i;

            /** Create the filter **/
            filter.reset (new localization::Usckf<WVectorState, WSingleState> (static_cast<const WVectorState> (vstate),
                                                                static_cast<const WVectorState> (verror),
                                                                static_cast<const localization::Usckf<WVectorState,
                                                                WSingleState>::VectorStateCovariance> (P0)));
            #ifdef DEBUG_PRINTS
            std::cout<<"[DONE]\n";
            #endif

            initFilter = true;
        }
        else
        {
            /** Typedef for the statek_i covariance type **/
            typedef localization::Usckf <WVectorState, WSingleState>::SingleStateCovariance SingleStateCovariance;

            double delta_t = (1.0/framework.backend_frequency); /** Delta integration time */
            localization::SingleState statek_i; /** Current robot state */

            #ifdef DEBUG_PRINTS
            std::cout<<"[BE POSE-SAMPLES] Performing Filter@delta ("<< delta_t <<")\n";
            #endif

            /** Robot's state estimate is propagate using the non-linear equation at FrontEnd **/

            /** Form the process covariance matrix **/
            localization::Usckf <WVectorState, WSingleState>::SingleStateCovariance processCovQ;
            processCovQ = localization::processNoiseCov
                                <WSingleState, SingleStateCovariance > (sensornoise.accrw, sensornoise.gyrorw,
                                                                        sensornoise.gbiasins, sensornoise.abiasins, static_cast<Eigen::Quaterniond&>(statek_i.orient), delta_t) ;

            /** Robot's error state is propagated using the non-linear noise dynamic model (processModel) **/

            /** Update using the statistical motion model info comming from the FrontEnd **/

        }

        flag.reset();
    }

    /** TO-DO: check how are the values (bias, gravity, etc) and perform the filter.predict **/
    /** TO-DO: check how if the rover velocity  comes in the body or in the world frame **/

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
    std::cout<<"[BE CONFIGURE] frontEndPoseSamples has capacity "<<frontEndPoseSamples.capacity()<<" and size "<<frontEndPoseSamples.size()<<"\n";
    std::cout<<"[BE CONFIGURE] inertialStateSamples has capacity "<<inertialStateSamples.capacity()<<" and size "<<inertialStateSamples.size()<<"\n";
    #endif

    /** Set the delta time for the noise calculation */
    /*  To choose the maximum between sensor bandwidth and backend delta time **/
    double delta_bandwidth = (1.0/sensornoise.bandwidth); /** Bandwidth delta interval */
    double delta_backend = (1.0/framework.backend_frequency); /** Bandwidth delta interval */

    if (delta_backend > delta_bandwidth)
        delta_noise = sqrt(delta_backend);
    else
        delta_noise = sqrt(delta_bandwidth);

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
}


void BackEnd::inputPortSamples(base::samples::RigidBodyState &frontEndPose, rover_localization::InertialState &inertialState)
{
    unsigned int frontEndPoseSize = frontEndPoseSamples.size();
    unsigned int inertialStateSize = inertialStateSamples.size();

    #ifdef DEBUG_PRINTS
    std::cout<<"[BE GET_INPORT] frontEndPoseSamples has capacity "<<frontEndPoseSamples.capacity()<<" and size "<<frontEndPoseSamples.size()<<"\n";
    std::cout<<"[BE GET_INPORT] inertialStateSamples has capacity "<<inertialStateSamples.capacity()<<" and size "<<inertialStateSamples.size()<<"\n";
    #endif

    /** Set the position comming from the FrontEnd **/
    frontEndPose = frontEndPoseSamples[0];
    frontEndPose.velocity.setZero();
    frontEndPose.cov_velocity.setZero();

    /** Process the buffer for velocity average **/
    for (register unsigned int i = 0; i<frontEndPoseSize; ++i)
    {
        frontEndPose.velocity += frontEndPoseSamples[i].velocity;
        frontEndPose.cov_velocity += frontEndPoseSamples[i].cov_velocity;
    }

    /** Velocity average over the BakcEnd time interval **/
    frontEndPose.velocity /= frontEndPoseSize;
    frontEndPose.cov_velocity /= frontEndPoseSize;


    /** Set the inertial measurements **/
    inertialState.acc.setZero();
    inertialState.gyro.setZero();
    inertialState.incl.setZero();

    /** Process the buffer **/
    for (register unsigned int i=0; i<inertialStateSize; ++i)
    {
	inertialState.acc += inertialStateSamples[i].acc;
	inertialState.gyro += inertialStateSamples[i].gyro;
	inertialState.incl += inertialStateSamples[i].incl;
    }

    /** Set the time **/
    inertialState.time = inertialStateSamples[0].time;

    /** Set the mean of this time interval **/
    inertialState.acc /= inertialStateSize;
    inertialState.gyro /= inertialStateSize;
    inertialState.incl /= inertialStateSize;

    /** Set all counters to zero **/
    counter.reset();

    return;
}



