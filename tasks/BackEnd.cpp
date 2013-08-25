/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BackEnd.hpp"

#define DEBUG_PRINTS 1

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
    adapValues.reset();
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
    std::cout<<"[BACK-END POSE-SAMPLES] Received new samples at "<<frontEndPoseSamples[0].time.toMicroseconds()<<"\n";
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
    std::cout<<"[BACK-END PROPRIO-SAMPLES] Received new samples at "<<inertialStateSamples[0].time.toMicroseconds()<<"\n";
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
            std::cout<<"[BACK-END POSE-SAMPLES] - "<<inertialState.time.toMicroseconds()<<" - Initializing Filter...";
            #endif

            /** The filter vector state variables one for the nav. quantities the other for the error **/
            WAugmentedState vstate;
            WAugmentedState verror;

            /************************************/
            /** Initialize the Back-End Filter **/
            /************************************/

            /** Initial covariance matrix **/
            localization::Usckf<WAugmentedState, WSingleState>::SingleStateCovariance P0single; /** Init P(0) for one state **/
            P0single.setZero();

            MTK::setDiagonal (P0single, &WSingleState::pos, 1e-06);
            MTK::setDiagonal (P0single, &WSingleState::vel, 1e-06);
            MTK::setDiagonal (P0single, &WSingleState::orient, 1e-06);
            MTK::setDiagonal (P0single, &WSingleState::gbias, 1e-10);
            MTK::setDiagonal (P0single, &WSingleState::abias, 1e-10);


            /** Initial covariance matrix for the Vector of States **/
            localization::Usckf<WAugmentedState, WSingleState>::AugmentedStateCovariance P0; /** Init P(0) for the whole Vectro State **/

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
            vstate.statek_i.pos = frontEndPose.position; //!Initial position from kinematics
            vstate.statek_i.vel = frontEndPose.velocity; //!Initial velo from kinematics
            vstate.statek_i.orient = static_cast< Eigen::Quaternion<double> >(frontEndPose.orientation);

            /** Set the initial acc and gyros bias offset in the state vector **/
            vstate.statek_i.gbias = sensornoise.gbiasoff + inertialState.gbias_onoff; //!Initial gyros bias offset
            vstate.statek_i.abias = sensornoise.abiasoff + inertialState.abias_onoff; //!Initial acc bias offset

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
            filter.reset (new localization::Usckf<WAugmentedState, WSingleState> (static_cast<const WAugmentedState> (vstate),
                                                                static_cast<const WAugmentedState> (verror),
                                                                static_cast<const localization::Usckf<WAugmentedState,
                                                                WSingleState>::AugmentedStateCovariance> (P0)));

            #ifdef DEBUG_PRINTS
            std::cout<<"[DONE]\n**[FILTER_INITIALIZATION]**\n";
            std::cout<<"[BACK-END POSE-SAMPLES] Single P0|0 is of size " <<P0single.rows()<<" x "<<P0single.cols()<<"\n";
            std::cout<<"[BACK-END POSE-SAMPLES] Single P0|0:\n"<<P0single<<"\n";
            std::cout<<"[BACK-END POSE-SAMPLES] P0|0 is of size " <<P0.rows()<<" x "<<P0.cols()<<"\n";
            std::cout<<"[BACK-END POSE-SAMPLES] P0|0:\n"<<P0<<"\n";
            std::cout<<"[BACK-END POSE-SAMPLES] position:\n"<<vstate.statek_i.pos<<"\n";
            std::cout<<"[BACK-END POSE-SAMPLES] velocity:\n"<<vstate.statek_i.vel<<"\n";
            Eigen::Matrix <double,localization::NUMAXIS,1> euler; /** In euler angles **/
            euler[2] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
            euler[1] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
            euler[0] = vstate.statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
	    std::cout<<"[BACK-END POSE-SAMPLES] Roll: "<<euler[0]*localization::R2D<<" Pitch: "<<euler[1]*localization::R2D<<" Yaw: "<<euler[2]*localization::R2D<<"\n";
            std::cout<<"\n";
            #endif

            initFilter = true;
        }

        double delta_t = (1.0/framework.backend_frequency); /** Delta integration time */
        WSingleState statek_i; /** Current robot state (copy from the filter object) */
        WSingleState deltaStatek_i; /** Delta in robot state to propagate the state (fill wih info coming from FrontEnd) */

        #ifdef DEBUG_PRINTS
        std::cout<<"[BACK-END POSE-SAMPLES] - "<<inertialState.time.toMicroseconds()<<" - Performing Filter@"<< delta_t <<" seconds\n";
        #endif

        /**********************************************/
        /** Propagation of the navigation quantities **/
        /**********************************************/

        /** Robot's state estimate is propagated using the non-linear equation **/
        deltaStatek_i.pos = inertialState.delta_vel * delta_t; //!Increment in position
        deltaStatek_i.vel = inertialState.delta_vel; //! Increment in velocity using inertial sensors.
        deltaStatek_i.orient = static_cast< MTK::SO3<double> > (inertialState.delta_orientation); //! Delta in orientation coming from Front End

        #ifdef DEBUG_PRINTS
        std::cout<<"\n[BACK-END POSE-SAMPLES] DELTA_STATE \n";
        std::cout<<"[BACK-END POSE-SAMPLES] position:\n"<<deltaStatek_i.pos<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] velocity:\n"<<deltaStatek_i.vel<<"\n";
        Eigen::Matrix <double,localization::NUMAXIS,1> eulerdelta; /** In euler angles **/
        eulerdelta[2] = deltaStatek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
        eulerdelta[1] = deltaStatek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
        eulerdelta[0] = deltaStatek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
        std::cout<<"[BACK-END POSE-SAMPLES] Roll: "<<eulerdelta[0]*localization::R2D<<" Pitch: "<<eulerdelta[1]*localization::R2D<<" Yaw: "<<eulerdelta[2]*localization::R2D<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] Roll(rad): "<<eulerdelta[0]<<" Pitch(rad): "<<eulerdelta[1]<<" Yaw(rad): "<<eulerdelta[2]<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] w: "<<deltaStatek_i.orient.w()<<" x: "<<deltaStatek_i.orient.x()<<" y: "<<deltaStatek_i.orient.y()<<" z:"<<deltaStatek_i.orient.z()<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] gbias:\n"<<deltaStatek_i.gbias<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] abias:\n"<<deltaStatek_i.abias<<"\n\n";
        std::cout<<"[BACK-END POSE-SAMPLES] DELTA_STATE in Vectorized_form:\n"<<deltaStatek_i.getVectorizedState(::localization::State::ERROR_QUATERNION)<<"\n\n";
        #endif

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
        std::cout<<"[BACK-END POSE-SAMPLES] Roll: "<<eulerb[0]*localization::R2D<<" Pitch: "<<eulerb[1]*localization::R2D<<" Yaw: "<<eulerb[2]*localization::R2D<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] gbias:\n"<<statek_i.gbias<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] abias:\n"<<statek_i.abias<<"\n\n";
        #endif

        //filter->propagate (deltaStatek_i);
        statek_i = statek_i + deltaStatek_i.getVectorizedState(); filter->setStatek_i(statek_i);
        //statek_i.orient.boxplus(eulerdelta);

        /** Get the current state (after propagation) **/
        statek_i = filter->muState().statek_i;

        #ifdef DEBUG_PRINTS
        std::cout<<"\n[BACK-END POSE-SAMPLES] AFTER_PROPAGATION \n";
        std::cout<<"[BACK-END POSE-SAMPLES] position:\n"<<statek_i.pos<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] velocity:\n"<<statek_i.vel<<"\n";
        Eigen::Matrix <double,localization::NUMAXIS,1> euler; /** In euler angles **/
        euler[2] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
        euler[1] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
        euler[0] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
        std::cout<<"[BACK-END POSE-SAMPLES] Roll: "<<euler[0]*localization::R2D<<" Pitch: "<<euler[1]*localization::R2D<<" Yaw: "<<euler[2]*localization::R2D<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] gbias:\n"<<statek_i.gbias<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] abias:\n"<<statek_i.abias<<"\n\n";
        #endif


        /*******************/
        /** Predict State **/
        /*******************/

        /** Form the process model matrix **/
        typedef Eigen::Matrix<double, WSingleState::DOF, WSingleState::DOF> SingleStateProcessModelMatrix;
        SingleStateProcessModelMatrix processModelF = localization::processModel
                                                < WSingleState, SingleStateProcessModelMatrix>
                                                (inertialState.gyro, inertialState.acc,
                                                static_cast<Eigen::Quaterniond&>(statek_i.orient), delta_t);

        /** Form the process covariance matrix **/

        /** Typedef for the statek_i covariance type **/
        typedef localization::Usckf <WAugmentedState, WSingleState>::SingleStateCovariance SingleStateCovariance;

        SingleStateCovariance processCovQ;
        processCovQ = localization::processNoiseCov
                            <WSingleState, SingleStateCovariance > (processModelF, sensornoise.accrw, sensornoise.gyrorw,
                                                                    sensornoise.gbiasins, sensornoise.abiasins, static_cast<Eigen::Quaterniond&>(statek_i.orient), delta_t) ;

        /** Robot's error state is propagated using the non-linear noise dynamic model (processModel) **/
        filter->ekfPredict(processModelF, processCovQ);

        /*******************/
        /** Update State **/
        /*******************/

        /** Delta velocity from Motion Model **/
        Eigen::Vector3d deltaVelocity;
        deltaVelocity.setZero(); //!TO-DO: take the statisticalMotion model

        /** Form the measurement covariance matrix **/
        Eigen::Matrix<double, 6, 6> measuCovR;
        measuCovR = localization::proprioceptiveMeasurementNoiseCov(sensornoise.accrw, delta_t);

        /** Adaptive part of the measurement covariance **/
        WSingleState::vectorized_type vxk_i = localization::getVectorizedSingleState <WSingleState, WSingleState::vectorized_type> (statek_i); /** Get the state in eigen vector form */
        Eigen::Matrix<double, 6, WSingleState::DOF> H;/** Observation matrix H (measurement model) */
        localization::Usckf<WAugmentedState, WSingleState>::SingleStateCovariance Pksingle; /** covariance of the single state */
        localization::Usckf<WAugmentedState, WSingleState>::AugmentedStateCovariance Pk; /** covarianec of the whole state (stochastic clonning states) */

        /** Observation matrix H (measurement model) **/
        H = localization::proprioceptiveMeasurementMatrix<WSingleState::DOF>(statek_i.orient, inertialState.theoretical_g);

        /** Fill the covariance matrix (whole vector and subvector) **/
        Pk = filter->PkAugmentedState();
        Pksingle = MTK::subblock (Pk, &WAugmentedState::statek_i, &WAugmentedState::statek_i);

        /** Adaptive covariance matrix of the measurememt **/
        measuCovR.bottomRightCorner<3,3>() = adapAtt->matrix<WSingleState::DOF>
            (vxk_i, Pksingle, inertialState.acc,
             H.block<3, WSingleState::DOF>(3,0),
             measuCovR.bottomRightCorner<3,3>());


        /** Update attitude using accelerometers and
         * the velocity using the statistical motion
         * model comming from the FrontEnd */
        typedef Eigen::Matrix<double, 6, 1> MeasurementVector;
        MeasurementVector z;/** Unified measurement vector (delta velocity and acc) */

        /** Fill the measurement vector **/
        z.block<3,1>(0,0) = deltaVelocity;
        z.block<3,1>(3,0) = inertialState.acc;

        /** Perform the update **/

        filter->singleUpdate (z, boost::bind (localization::proprioceptiveMeasurementModel
                    < WSingleState, MeasurementVector, WSingleState::vectorized_type >,
                    _1, H), measuCovR);

        /** Peform the update in a EKF form **/
        //filter->ekfUpdate < Eigen::Matrix<double,6,1>,
        //                    Eigen::Matrix<double,6, WSingleState::DOF>,
        //                    Eigen::Matrix<double,6,6>
        //                        > (vxk_i, z, H, measuCovR);

        /** Copy the error state to port it out */
        WAugmentedState errorAugmentedState = filter->muError();

        /** Reset the error vector state and perform clonning */
        filter->muErrorSingleReset();
        filter->clonning();

        /** Get the corrected state **/
        statek_i = filter->muState().statek_i;

        #ifdef DEBUG_PRINTS
        std::cout<<"\n[BACK-END POSE-SAMPLES] AFTER CORRECTION \n";
        std::cout<<"[BACK-END POSE-SAMPLES] position:\n"<<statek_i.pos<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] velocity:\n"<<statek_i.vel<<"\n";
        euler[2] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
        euler[1] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
        euler[0] = statek_i.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
        std::cout<<"[BACK-END POSE-SAMPLES] Roll: "<<euler[0]*localization::R2D<<" Pitch: "<<euler[1]*localization::R2D<<" Yaw: "<<euler[2]*localization::R2D<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] gbias:\n"<<statek_i.gbias<<"\n";
        std::cout<<"[BACK-END POSE-SAMPLES] abias:\n"<<statek_i.abias<<"\n\n";
        #endif

        /** Out port the information of the Back-End **/
        this->outputPortSamples (inertialState.time, filter, errorAugmentedState);

        flag.reset();
    }

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
    #endif

    /** Set the delta time for the noise calculation */
    /*  To choose the maximum between sensor bandwidth and backend delta time **/
    double delta_bandwidth = (1.0/sensornoise.bandwidth); /** Bandwidth delta interval */
    double delta_backend = (1.0/framework.backend_frequency); /** Bandwidth delta interval */

    if (delta_backend > delta_bandwidth)
        this->delta_noise = sqrt(delta_backend);
    else
        this->delta_noise = sqrt(delta_bandwidth);

    /** Create the class for the adaptive measurement update of the orientation **/
    adapAtt.reset (new localization::AdaptiveAttitudeCov (adapValues.M1, adapValues.M2, adapValues.gamma, adapValues.r2count));

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
}

/*WSingleState::vectorized_type BackEnd::getVectorizeSingleState (WSingleState & state)
{
    WSingleState::vectorized_type vstate;

    vstate.block<localization::vec3::DOF, 1>(0,0) = state.pos; //!Position
    vstate.block<localization::vec3::DOF, 1>(localization::vec3::DOF,0) = state.vel; //!Velocity

    Eigen::Matrix<double, localization::SO3::DOF, 1> qe; //! Error quaternion
    qe << state.orient.x(), state.orient.y(), state.orient.z();
    vstate.block<localization::SO3::DOF, 1>(2*localization::vec3::DOF, 0) = qe;

    vstate.block<localization::vec3::DOF, 1>(2*localization::vec3::DOF + localization::SO3::DOF, 0) = state.gbias; //! Gyros bias
    vstate.block<localization::vec3::DOF, 1>(3*localization::vec3::DOF + localization::SO3::DOF, 0) = state.abias; //! Acc bias

    return vstate;
}*/

void BackEnd::inputPortSamples(base::samples::RigidBodyState &frontEndPose, rover_localization::InertialState &inertialState)
{
    unsigned int frontEndPoseSize = frontEndPoseSamples.size();
    unsigned int inertialStateSize = inertialStateSamples.size();
    Eigen::Quaterniond delta_quaternion; /** Accumulation of delta in orientation along the buffers */

    /** Deta quaternion to identuty **/
    delta_quaternion.setIdentity();

    #ifdef DEBUG_PRINTS
    std::cout<<"[BACK-END GET_INPORT] frontEndPoseSamples has capacity "<<frontEndPoseSamples.capacity()<<" and size "<<frontEndPoseSamples.size()<<"\n";
    std::cout<<"[BACK-END GET_INPORT] inertialStateSamples has capacity "<<inertialStateSamples.capacity()<<" and size "<<inertialStateSamples.size()<<"\n";
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


    /** Set the inertial measurements comming from the FrontEnd **/
    inertialState = inertialStateSamples[0];
    inertialState.acc.setZero();
    inertialState.gyro.setZero();
    inertialState.incl.setZero();
    inertialState.delta_vel.setZero();

    /** Process the buffer **/
    for (register unsigned int i=0; i<inertialStateSize; ++i)
    {
        delta_quaternion = delta_quaternion * inertialStateSamples[i].delta_orientation;
	inertialState.acc += inertialStateSamples[i].acc;
	inertialState.gyro += inertialStateSamples[i].gyro;
	inertialState.incl += inertialStateSamples[i].incl;
        inertialState.delta_vel += inertialStateSamples[i].delta_vel;
    }

    /** Set the delta in orientation along the time interval **/
    inertialState.delta_orientation = delta_quaternion;

    /** Set the mean of the values for this time interval **/
    inertialState.acc /= inertialStateSize;
    inertialState.gyro /= inertialStateSize;
    inertialState.incl /= inertialStateSize;
    inertialState.delta_vel /= inertialStateSize;

    /** Set all counters to zero **/
    counter.reset();

    return;
}

void BackEnd::outputPortSamples (const base::Time &timestamp, const boost::shared_ptr< localization::Usckf<WAugmentedState, WSingleState> > filter, const WAugmentedState &errorAugmentedState)
{
    base::samples::RigidBodyState poseOut;
    WSingleState statek_i; /** Current robot state */

    /** Init **/
    poseOut.invalidate();
    statek_i = filter->muState().statek_i;

    /***************************************/
    /** Port out the OutPorts information **/
    /***************************************/

    /** The Back-End Estimated pose **/
    poseOut.time = timestamp;
    poseOut.position = statek_i.pos;
    poseOut.velocity = statek_i.vel;
    poseOut.orientation = statek_i.orient;
    _pose_samples_out.write(poseOut);

    /** Port out the estimated error in this sample interval **/


    /** Port out the filter information **/

    return;
}


