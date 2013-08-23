/* Generated from orogen/lib/orogen/templates/tasks/FrontEnd.cpp */

#include "FrontEnd.hpp"

//#define DEBUG_PRINTS 1

using namespace rover_localization;

FrontEnd::FrontEnd(std::string const& name)
    : FrontEndBase(name)
{

    /******************************/
    /*** Control Flow Variables ***/
    /******************************/
    initPosition = false;
    initAttitude = false;

    flag.reset();
    counter.reset();
    number.reset();

    /***************************/
    /** Output port variables **/
    /***************************/
    poseOut.invalidate();
    referenceOut.invalidate();
    inertialState.time.fromMicroseconds(base::NaN<uint64_t>());
    inertialState.acc = base::NaN<double>() * base::Vector3d::Zero();
    inertialState.gyro = base::NaN<double>() * base::Vector3d::Zero();
    inertialState.incl = base::NaN<double>() * base::Vector3d::Zero();
    inertialState.theoretical_g = base::NaN<double>();
    inertialState.estimated_g = base::NaN<double>();
    inertialState.abias_onoff = base::NaN<double>() * base::Vector3d::Zero();
    inertialState.gbias_onoff = base::NaN<double>() * base::Vector3d::Zero();

    /**********************************/
    /*** Internal Storage Variables ***/
    /**********************************/

    /** Default size for initial acceleration leveling **/
    init_leveling_acc.resize (localization::NUMAXIS, DEFAULT_INIT_LEVELING_SIZE);

    /** Default size for initial acceleration leveling **/
    init_leveling_incl.resize (localization::NUMAXIS, DEFAULT_INIT_LEVELING_SIZE);

    /** Default size for the circular_buffer of the raw port samples **/
    cbEncoderSamples = boost::circular_buffer<base::actuators::Status>(DEFAULT_CIRCULAR_BUFFER_SIZE);
    cbAsguardStatusSamples = boost::circular_buffer<sysmon::SystemStatus> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    cbImuSamples = boost::circular_buffer<base::samples::IMUSensors> (DEFAULT_CIRCULAR_BUFFER_SIZE);

    /** Default size for the circular_buffer for the filtered port samples **/
    encoderSamples = boost::circular_buffer<base::actuators::Status>(DEFAULT_CIRCULAR_BUFFER_SIZE);
    asguardStatusSamples = boost::circular_buffer<sysmon::SystemStatus> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    imuSamples = boost::circular_buffer<base::samples::IMUSensors> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    poseSamples = boost::circular_buffer<base::samples::RigidBodyState> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    backEndEstimationSamples = boost::circular_buffer<rover_localization::BackEndEstimation> (DEFAULT_CIRCULAR_BUFFER_SIZE);

    /** Default size for the std_vector for the cartesian 6DoF velocities variables **/
    vectorCartesianVelocities = std::vector< Eigen::Matrix <double, 2*localization::NUMAXIS, 1> , Eigen::aligned_allocator < Eigen::Matrix <double, 2*localization::NUMAXIS, 1> > > (DEFAULT_CIRCULAR_BUFFER_SIZE);

    /** Set eccentricity to NaN **/
    eccx[0] = base::NaN<double>();
    eccx[1] = base::NaN<double>();
    eccx[2] = base::NaN<double>();

    eccy[0] = base::NaN<double>();
    eccy[1] = base::NaN<double>();
    eccy[2] = base::NaN<double>();

    eccz[0] = base::NaN<double>();
    eccz[1] = base::NaN<double>();
    eccz[2] = base::NaN<double>();

    /** Uncertainty matrices for the motion model **/
    cartesianVelCov = base::NaN<double>() * Eigen::Matrix< double, 6, 6 >::Identity();
    modelVelCov = base::NaN<double>() * Eigen::Matrix< double, frontEndMotionModel::MODEL_DOF, frontEndMotionModel::MODEL_DOF >::Identity();

}

FrontEnd::FrontEnd(std::string const& name, RTT::ExecutionEngine* engine)
    : FrontEndBase(name, engine)
{
}

FrontEnd::~FrontEnd()
{
}

void FrontEnd::reference_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &reference_pose_samples_sample)
{
    poseSamples.push_front(reference_pose_samples_sample);

    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation in quaternion form **/

    /** Get the transformation **/
    if (!_vicon2body.get(ts, tf, false))
	return;

    qtf = Eigen::Quaternion <double> (tf.rotation());

    #ifdef DEBUG_PRINTS
    std::cout<<"** [FE REFERENCE-POSE]Received poseSamples sample at("<<reference_pose_samples_sample.time.toMicroseconds()<<") **\n";
    #endif

    /** Apply the transformer pose offset **/
    poseSamples[0].position += qtf * tf.translation();
    poseSamples[0].orientation = poseSamples[0].orientation * qtf;
    /*poseSamples[0].velocity = qtf * poseSamples[0].velocity;
    poseSamples[0].cov_velocity = tf.rotation() * poseSamples[0].cov_velocity;
    poseSamples[0].cov_angular_velocity = tf.rotation() * poseSamples[0].cov_angular_velocity;*/

   if (!initPosition)
    {
	poseOut.position = poseSamples[0].position;
	poseOut.velocity.setZero();
	
	/** Assume well known starting position **/
	poseOut.cov_position = Eigen::Matrix <double, localization::NUMAXIS , localization::NUMAXIS>::Zero();
	poseOut.cov_velocity = Eigen::Matrix <double, localization::NUMAXIS , localization::NUMAXIS>::Zero();
	
	#ifdef DEBUG_PRINTS
	Eigen::Matrix <double,localization::NUMAXIS,1> euler; /** In euler angles **/
	euler[2] = poseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
	euler[1] = poseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
	euler[0] = poseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
 	std::cout<<"** [FE REFERENCE-POSE]poseSamples at ("<<poseSamples[0].time.toMicroseconds()<< ")**\n";
	std::cout<<"** position offset\n"<<tf.translation()<<"\n";
	std::cout<<"** rotation offset\n"<<tf.rotation()<<"\n";
	std::cout<<"** position\n"<< poseSamples[0].position<<"\n";
	std::cout<<"** Roll: "<<euler[0]*localization::R2D<<" Pitch: "<<euler[1]*localization::R2D<<" Yaw: "<<euler[2]*localization::R2D<<"\n";
	#endif

	/** Initial attitude from IMU acceleration has been already calculated **/
	if (initAttitude)
	{
	    Eigen::Matrix <double,localization::NUMAXIS,1> euler; /** In euler angles **/
            Eigen::Quaternion <double> attitude = poseOut.orientation; /** Initial attitude from accelerometers has been already calculated **/
	
	    /** Get the initial Yaw from the initialPose and the pitch and roll **/
	    euler[2] = poseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
	    euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
	    euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
	
            /** Check the Initial attitude */
            if (base::isNaN<double>(euler[0]) || base::isNaN<double>(euler[1]) || base::isNaN<double>(euler[2]))
                RTT::log(RTT::Fatal)<<"[FRONT-END FATAL ERROR]  Initial Attitude from External Reference is NaN."<<RTT::endlog();


	    /** Set the initial attitude with the Yaw provided from the initial pose **/
	    attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
	    Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
	
	    /**Store the value as the initial one for the poseOut **/
	    poseOut.orientation = attitude;
	
	    #ifdef DEBUG_PRINTS
            std::cout<< "[FE REFERENCE-POSE]\n";
	    std::cout<< "******** Initial Attitude in Pose Init Samples *******"<<"\n";
	    std::cout<< "Init Roll: "<<euler[0]*localization::R2D<<"Init Pitch: "<<euler[1]*localization::R2D<<"Init Yaw: "<<euler[2]*localization::R2D<<"\n";
	    #endif
	}
	
	/** Initial angular velocity **/
	poseOut.angular_velocity.setZero();
	
	/** Assume very well know initial attitude **/
	poseOut.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
	poseOut.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
		
	initPosition = true;
    }

   flag.referencePoseSamples = true;
}

void FrontEnd::inertial_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation in quaternion form **/

    /** Get the transformation **/
    if (!_imu2body.get(ts, tf, false))
	return;

    qtf = Eigen::Quaternion <double> (tf.rotation());//!Quaternion from IMU to Body

    #ifdef DEBUG_PRINTS
   // std::cout<<"Transformer:\n"<<tf.matrix()<<"\n";
    #endif
	
    /** Check if the eccentricities are not defined **/
    if (base::isNaN(eccx[0]) && base::isNaN(eccy[0]) && base::isNaN(eccz[0]))
    {
	#ifdef DEBUG_PRINTS
	std::cout<<"[FE INERTIAL-SAMPLES]\nEccentricities are NaN\n";
	std::cout<<"Translation:\n"<<tf.translation()<<"\n";
	std::cout<<"Eccx :\n"<<tf.rotation() * _eccx.value()<<"\n";
	std::cout<<"Eccy :\n"<<tf.rotation() * _eccy.value()<<"\n";
	std::cout<<"Eccz :\n"<<tf.rotation() * _eccz.value()<<"\n";
	#endif
	
	/** Store the excentricity **/
	eccx = tf.translation() + tf.rotation() *_eccx.value();
	eccy = tf.translation() + tf.rotation() *_eccy.value();
	eccz = tf.translation() + tf.rotation() *_eccz.value();
	
    }

    /** If the initial attitude is not defined and the orientation port is not connected **/
    if (!initAttitude)
    {
	#ifdef DEBUG_PRINTS
	std::cout<< "[FE INERTIAL-SAMPLES]Calculating initial level position since Init orientation is not provided.["<<init_leveling_accidx<<"]\n";
	#endif
	
	/** Add one acc sample to the buffer **/
	init_leveling_acc.col(init_leveling_accidx) = inertial_samples_sample.acc;
	
	/** Add one acc(inclinometer) sample to the buffer **/
	init_leveling_incl.col(init_leveling_accidx) = inertial_samples_sample.mag; //!TO-DO: still incl values come into the mag field of the class
	
	init_leveling_accidx++;
	
	/** Chekc if the number of stores acceleration corresponds to the initial leveling **/
	if (init_leveling_accidx == this->init_leveling_size)
	{
	    Eigen::Matrix <double,localization::NUMAXIS,1> meanacc; /** Mean value of accelerometers **/
	    Eigen::Matrix <double,localization::NUMAXIS,1> meanincl; /** Mean value of inclinometer **/
	    Eigen::Matrix <double,localization::NUMAXIS,1> euler; /** In euler angles **/
	    Eigen::Quaternion <double> attitude; /** Initial attitude in case no port in orientation is connected **/
            Eigen::Quaternion <double> q_world2imu;  /** Initial orienttation of the imu frame wrt the world fixed frame **/
	
	    /** Mean inclinometers value **/
	    meanincl[0] = init_leveling_incl.row(0).mean();
	    meanincl[1] = init_leveling_incl.row(1).mean();
	    meanincl[2] = init_leveling_incl.row(2).mean();
	
	    /** Mean accelerometers value **/
	    meanacc[0] = init_leveling_acc.row(0).mean();
	    meanacc[1] = init_leveling_acc.row(1).mean();
	    meanacc[2] = init_leveling_acc.row(2).mean();

	
	    #ifdef DEBUG_PRINTS
            std::cout<<"*** [FE INERTIAL-SAMPLES]\n";
	    std::cout<<"*** Computed mean acc values: "<<meanacc[0]<<" "<<meanacc[1]<<" "<<meanacc[2]<<"\n";
	    std::cout<<"*** Computed mean incl values: "<<meanincl[0]<<" "<<meanincl[1]<<" "<<meanincl[2]<<"\n";
	    std::cout<<"*** Computed gravity (acc): "<<meanacc.norm()<<"\n";
	    if (framework.use_inclinometers_leveling)
	    {
		std::cout<<"*** Computed gravity (incl): "<<meanincl.norm()<<"\n";
	    }
	    #endif
	
	    if (framework.use_inclinometers_leveling)
	    {
		euler[0] = (double) asin((double)meanincl[1]/ (double)meanincl.norm()); // Roll
		euler[1] = (double) -atan(meanincl[0]/meanincl[2]); //Pitch
		euler[2] = - M_PI;
	    }
	    else
	    {
		euler[0] = (double) asin((double)meanacc[1]/ (double)meanacc.norm()); // Roll
		euler[1] = (double) -atan(meanacc[0]/meanacc[2]); //Pitch
		euler[2] = - M_PI;
	    }

            /** Check the Initial attitude */
            if (base::isNaN<double>(euler[0]) || base::isNaN<double>(euler[1]) || base::isNaN<double>(euler[2]))
                RTT::log(RTT::Fatal)<<"[FRONT-END FATAL ERROR]  Initial Attitude from static leveling is NaN."<<RTT::endlog();

	    /** Set the initial attitude when no initial IMU orientation is provided **/
	    attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
	    Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));

	    #ifdef DEBUG_PRINTS
            std::cout<< "******** [FE INERTIAL-SAMPLES]\n";
	    std::cout<< "******** Initial Attitude (IMU frame)  *******"<<"\n";
	    std::cout<< "Init Roll: "<<euler[0]*localization::R2D<<" Init Pitch: "<<euler[1]*localization::R2D<<" Init Yaw: "<<euler[2]*localization::R2D<<"\n";
	    #endif
	
	    /** Set the quaternion world to imu frame **/
	    q_world2imu = attitude;
	
	    /** This attitude is in the IMU frame. It needs to be expressed in body with the help of the transformer **/
	    attitude = attitude * qtf; //! world2body = world2imu * imu2body
	
	    #ifdef DEBUG_PRINTS
	    euler[2] = attitude.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
	    euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
	    euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
            std::cout<< "******** [FE INERTIAL-SAMPLES]\n";
	    std::cout<< "******** Initial Attitude (after applying qtf)  *******"<<"\n";
	    std::cout<< "Init Roll: "<<euler[0]*localization::R2D<<" Init Pitch: "<<euler[1]*localization::R2D<<" Init Yaw: "<<euler[2]*localization::R2D<<"\n";
	    #endif
	
	    attitude.normalize();
	
	    /** Check if there is initial pose connected **/
	    if (_reference_pose_samples.connected() && initPosition)
	    {
		/** Alternative method: Align the yaw from the poseSamples Yaw **/
		attitude = Eigen::Quaternion <double>(Eigen::AngleAxisd(poseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[0], Eigen::Vector3d::UnitZ())) * attitude;
		
		attitude.normalize();
		
		initAttitude = true;
		
	    }
	    else if (!_reference_pose_samples.connected() || !initPosition)
	    {
		initAttitude = true;
	    }
	
	    /** Set the initial attitude to the rover rbs **/
	    if (initAttitude)
	    {
                double estimated_g = meanacc.norm();

		/** Store the value as the initial one for the poseOut **/
		poseOut.orientation = attitude;
		poseOut.angular_velocity.setZero();
		
		/** Assume very well know initial attitude **/
		poseOut.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
		poseOut.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
		
		/** Gravity error between theoretical gravity and estimated **/
                /** In this framework error = truth - estimation **/
                base::Vector3d g_error;
		g_error << 0.00, 0.00, (inertialState.theoretical_g - estimated_g);
		
		#ifdef DEBUG_PRINTS
                std::cout<< "[FE INERTIAL-SAMPELS] Computed Theoretical gravity: "<<inertialState.theoretical_g<<"\n";
		std::cout<< "[FE INERTIAL-SAMPLES] G_error in world\n"<<g_error <<"\n";
		#endif
		
		/** Gravity error in IMU frame **/
		g_error = q_world2imu.inverse() * g_error; /** g_error_imu = (Tworld_imu)^-1 * g_error_world */
		
		#ifdef DEBUG_PRINTS
		std::cout<< "[FE INERTIAL-SAMPLES] G_error in imu\n"<<g_error <<"\n";
		#endif
		
		g_error << 0.00, 0.00, (inertialState.theoretical_g - estimated_g);

                /** Attitude is Tworld_body (base frame of body expressed in world frame) **/
                /** Therefore the inverse of attitude is needed to have g_error in body frame **/
		g_error = attitude.inverse() * g_error;
		
		#ifdef DEBUG_PRINTS
		std::cout<< "[FE INERTIAL-SAMPLES] G_error in body\n"<<g_error <<"\n";
		#endif

                /** Set up the initial computed gravity into the inertialState variable **/
                inertialState.estimated_g = estimated_g;
		
		/** Set up ON/OFF Bias in the Inertial State **/
                inertialState.gbias_onoff.setZero(); /** Set to zero (calibrated gyros) */

                /** It is true that error = truth - estimation , however in sensor modelling
                 * raw_value = truth_value + bias, therefore truth_value = raw_value - bias
                 * and g_error in body frame goes with negative sign */
                inertialState.abias_onoff = -g_error;
		
		#ifdef DEBUG_PRINTS
                euler[2] = attitude.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
                euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
	        euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
                std::cout<< "******** [FE INERTIAL-SAMPLES]\n";
		std::cout<< "******** Initial Attitude *******"<<"\n";
		std::cout<< "Init Roll: "<<euler[0]*localization::R2D<<" Init Pitch: "<<euler[1]*localization::R2D<<" Init Yaw: "<<euler[2]*localization::R2D<<"\n";
		#endif
	    }
	
	}
    }
    else
    {
	base::samples::IMUSensors imusample;
	
	/** A new sample arrived to the port**/
	
	#ifdef DEBUG_PRINTS
        std::cout<<"** [FE INERTIAL-SAMPLES] counter.imuSamples("<<counter.imuSamples<<") at ("<<inertial_samples_sample.time.toMicroseconds()<< ")**\n";
	std::cout<<"acc(imu_frame):\n"<<inertial_samples_sample.acc<<"\n";
	std::cout<<"acc(quat body_frame ):\n"<<qtf * inertial_samples_sample.acc<<"\n";
	std::cout<<"acc(Rot body_frame):\n"<< tf.rotation() * inertial_samples_sample.acc<<"\n";
	std::cout<<"gyro(imu_frame):\n"<<inertial_samples_sample.gyro<<"\n";
	std::cout<<"gyro(quat body_frame):\n"<<qtf * inertial_samples_sample.gyro<<"\n";
	std::cout<<"mag(imu_frame):\n"<<inertial_samples_sample.mag<<"\n";
        std::cout<<"mag(quat body_frame):\n"<<qtf * inertial_samples_sample.mag<<"\n";
	#endif
	
	/** Convert the IMU values in the body frame **/
	imusample.time = inertial_samples_sample.time;
	imusample.acc = qtf * inertial_samples_sample.acc;
	imusample.gyro = qtf * inertial_samples_sample.gyro;
	imusample.mag = qtf * inertial_samples_sample.mag;

        /** Substract Earth rotation from gyros **/
        localization::Util::SubstractEarthRotation(&(imusample.gyro), &(poseOut.orientation), location.latitude);

        /** Eliminate noise from sensor body frame **/
        imusample.gyro -= backEndEstimationSamples[0].gbias; /** Elminate estimated bias */

        /** Eliminate gravity perturbation from acc in body frame **/
        Eigen::Matrix <double,3,1>  gtilde_body, gtilde;
        gtilde << 0.00, 0.00, inertialState.theoretical_g;
        gtilde_body = poseOut.orientation.inverse() * gtilde;
        imusample.acc = imusample.acc - backEndEstimationSamples[0].abias - gtilde_body;

	/** Push the corrected inertial values into the buffer **/
	cbImuSamples.push_front(imusample);

        #ifdef DEBUG_PRINTS
        std::cout<<"** [FE INERTIAL-SAMPLES] Corrected inertial"<<counter.imuSamples<<") at ("<<inertial_samples_sample.time.toMicroseconds()<< ")**\n";
	std::cout<<"acc(imu_frame):\n"<<imusample.acc<<"\n";
	std::cout<<"gyro(imu_frame):\n"<<imusample.gyro<<"\n";
	std::cout<<"mag(imu_frame):\n"<<imusample.mag<<"\n";
	#endif

	/** Set the flag of IMU values valid to true **/
	if (!flag.imuSamples && (cbImuSamples.size() == cbImuSamples.capacity()))
	    flag.imuSamples = true;
	
	counter.imuSamples++;
    }

}

void FrontEnd::torque_samplesTransformerCallback(const base::Time &ts, const ::torque_estimator::WheelTorques &torque_samples_sample)
{
    throw std::runtime_error("Transformer callback for torque_samples not implemented");
}

void FrontEnd::ground_forces_samplesTransformerCallback(const base::Time &ts, const ::torque_estimator::GroundForces &ground_forces_samples_sample)
{
    throw std::runtime_error("Transformer callback for ground_forces_samples not implemented");
}

void FrontEnd::systemstate_samplesTransformerCallback(const base::Time &ts, const ::sysmon::SystemStatus &systemstate_samples_sample)
{
    /** A new sample arrived to the port **/
    cbAsguardStatusSamples.push_front(systemstate_samples_sample);

    /** Set the flag of Asguard Status values valid to true **/
    if (!flag.asguardStatusSamples && (cbAsguardStatusSamples.size() == cbAsguardStatusSamples.capacity()))
    	flag.asguardStatusSamples = true;

    counter.asguardStatusSamples ++;

    #ifdef DEBUG_PRINTS
    std::cout<<"** [FE PASSIVE-JOINT] counter.asguardStatusSamples("<<counter.asguardStatusSamples<<") at ("<<systemstate_samples_sample.time.toMicroseconds()<< ")**\n";
    std::cout<<"** [FE PASSIVE-JOINT] passive joint value: "<< systemstate_samples_sample.asguardJointEncoder<<"\n";
    #endif

}

void FrontEnd::encoder_samplesTransformerCallback(const base::Time &ts, const ::base::actuators::Status &encoder_samples_sample)
{
    /** Joint encoders, Slip and Contact Angle positions NOTE: The order of the storage needs to be coincident if used as input for the motionModel **/
    Eigen::Matrix< double, frontEndMotionModel::MODEL_DOF, 1  > modelPositions;

    /** Joint encoders, Slip and Contact Angle velocities NOTE: The order of the storage needs to be coincident if used as input for the motionModel **/
    Eigen::Matrix< double, frontEndMotionModel::MODEL_DOF, 1  > modelVelocities;

    /** Linear and Angular velocities NOTE: The order of the storage needs to be coincident if used as input for the motionModel **/
    Eigen::Matrix< double, 6, 1  > cartesianVelocities;

    /** Weigthing matrix for the Motion Model **/
    Eigen::Matrix< double, 6*asguard::NUMBER_OF_WHEELS, 6*asguard::NUMBER_OF_WHEELS  > W;

    /** A new sample arrived to the inport **/
    cbEncoderSamples.push_front(encoder_samples_sample);

    counter.encoderSamples++;

    if (counter.encoderSamples == number.encoderSamples)
    {
	flag.encoderSamples = true;
	counter.encoderSamples = 0;
    }
    else
    {
	flag.encoderSamples = false;
    }
	

    #ifdef DEBUG_PRINTS
    std::cout<<"** [FE ENCODERS-SAMPLES] counter.encoderSamples("<<counter.encoderSamples<<") at ("<<encoder_samples_sample.time.toMicroseconds()
	<<") received FR ("<<encoder_samples_sample.states[0].positionExtern<<")**\n";
    #endif

    #ifdef DEBUG_PRINTS
    std::cout<<"** [FE ENCODERS-SAMPLES] [COUNTERS] encoderCounter ("<<counter.encoderSamples<<") asguardCounter("<<counter.asguardStatusSamples<<") imuCounter("<<counter.imuSamples<<") **\n";
    std::cout<<"** [FE ENCODERS-SAMPLES] [FLAGS] initAttitude ("<<initAttitude<<") initPosition("<<initPosition<<") **\n";
    std::cout<<"** [FE ENCODERS-SAMPLES] [FLAGS] flagEncoders ("<<flag.encoderSamples<<") flagAsguard("<<flag.asguardStatusSamples<<") flagIMU("<<flag.imuSamples<<") **\n";
    #endif

    if (initAttitude && initPosition)
    {
        if (flag.imuSamples && flag.asguardStatusSamples && flag.encoderSamples)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"[ON] ** [FE ENCODERS-SAMPLES] ** [ON] ("<<encoderSamples[0].time.toMicroseconds()<<")\n";
       	    #endif

            /** Get the correct values from the input ports buffers (data types of the inports) **/
            /** and get them in a general form using Eigen (no input port class dependent) **/
            this->inputPortSamples(modelPositions);

            /** Calculate velocities from the input ports to use for the motion model **/
            this->calculateVelocities(cartesianVelocities, modelVelocities);

            /** Update the Motion Model (Forward Kinematics and Contact Points) **/
            this->motionModel.updateKinematics(modelPositions);

            /** Solve the navigation kinematics **/
            this->motionModel.navSolver(modelPositions, cartesianVelocities, modelVelocities,
                                        cartesianVelCov, modelVelCov);

            /** Bessel IIR Low-pass filter of the linear cartesianVelocities from the Motion Model **/
            Eigen::Matrix<double, localization::NUMAXIS, 1> velocity = cartesianVelocities.block<localization::NUMAXIS, 1>(0,0);
            cartesianVelocities.block<localization::NUMAXIS, 1>(0,0) = this->bessel->perform(velocity);

            /** Update the cartesian velocities on the std_vector **/
            this->vectorCartesianVelocities[1] = this->vectorCartesianVelocities[0];
            this->vectorCartesianVelocities[0] = cartesianVelocities;

            /** Perform the velocities integration to get the pose (Dead Reckoning) **/
            base::samples::RigidBodyState deltaPose;
            deltaPose = localization::DeadReckon::updatePose (static_cast<const double>(1.0/framework.frontend_frequency),
                    this->vectorCartesianVelocities, cartesianVelCov, poseOut, poseOut);

            /** Out port the information of the Front-End **/
            this->outputPortSamples (encoderSamples[0].time, modelPositions, cartesianVelocities, modelVelocities, deltaPose);

            /** Reset back the counters and the flags **/
            counter.reset();
            flag.reset();

        }
    }
}

void FrontEnd::backend_estimation_samplesTransformerCallback(const base::Time &ts, const ::rover_localization::BackEndEstimation &backend_estimation_samples_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"** [FE BACK-END] received backend info\n";
    #endif

    /** A new sample arrived to the inport **/
    backEndEstimationSamples.push_front(backend_estimation_samples_sample);

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See FrontEnd.hpp for more detailed
// documentation about them.

bool FrontEnd::configureHook()
{
    if (! FrontEndBase::configureHook())
        return false;

    /************************/
    /** Read configuration **/
    /************************/
    location = _location.value();
    framework = _framework.value();
    propriosensor = _proprioceptive_sensors.value();


    /******************/
    /** Initial Pose **/
    /******************/

    /** Set the initial pose to the Geographic frame **/
    poseOut.invalidate();
    poseOut.sourceFrame = "Body Frame";
    poseOut.targetFrame = "Geographic_Frame (North-West-Up)";

    /** If there is not an external init position **/
    if (!_reference_pose_samples.connected())
    {
	/** set zero position **/
	poseOut.position.setZero();
	poseOut.velocity.setZero();
	poseOut.angular_velocity.setZero();
	
	/** Assume very well know initial attitude **/
	poseOut.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
	poseOut.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
	
	/** Assume well known starting position **/
	poseOut.cov_position = Eigen::Matrix <double, 3 , 3>::Zero();
	poseOut.cov_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
	
	initPosition = true;
    }

    /***********************************************/
    /** Use properties to Configure the Framework **/
    /***********************************************/

    /** Set the initial number of samples for the attitude **/
    if (framework.init_leveling_time != 0.00)
        init_leveling_size = static_cast<int>(framework.init_leveling_time/_inertial_samples_period.value());
    else
        init_leveling_size = 1;

    /** Set the index to Zero **/
    init_leveling_accidx = 0;

    /** Init size for initial acceleration leveling (using accelerometers) **/
    init_leveling_acc.resize (localization::NUMAXIS, init_leveling_size);

    /** Init size for initial acceleration leveling (using inclinometers) **/
    init_leveling_incl.resize (localization::NUMAXIS, init_leveling_size);

    /** Set the Inports counter to Zero **/
    counter.reset();

    /** Set the number of samples between each sensor input (if there are not comming at the same sampling rate) */
    if (framework.frontend_frequency != 0.00)
    {
	number.imuSamples = (1.0/_inertial_samples_period.value())/framework.frontend_frequency;
	number.encoderSamples = (1.0/_encoder_samples_period.value())/framework.frontend_frequency;
	number.asguardStatusSamples = (1.0/_systemstate_samples_period.value())/framework.frontend_frequency;
	number.forceSamples = (1.0/_ground_forces_samples_period.value())/framework.frontend_frequency;
	number.torqueSamples = (1.0/_torque_samples_period.value())/framework.frontend_frequency;
	number.referencePoseSamples = (1.0/_reference_pose_samples_period.value())/framework.frontend_frequency;
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[FE CONFIGURE] cbEncoderSamples has init capacity "<<cbEncoderSamples.capacity()<<" and size "<<cbEncoderSamples.size()<<"\n";
    std::cout<<"[FE CONFIGURE] cbAsguardStatusSamples has init capacity "<<cbAsguardStatusSamples.capacity()<<" and size "<<cbAsguardStatusSamples.size()<<"\n";
    std::cout<<"[FE CONFIGURE] cbImuSamples has init capacity "<<cbImuSamples.capacity()<<" and size "<<cbImuSamples.size()<<"\n";
    #endif

    /** Set the capacity of the circular_buffer according to the sampling rate **/
    cbEncoderSamples.set_capacity(number.encoderSamples);
    cbAsguardStatusSamples.set_capacity(number.asguardStatusSamples);
    cbImuSamples.set_capacity(number.imuSamples);

    for(register unsigned int i=0; i<cbEncoderSamples.size(); ++i)
    {
	cbEncoderSamples[i].resize(asguard::NUMBER_OF_WHEELS);
    }


    #ifdef DEBUG_PRINTS
    std::cout<<"[FE CONFIGURE] cbEncoderSamples has capacity "<<cbEncoderSamples.capacity()<<" and size "<<cbEncoderSamples.size()<<"\n";
    std::cout<<"[FE CONFIGURE] cbAsguardStatusSamples has capacity "<<cbAsguardStatusSamples.capacity()<<" and size "<<cbAsguardStatusSamples.size()<<"\n";
    std::cout<<"[FE CONFIGURE] cbImuSamples has capacity "<<cbImuSamples.capacity()<<" and size "<<cbImuSamples.size()<<"\n";
    #endif

    /** Initialize the samples for the filtered buffer hbridge values **/
    for(register unsigned int i=0;i<encoderSamples.size();i++)
    {
	/** Sizing hbridgeStatus **/
	encoderSamples[i].resize(asguard::NUMBER_OF_WHEELS);

	/** Set to a NaN index **/
	encoderSamples[i].index = base::NaN<unsigned int>();
    }

    /** Initialize the samples for the filtered buffer asguardStatus values **/
    for(register unsigned int i=0; i<asguardStatusSamples.size(); ++i)
    {
	/** Set to NaN passiveJoint **/
	asguardStatusSamples[i].asguardJointEncoder = base::NaN<double>();
    }

    /** Initialize the samples for the filtered buffer imuSamples values **/
    for(register unsigned int i=0; i<imuSamples.size();++i)
    {
	/** IMU Samples **/
	imuSamples[i].acc[0] = base::NaN<double>();
	imuSamples[i].acc[1] = base::NaN<double>();
	imuSamples[i].acc[2] = base::NaN<double>();
	imuSamples[i].gyro = imuSamples[0].acc;
	imuSamples[i].mag = imuSamples[0].acc;
    }

    /** Initialize the samples for the filtered buffer poseSamples values **/
    for(register unsigned int i=0; i<poseSamples.size(); ++i)
    {
	/** Pose Init **/
	poseSamples[i].invalidate();
    }

    /** Initialize the samples for the filtered buffer poseSamples values **/
    for(register unsigned int i=0; i<backEndEstimationSamples.capacity(); ++i)
    {
        rover_localization::BackEndEstimation backend;

        /** Back-End estimation Init **/
	backend.time.fromMicroseconds (base::NaN<uint64_t>());
	backend.statek_i.setZero();
	backend.estatek_i.setZero();
	backend.abias.setZero();
	backend.gbias.setZero();
	backend.Pki.setZero();
	backend.K.setZero();

        /** Push one **/
        backEndEstimationSamples.push_front(backend);
	
    }


    #ifdef DEBUG_PRINTS
    std::cout<<"[FE CONFIGURE] encoderSamples has capacity "<<encoderSamples.capacity()<<" and size "<<encoderSamples.size()<<"\n";
    std::cout<<"[FE CONFIGURE] asguardStatusSamples has capacity "<<asguardStatusSamples.capacity()<<" and size "<<asguardStatusSamples.size()<<"\n";
    std::cout<<"[FE CONFIGURE] imuSamples has capacity "<<imuSamples.capacity()<<" and size "<<imuSamples.size()<<"\n";
    std::cout<<"[FE CONFIGURE] poseSamples has capacity "<<poseSamples.capacity()<<" and size "<<poseSamples.size()<<"\n";
    std::cout<<"[FE CONFIGURE] backEndEstimationSamples has capacity "<<backEndEstimationSamples.capacity()<<" and size "<<backEndEstimationSamples.size()<<"\n";
    #endif

    for (register unsigned int i=0; i<vectorCartesianVelocities.size(); ++i)
    {
        /** cartesian Velocities Init **/
        vectorCartesianVelocities[i].setZero();
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[FE CONFIGURE] vectorCartesianVelocities has capacity "<<vectorCartesianVelocities.capacity()<<" and size "<<vectorCartesianVelocities.size()<<"\n";
    #endif

    /** Gravitational value according to the location. Ideal theoretical value **/
    inertialState.theoretical_g = localization::Util::GravityModel (location.latitude, location.altitude);

    /*********************************************/
    /** Configure the Motion Model of the Robot **/
    /*********************************************/

    /** TO-DO: Make this loading part general for any robot **/

    /** Robot Kinematics Model **/
    robotKinematics.reset(new asguard::AsguardKinematicModel (asguard::NUMBER_OF_WHEELS, asguard::FEET_PER_WHEEL));

    /** Create the Motion Model **/
    bool motionModelStatus;
    motionModel =  frontEndMotionModel(motionModelStatus, frontEndMotionModel::LOWEST_POINT , robotKinematics);

    /** Bessel IIR filter Coefficients **/
    Eigen::Matrix <double, localization::NORDER_BESSEL_FILTER+1, 1> besselBCoeff, besselACoeff;

    besselBCoeff[0] = 0.00467048;
    besselBCoeff[1] = 0.03736385;
    besselBCoeff[2] = 0.13077349;
    besselBCoeff[3] = 0.26154698;
    besselBCoeff[4] = 0.32693372;
    besselBCoeff[5] = 0.26154698;
    besselBCoeff[6] = 0.13077349;
    besselBCoeff[7] = 0.03736385;
    besselBCoeff[8] = 0.00467048;

    besselACoeff[0] = 1.00000000e+00;
    besselACoeff[1] = -3.87747570e-01;
    besselACoeff[2] = 7.13520818e-01;
    besselACoeff[3] = -2.49594003e-01;
    besselACoeff[4] = 1.47736180e-01;
    besselACoeff[5] = -3.59003821e-02;
    besselACoeff[6] = 8.56259334e-03;
    besselACoeff[7] = -9.97047726e-04;
    besselACoeff[8] = 6.27404353e-05;

    /** Create the Bessel Low-pass filter with the right coeffiecients **/
    bessel.reset(new localization::IIR<localization::NORDER_BESSEL_FILTER, localization::NUMAXIS> (besselBCoeff, besselACoeff));

    /*****************************/
    /** Sensor Noise Covariance **/
    /*****************************/
    double sqrtdelta_t; /** Square root of delta time interval */

    /** Select the right delta_t to compute the noise matrices **/
    if ((1.0/framework.frontend_frequency) > (1.0/propriosensor.bandwidth))
        sqrtdelta_t = sqrt((1.0/framework.frontend_frequency));
    else
        sqrtdelta_t = sqrt((1.0/propriosensor.bandwidth));

    /** Angular velocity comming from gyros **/
    cartesianVelCov.setZero(); modelVelCov.setZero();

    cartesianVelCov(3,3) = pow(propriosensor.gyrorw[0]/sqrtdelta_t,2);
    cartesianVelCov(4,4) = pow(propriosensor.gyrorw[1]/sqrtdelta_t,2);
    cartesianVelCov(5,5) = pow(propriosensor.gyrorw[2]/sqrtdelta_t,2);

    /** Encoders reading from the joints **/
    Eigen::Matrix<double, asguard::ASGUARD_JOINT_DOF, 1> encodersNoiseVector;
    encodersNoiseVector = Eigen::Map<const Eigen::Matrix <double, asguard::ASGUARD_JOINT_DOF, 1> >
                        (&(propriosensor.encodersrw[0]), propriosensor.encodersrw.size());
    modelVelCov.block<asguard::ASGUARD_JOINT_DOF, asguard::ASGUARD_JOINT_DOF> (0,0) = encodersNoiseVector.asDiagonal();

    #ifdef DEBUG_PRINTS
    std::cout<<"[FE CONFIGURE] cartesianVelCov:\n"<< cartesianVelCov <<"\n";
    std::cout<<"[FE CONFIGURE] modelVelCov:\n"<< modelVelCov <<"\n";
    #endif


    /*******************************************/
    /** Info and Warnings about the Framework **/
    /*******************************************/

    if (_inertial_samples.connected())
    {
	RTT::log(RTT::Warning) << "[Info Front-End] IMU Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "[Info Front-End] IMU samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "[Info Front-End] Malfunction on the task!!" << RTT::endlog();
    }

    if (_encoder_samples.connected())
    {
	RTT::log(RTT::Warning) << "[Info Front-End] Encoders Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "[Info Front-End] Encoders samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "[Info Front-End] Malfunction on the task!!" << RTT::endlog();
    }

    if (_systemstate_samples.connected())
    {
	RTT::log(RTT::Warning) << "[Info Front-End] System State Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "[Info Front-End] System State samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "[Info Front-End] Malfunction on the task!!" << RTT::endlog();
    }

    if (_torque_samples.connected())
    {
	RTT::log(RTT::Warning) << "[Info Front-End] Wheel Torque samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "[Info Front-End] Wheel Torque samples NO connected." << RTT::endlog();
    }

    if (_ground_forces_samples.connected())
    {
	RTT::log(RTT::Warning) << "[Info Front-End] Wheel ground force estimation samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "[Info Front-End] Wheel ground force estimation samples NO connected." << RTT::endlog();
    }

    if (_reference_pose_samples.connected())
    {
	RTT::log(RTT::Warning) << "[Info Front-End] Initial pose/Ground Truth connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "[Info Front-End] Initial orientation/Ground Truth NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "[Info Front-End] Initial orientation is not provided."<< RTT::endlog();
	RTT::log(RTT::Warning) << "[Info Front-End] Zero Yaw angle pointing to North is then assumed." << RTT::endlog();
	RTT::log(RTT::Warning) << "[Info Front-End] Pitch and Roll are taken from accelerometers assuming static body at Initial phase of this task." << RTT::endlog();
    }

    RTT::log(RTT::Warning)<<"[Info Front-End] Frequency of IMU samples[Hertz]: "<<(1.0/_inertial_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] Frequency of Encoders Samples[Hertz]: "<<(1.0/_encoder_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] Frequency of Asguard Status Samples[Hertz]: "<<(1.0/_systemstate_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] Frequency of Torque Samples[Hertz]: "<<(1.0/_torque_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] Frequency of Ground Force Samples[Hertz]: "<<(1.0/_ground_forces_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] Front-End running at Frequency[Hertz]: "<<framework.frontend_frequency<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] Back-End  running at Frequency[Hertz]: "<<framework.backend_frequency<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] Visualization at Frequency[Hertz]: "<<framework.visualization_frequency<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] Initial leveling time[Seconds]: "<<framework.init_leveling_time<<" which at "<<(1.0)/_inertial_samples_period.value()
        <<" Hertz are "<<init_leveling_size<<" #Samples"<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] number.imuSamples: "<<number.imuSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] number.encoderSamples: "<<number.encoderSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] number.asguardStatusSamples: "<<number.asguardStatusSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] number.forceSamples: "<<number.forceSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info Front-End] number.torqueSamples: "<<number.torqueSamples<<RTT::endlog();

    if ((number.encoderSamples == 0)||(number.asguardStatusSamples == 0)||(number.imuSamples == 0))
    {
        RTT::log(RTT::Warning)<<"[FRONT-END FATAL ERROR] Front-End frequency cannot be higher than proprioceptive sensors frequency."<<RTT::endlog();
        return false;
    }
    else if (!motionModelStatus)
    {
        RTT::log(RTT::Warning)<<"[FRONT-END FATAL ERROR] Motion Model returned false at Configure phase (Constructor)."<<RTT::endlog();
        return false;
    }
    else if (framework.frontend_frequency < framework.backend_frequency)
    {
        RTT::log(RTT::Warning)<<"[FRONT-END FATAL ERROR]  Back-End frequency cannot be higher than Front-End frequency."<<RTT::endlog();
        return false;
    }

    return true;
}

bool FrontEnd::startHook()
{
    if (! FrontEndBase::startHook())
        return false;
    return true;
}
void FrontEnd::updateHook()
{
    FrontEndBase::updateHook();
}
void FrontEnd::errorHook()
{
    FrontEndBase::errorHook();
}
void FrontEnd::stopHook()
{
    FrontEndBase::stopHook();
}
void FrontEnd::cleanupHook()
{
    FrontEndBase::cleanupHook();

    /** Liberate the memory of the shared_ptr **/
    bessel.reset();

}

void FrontEnd::inputPortSamples(Eigen::Matrix< double, frontEndMotionModel::MODEL_DOF, 1  > &modelPositions)
{
    unsigned int cbEncoderSize = cbEncoderSamples.size();
    unsigned int cbAsguardStatusSize = cbAsguardStatusSamples.size();
    unsigned int cbImuSize = cbImuSamples.size();

    /** Local variable of the ports **/
    base::actuators::Status encoder;
    sysmon::SystemStatus asguardStatus;
    base::samples::IMUSensors imu;

    /** sizing the encoders **/
    encoder.resize(asguard::NUMBER_OF_WHEELS);

    #ifdef DEBUG_PRINTS
    std::cout<<"[GetInportValue] cbEncoderSamples has capacity "<<cbEncoderSamples.capacity()<<" and size "<<cbEncoderSamples.size()<<"\n";
    std::cout<<"[GetInportValue] cbAsguardStatusSamples has capacity "<<cbAsguardStatusSamples.capacity()<<" and size "<<cbAsguardStatusSamples.size()<<"\n";
    std::cout<<"[GetInportValue] cbImuSamples has capacity "<<cbImuSamples.capacity()<<" and size "<<cbImuSamples.size()<<"\n";
    #endif

    /** ********* **/
    /** Encoders  **/
    /** ********* **/
    for (unsigned int i = 0; i<asguard::NUMBER_OF_WHEELS; ++i)
    {
	encoder.states[i].current = 0.0;
	encoder.states[i].position = 0.0;
	encoder.states[i].positionExtern = 0.0;
	encoder.states[i].pwm = 0.0;
    }
	
    /** Process the buffer **/
    for (register unsigned int i = 0; i<cbEncoderSize; ++i)
    {
	for (register unsigned int j = 0; j<asguard::NUMBER_OF_WHEELS; ++j)
	{
	    encoder.states[j].current += cbEncoderSamples[i].states[j].current;
	    encoder.states[j].pwm += cbEncoderSamples[i].states[j].pwm;
	}
    }

    /** Set the time **/
    encoder.time = (cbEncoderSamples[cbEncoderSize-1].time + cbEncoderSamples[0].time)/2.0;

    encoder.index = cbEncoderSamples[0].index;

    for (register unsigned int i = 0; i<asguard::NUMBER_OF_WHEELS; ++i)
    {
	encoder.states[i].current /= cbEncoderSize; //the mean current in mA
	encoder.states[i].position = cbEncoderSamples[0].states[i].position;
	encoder.states[i].positionExtern = cbEncoderSamples[0].states[i].positionExtern;
	encoder.states[i].pwm /= cbEncoderSize; //the mean PWM signal duty cicle [-1,1]
    }

    /** Push the result in the buffer **/
    encoderSamples.push_front(encoder);
	
    /** ************** **/
    /** Asguard status **/
    /** ************** **/
    asguardStatus.asguardVoltage = 0.00;

    /** Process the buffer **/
    for (register unsigned int i=0; i<cbAsguardStatusSize; ++i)
    {
	asguardStatus.asguardVoltage += cbAsguardStatusSamples[i].asguardVoltage;
    }

    asguardStatus.asguardJointEncoder = cbAsguardStatusSamples[0].asguardJointEncoder;
    asguardStatus.systemState = cbAsguardStatusSamples[0].systemState;
    asguardStatus.packetsPerSec = cbAsguardStatusSamples[0].packetsPerSec;
    asguardStatus.controlPacketsPerSec = cbAsguardStatusSamples[0].controlPacketsPerSec;

    /** Set the time **/
    asguardStatus.time = (cbAsguardStatusSamples[cbAsguardStatusSize-1].time + cbAsguardStatusSamples[0].time)/2.0;

    /** Set the voltage **/
    asguardStatus.asguardVoltage /= cbAsguardStatusSize; //the mean voltage

    /** Push the result into the buffer **/
    asguardStatusSamples.push_front(asguardStatus);

    /** ************ **/
    /** IMU samples **/
    /** ************ **/
    imu.acc.setZero();
    imu.gyro.setZero();
    imu.mag.setZero();

    /** Process the buffer **/
    for (register unsigned int i=0; i<cbImuSize; ++i)
    {
	imu.acc += cbImuSamples[i].acc;
	imu.gyro += cbImuSamples[i].gyro;
	imu.mag += cbImuSamples[i].mag;
    }

    /** Set the time **/
    imu.time = (cbImuSamples[cbImuSize-1].time + cbImuSamples[0].time)/2.0;

    /** Set the mean of this time interval **/
    imu.acc /= cbImuSize;
    imu.gyro /= cbImuSize;
    imu.mag /= cbImuSize;

    /** Push the result into the buffer **/
    imuSamples.push_front(imu);

    /******************************************************************/
    /** Store the Joint values into the correspondent modelPositions **/
    /******************************************************************/
    modelPositions.setZero(); // Initialize
    modelPositions[0] = asguardStatusSamples[0].asguardJointEncoder;
    for (register int i=0; i<static_cast<int> (asguard::NUMBER_OF_WHEELS); ++i)
    {
        modelPositions[i+1] = encoderSamples[0].states[i].positionExtern;
    }


    /** Set all counters to zero **/
    counter.reset();

    return;
}

void FrontEnd::calculateVelocities(Eigen::Matrix< double, 6, 1  > &cartesianVelocities, Eigen::Matrix< double, frontEndMotionModel::MODEL_DOF, 1  > &modelVelocities)
{
    double delta_t = (1.0/framework.frontend_frequency);
    base::Time encoderDelta_t = encoderSamples[0].time - encoderSamples[1].time;
    base::Time asguardStatusDelta_t = asguardStatusSamples[0].time - asguardStatusSamples[1].time;
    base::Time imuDelta_t = imuSamples[0].time - imuSamples[1].time;
    Eigen::Matrix<double, Eigen::Dynamic, 1> derivationEncoderSamples; //!Local variable for the derivation of the encoders
    Eigen::Matrix<double, Eigen::Dynamic, 1> derivationAsguardStatusSamples; //!Local variable for the derivation of Asguard Status (pasive Joint)

    /** At least two values to perform the derivative **/
    if (static_cast<int>(encoderSamples.size()) < 2)
    {
	cartesianVelocities.setZero();
        modelVelocities.setZero();
    }
    else
    {
        /** Set the correct size for the derivative data vectors **/
	derivationEncoderSamples.resize(static_cast<int>(encoderSamples.size()),1);
	derivationAsguardStatusSamples.resize(static_cast<int>(encoderSamples.size()),1);

        /** Set the initial derivation vector **/
        derivationAsguardStatusSamples.setZero();
	
	#ifdef DEBUG_PRINTS
        std::cout<<"[FE CALCULATING_VELO] ********************************************* \n";
	std::cout<<"[FE CALCULATING_VELO] Timestamp New(asguardStatus): "<< encoderSamples[0].time.toMicroseconds() <<" Timestamp Prev: "<<encoderSamples[1].time.toMicroseconds()<<"\n";
	std::cout<<"[FE CALCULATING_VELO] Delta time(asguardStatus): "<< asguardStatusDelta_t.toSeconds()<<"\n";
	std::cout<<"[FE CALCULATING_VELO] Timestamp New(IMU): "<< imuSamples[0].time.toMicroseconds() <<" Timestamp Prev: "<<imuSamples[1].time.toMicroseconds()<<"\n";
	std::cout<<"[FE CALCULATING_VELO] Delta time(IMU): "<< imuDelta_t.toSeconds()<<"\n";
	std::cout<<"[FE CALCULATING_VELO] asguardStatus(passive Joint): "<< asguardStatusSamples[0].asguardJointEncoder <<" Prev: "<<asguardStatusSamples[1].asguardJointEncoder<<"\n";
	#endif

	/** Fill the derivative vector **/
	for (register int i=0; i< static_cast<int>(encoderSamples.size()); ++i)
	{
	    derivationAsguardStatusSamples[i] = asguardStatusSamples[i].asguardJointEncoder;
	}
	
	#ifdef DEBUG_PRINTS
	std::cout<<"[FE CALCULATING_VELO] passiveJoint old velocity: "<<(asguardStatusSamples[0].asguardJointEncoder - asguardStatusSamples[1].asguardJointEncoder)/delta_t<<"\n";
	#endif
		
	/** Passive joint velocity **/
	modelVelocities[0] = localization::Util::finiteDifference (derivationAsguardStatusSamples, delta_t); //passive joints speed
	
	#ifdef DEBUG_PRINTS
	std::cout<<"[FE CALCULATING_VELO] passiveJoint new velocity: "<<modelVelocities[0]<<"\n";
	#endif

	/** Velocities for the vector **/
	for (register int i = 0; i<static_cast<int>(asguard::NUMBER_OF_WHEELS); ++i)
	{
	    #ifdef DEBUG_PRINTS
	    std::cout<<"[FE CALCULATING_VELO] Timestamp New(encoders): "<< encoderSamples[0].time.toMicroseconds() <<" Timestamp Prev: "<<encoderSamples[1].time.toMicroseconds()<<"\n";
	    std::cout<<"[FE CALCULATING_VELO] Delta time(encoders): "<< encoderDelta_t.toSeconds()<<"\n";
	    std::cout<<"[FE CALCULATING_VELO] ["<<i<<"] New: "<< encoderSamples[0].states[i].positionExtern <<" Prev: "<<encoderSamples[1].states[i].positionExtern<<"\n";
	    #endif
	
	    derivationEncoderSamples.setZero();//!Set to zero
	
	    /** Fill the derivative vector **/
	    for (register int j=0; j<static_cast<int>(encoderSamples.size()); ++j)
	    {
		derivationEncoderSamples[j] = encoderSamples[j].states[i].positionExtern;
	    }
	
	    #ifdef DEBUG_PRINTS
	    std::cout<<"[FE CALCULATING_VELO] ["<<i<<"] encoderSamples old velocity: "<<(encoderSamples[0].states[i].positionExtern - encoderSamples[1].states[i].positionExtern)/delta_t<<"\n";
	    #endif
	
	    /** Motor joint velocity **/
	    modelVelocities[i+1] = localization::Util::finiteDifference(derivationEncoderSamples, delta_t); //!wheel rotation speed
	
	    #ifdef DEBUG_PRINTS
	    std::cout<<"[FE CALCULATING_VELO] ["<<i<<"] encoderSamples new velocity: "<<modelVelocities[i+1]<<"\n";
	    #endif

	}

        /** Fill the rest of modelVelocities (unknow quantities) **/
        modelVelocities.block<asguard::NUMBER_OF_WHEELS*asguard::SLIP_VECTOR_SIZE,1> (asguard::ASGUARD_JOINT_DOF, 0) = Eigen::Matrix<double, asguard::NUMBER_OF_WHEELS*asguard::SLIP_VECTOR_SIZE, 1>::Identity() * base::NaN<double>();
        modelVelocities.block<asguard::NUMBER_OF_WHEELS*asguard::CONTACT_POINT_DOF,1> (asguard::ASGUARD_JOINT_DOF+(asguard::NUMBER_OF_WHEELS*asguard::SLIP_VECTOR_SIZE), 0) = Eigen::Matrix<double, asguard::NUMBER_OF_WHEELS*asguard::CONTACT_POINT_DOF, 1>::Identity() * base::NaN<double>();

        /** Fill the cartesian Velocities **/
        cartesianVelocities.block<3,1> (0,0) = Eigen::Matrix<double, 3, 1>::Identity() * base::NaN<double>();
        cartesianVelocities.block<3,1> (3,0) = imuSamples[0].gyro;//!Angular velocities come from gyros
	
        /** TO-DO: remove this debug info ports **/
	_angular_position.write(encoderSamples[0].states[3].positionExtern);
	_angular_rate.write(modelVelocities[3]); //!Front Left
	_angular_rate_old.write((encoderSamples[0].states[3].positionExtern - encoderSamples[1].states[3].positionExtern)/delta_t);
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[FE CALCULATING_VELO]: Model\n"<<modelVelocities<<"\n";
    std::cout<<"[FE CALCULATING_VELO]: Cartesian\n"<<cartesianVelocities<<"\n";
    std::cout<<"[FE CALCULATING_VELO] ******************** END ******************** \n";
    #endif

    return;
}

void FrontEnd::outputPortSamples(const base::Time &timestamp,
                                const Eigen::Matrix< double, frontEndMotionModel::MODEL_DOF, 1  > &modelPositions,
                                const Eigen::Matrix< double, 6, 1  > &cartesianVelocities,
                                const Eigen::Matrix< double, frontEndMotionModel::MODEL_DOF, 1  > &modelVelocities,
                                const base::samples::RigidBodyState &deltaPose)
{
    std::vector<Eigen::Affine3d> fkRobot;
    RobotContactPoints robotChains;

    /***************************************/
    /** Port out the OutPorts information **/
    /***************************************/

    /** The Front-End Estimated pose **/
    poseOut.time = timestamp;
    _pose_samples_out.write(poseOut);

    /** Ground Truth if available **/
    if (_reference_pose_samples.connected())
    {
        /** Port Out the info comming from the ground truth **/
        referenceOut = poseSamples[0];
        referenceOut.velocity = poseOut.orientation.inverse() * poseSamples[0].velocity; //velocity in body frame
        referenceOut.time = timestamp;
        _reference_pose_samples_out.write(referenceOut);

        /** Delta increments of the ground truth at delta_t given by the frontend_frequency **/
        referenceOut.position = poseSamples[0].position - poseSamples[1].position;
        referenceOut.cov_position = poseSamples[0].cov_position - poseSamples[1].cov_position;
        referenceOut.velocity = poseOut.orientation.inverse() * (poseSamples[0].velocity - poseSamples[1].velocity);//in body frame
        referenceOut.cov_velocity = poseOut.orientation.inverse() * (poseSamples[0].cov_velocity - poseSamples[1].cov_velocity);
        _incre_reference_pose_samples_out.write(referenceOut);
    }

    /** Proprioceptive sensors (IMU) **/
    inertialState.time = timestamp;
    inertialState.delta_orientation = deltaPose.orientation; /** Delta orientation from IMU */
    inertialState.acc = imuSamples[0].acc; /** Bias and gravity corrected acceleration */
    inertialState.delta_vel = imuSamples[0].acc * (1.0/framework.frontend_frequency); /** Delta velocity (acc integration) in body frame */
    inertialState.gyro = imuSamples[0].gyro; /** Earth rotation corrected angular velocity */
    inertialState.incl = imuSamples[0].mag; /** Raw inclinometers values (with gravity perturbation) */
    _inertial_samples_out.write(inertialState);


    /** The Forward Kinematics information **/
    if (_fkchains_samples_out.connected())
    {
        robotChains.time = timestamp;//time stamp
        this->motionModel.getKinematics(fkRobot, robotChains.cov);//Get the Forward Kinematics from the model

        robotChains.chain.resize(fkRobot.size());
        for (std::vector<int>::size_type i = 0; i < fkRobot.size(); ++i)
        {
            robotChains.chain[i] = fkRobot[i].matrix();
        }

        /** Joints positions in std_vector form **/
        robotChains.modelPositions.resize(modelPositions.size());
        Eigen::Map <Eigen::Matrix <double, frontEndMotionModel::MODEL_DOF, 1> > (&(robotChains.modelPositions[0]), frontEndMotionModel::MODEL_DOF) = modelPositions;

        /** Contact Points **/
        robotChains.contactPoints = this->motionModel.getPointsInContact();

        _fkchains_samples_out.write(robotChains);
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[FE OUTPUT_PORTS]: poseOut.position\n"<<poseOut.position<<"\n";
    std::cout<<"[FE OUTPUT_PORTS]: poseOut.velocity\n"<<poseOut.velocity<<"\n";
    std::cout<<"[FE OUTPUT_PORTS]: referenceOut.position\n"<<referenceOut.velocity<<"\n";
    std::cout<<"[FE OUTPUT_PORTS] ******************** END ******************** \n";
    #endif
    /** Store the Debug OutPorts information **/

    return;
}
