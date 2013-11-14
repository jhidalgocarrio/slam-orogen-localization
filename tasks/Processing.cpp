/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Processing.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

//#define DEBUG_PRINTS 1

using namespace rover_localization;

Processing::Processing(std::string const& name)
    : ProcessingBase(name)
{

    /******************************/
    /*** Control Flow Variables ***/
    /******************************/
    initPosition = false;
    initAttitude = false;

    counter.reset();
    number.reset();
    flag.reset();

    /***************************/
    /** Output port variables **/
    /***************************/
    world2navigationRbs.invalidate();
    referenceOut.invalidate();
    inertialState.time.fromMicroseconds(base::NaN<uint64_t>());
    inertialState.theoretical_g = base::NaN<double>();
    inertialState.estimated_g = base::NaN<double>();
    inertialState.abias_onoff = base::NaN<double>() * base::Vector3d::Zero();
    inertialState.gbias_onoff = base::NaN<double>() * base::Vector3d::Zero();

    /**********************************/
    /*** Internal Storage Variables ***/
    /**********************************/

    /** Default size for initial acceleration leveling **/
    init_leveling_acc.resize (3, DEFAULT_INIT_LEVELING_SIZE);

    /** Default size for initial acceleration leveling **/
    init_leveling_incl.resize (3, DEFAULT_INIT_LEVELING_SIZE);

    /** Default size for the circular_buffer of the raw port samples **/
    cbEncoderSamples = boost::circular_buffer<base::actuators::Status>(DEFAULT_CIRCULAR_BUFFER_SIZE);
    cbAsguardStatusSamples = boost::circular_buffer<sysmon::SystemStatus> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    cbImuSamples = boost::circular_buffer<base::samples::IMUSensors> (DEFAULT_CIRCULAR_BUFFER_SIZE);

    /** Default size for the circular_buffer for the filtered port samples **/
    encoderSamples = boost::circular_buffer<base::actuators::Status>(DEFAULT_CIRCULAR_BUFFER_SIZE);
    asguardStatusSamples = boost::circular_buffer<sysmon::SystemStatus> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    imuSamples = boost::circular_buffer<base::samples::IMUSensors> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    poseSamples = boost::circular_buffer<base::samples::RigidBodyState> (DEFAULT_CIRCULAR_BUFFER_SIZE);

}

Processing::Processing(std::string const& name, RTT::ExecutionEngine* engine)
    : ProcessingBase(name, engine)
{
}

Processing::~Processing()
{
}

void Processing::reference_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &reference_pose_samples_sample)
{
    poseSamples.push_front(reference_pose_samples_sample);

    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation in quaternion form **/

    /** Get the transformation **/
    if (!_reference2body.get(ts, tf, false))
	return;

    qtf = Eigen::Quaternion <double> (tf.rotation());

    #ifdef DEBUG_PRINTS
    std::cout<<"** [PROCESSING REFERENCE-POSE]Received poseSamples sample at("<<reference_pose_samples_sample.time.toMicroseconds()<<") **\n";
    #endif

    /** Apply the transformer pose offset **/
    poseSamples[0].position += qtf * tf.translation();
    poseSamples[0].orientation = poseSamples[0].orientation * qtf; //world_2_body = world_2_reference * reference_2_body
    /*poseSamples[0].velocity = qtf.inverse() * poseSamples[0].velocity;
    poseSamples[0].cov_velocity = tf.rotation().inverse() * poseSamples[0].cov_velocity;
    poseSamples[0].cov_angular_velocity = tf.rotation().inverse() * poseSamples[0].cov_angular_velocity;*/

    if (!initPosition)
    {
	world2navigationRbs.position = poseSamples[0].position;
	world2navigationRbs.velocity.setZero();
	
	/** Assume well known starting position **/
	world2navigationRbs.cov_position = Eigen::Matrix3d::Zero();
	world2navigationRbs.cov_velocity = Eigen::Matrix3d::Zero();
	
	#ifdef DEBUG_PRINTS
	Eigen::Matrix <double,3,1> euler; /** In euler angles **/
	euler[2] = poseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
	euler[1] = poseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
	euler[0] = poseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
 	std::cout<<"** [PROCESSING REFERENCE-POSE]poseSamples at ("<<poseSamples[0].time.toMicroseconds()<< ")**\n";
	std::cout<<"** position offset\n"<<tf.translation()<<"\n";
	std::cout<<"** rotation offset\n"<<tf.rotation()<<"\n";
	std::cout<<"** position\n"<< poseSamples[0].position<<"\n";
	std::cout<<"** Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
	#endif

	/** Initial attitude from IMU acceleration has been already calculated **/
	if (initAttitude)
	{
	    Eigen::Matrix <double,3,1> euler; /** In euler angles **/
            Eigen::Quaternion <double> attitude = world2navigationRbs.orientation; /** Initial attitude from accelerometers has been already calculated **/
	
	    /** Get the initial Yaw from the initial Pose and the pitch and roll **/
	    euler[2] = poseSamples[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
	    euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
	    euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
	
            /** Check the Initial attitude */
            if (base::isNaN<double>(euler[0]) || base::isNaN<double>(euler[1]) || base::isNaN<double>(euler[2]))
                RTT::log(RTT::Fatal)<<"[FATAL ERROR]  Initial Attitude from External Reference is NaN."<<RTT::endlog();

	    /** Set the initial attitude with the Yaw provided from the initial pose **/
	    attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
	    Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
	
	    /**Store the value as the initial one for the world2navigationRbs **/
	    world2navigationRbs.orientation = attitude;
	
	    #ifdef DEBUG_PRINTS
            std::cout<< "[PROCESSING REFERENCE-POSE]\n";
	    std::cout<< "******** Initial Attitude in Pose Init Samples *******"<<"\n";
	    std::cout<< "Init Roll: "<<euler[0]*R2D<<"Init Pitch: "<<euler[1]*R2D<<"Init Yaw: "<<euler[2]*R2D<<"\n";
	    #endif
	}
	
	/** Initial angular velocity **/
	world2navigationRbs.angular_velocity.setZero();
	
	/** Assume very well know initial attitude **/
	world2navigationRbs.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
	world2navigationRbs.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
		
	initPosition = true;
    }


   flag.referencePoseSamples = true;
}

void Processing::inertial_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation in quaternion form **/

    /** Get the transformation **/
    if (!_imu2body.get(ts, tf, false))
	return;

    qtf = Eigen::Quaternion <double> (tf.rotation());//!Quaternion from IMU to Body

    /** If the initial attitude is not defined **/
    if (!initAttitude)
    {
	#ifdef DEBUG_PRINTS
	std::cout<< "[PROCESSING INERTIAL-SAMPLES]Calculating initial level position since Init orientation is not provided.["<<init_leveling_accidx<<"]\n";
	#endif
	
	/** Add one acc sample to the buffer **/
	init_leveling_acc.col(init_leveling_accidx) = inertial_samples_sample.acc;
	
	/** Add one acc(inclinometer) sample to the buffer **/
	init_leveling_incl.col(init_leveling_accidx) = inertial_samples_sample.mag; //!TO-DO: still incl values come into the mag field of the class
	
	init_leveling_accidx++;
	
	/** Check if the number of stores acceleration corresponds to the initial leveling **/
	if (init_leveling_accidx == this->init_leveling_size)
	{
	    Eigen::Matrix <double,3,1> meanacc; /** Mean value of accelerometers **/
	    Eigen::Matrix <double,3,1> meanincl; /** Mean value of inclinometer **/
	    Eigen::Matrix <double,3,1> euler; /** In euler angles **/
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
            std::cout<<"*** [PROCESSING INERTIAL-SAMPLES]\n";
	    std::cout<<"*** Computed mean acc values: "<<meanacc[0]<<" "<<meanacc[1]<<" "<<meanacc[2]<<"\n";
	    std::cout<<"*** Computed mean incl values: "<<meanincl[0]<<" "<<meanincl[1]<<" "<<meanincl[2]<<"\n";
	    std::cout<<"*** Computed gravity (acc): "<<meanacc.norm()<<"\n";
	    if (config.use_inclinometers_leveling)
	    {
		std::cout<<"*** Computed gravity (incl): "<<meanincl.norm()<<"\n";
	    }
	    #endif
	
	    if (config.use_inclinometers_leveling)
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
                RTT::log(RTT::Fatal)<<"[FATAL ERROR]  Initial Attitude from static leveling is NaN."<<RTT::endlog();

	    /** Set the initial attitude when no initial IMU orientation is provided **/
	    attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
	    Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));

	    #ifdef DEBUG_PRINTS
            std::cout<< "******** [PROCESSING INERTIAL-SAMPLES]\n";
	    std::cout<< "******** Initial Attitude (IMU frame)  *******"<<"\n";
	    std::cout<< "Init Roll: "<<euler[0]*R2D<<" Init Pitch: "<<euler[1]*R2D<<" Init Yaw: "<<euler[2]*R2D<<"\n";
	    #endif
	
	    /** Set the quaternion world to imu frame **/
	    q_world2imu = attitude;
	
	    /** This attitude is in the IMU frame. It needs to be expressed in body with the help of the transformer **/
	    attitude = attitude * qtf; //! world2body = world2imu * imu2body
	
	    #ifdef DEBUG_PRINTS
	    euler[2] = attitude.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
	    euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
	    euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
            std::cout<< "******** [PROCESSING INERTIAL-SAMPLES]\n";
	    std::cout<< "******** Initial Attitude (after applying qtf)  *******"<<"\n";
	    std::cout<< "Init Roll: "<<euler[0]*R2D<<" Init Pitch: "<<euler[1]*R2D<<" Init Yaw: "<<euler[2]*R2D<<"\n";
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
	
	    /** Set the initial attitude to the world to navigation transform **/
	    if (initAttitude)
	    {
                double estimated_g = meanacc.norm();

		/** Store the value as the initial one for the world to navigation **/
		world2navigationRbs.orientation = attitude;
		world2navigationRbs.angular_velocity.setZero();
		
		/** Assume very well know initial attitude **/
		world2navigationRbs.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
		world2navigationRbs.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
		
		/** Gravity error between theoretical gravity and estimated **/
                /** In this framework error = truth - estimation **/
                base::Vector3d gravity;
		gravity << 0.00, 0.00, (inertialState.theoretical_g);
		
		#ifdef DEBUG_PRINTS
                std::cout<< "[PROCESSING INERTIAL-SAMPLES] Computed Theoretical gravity: "<<inertialState.theoretical_g<<"\n";
		std::cout<< "[PROCESSING INERTIAL-SAMPLES] Gravity in world\n"<<gravity <<"\n";
		#endif
		
		/** Gravity in IMU frame **/
		gravity = q_world2imu.inverse() * gravity; /** g_error_imu = (Tworld_imu)^-1 * g_error_world */
		
		#ifdef DEBUG_PRINTS
		std::cout<< "[PROCESSING INERTIAL-SAMPLES] Gravity in imu\n"<<gravity <<"\n";
		#endif
		
		gravity << 0.00, 0.00, (inertialState.theoretical_g);

                /** Attitude is Tworld_body (base frame of body expressed in world frame) **/
                /** Therefore the inverse of attitude is needed to have g_error in body frame **/
		gravity = attitude.inverse() * gravity;
		
		#ifdef DEBUG_PRINTS
		std::cout<< "[PROCESSING INERTIAL-SAMPLES] Gravity in body\n"<<gravity <<"\n";
		#endif

                /** Set up the initial computed gravity into the inertialState variable **/
                inertialState.estimated_g = estimated_g;
		
		/** Set up ON/OFF Bias in the Inertial State **/
                inertialState.gbias_onoff.setZero(); /** Set to zero (calibrated gyros) */

                /** It is true that error = truth - estimation , however in sensor modelling
                 * raw_value = truth_value + bias, therefore truth_value = raw_value - bias
                 * and g_error in body frame goes with negative sign */
                inertialState.abias_onoff = - (gravity - (qtf.inverse() * meanacc));

		#ifdef DEBUG_PRINTS
		std::cout<< "[PROCESSING INERTIAL-SAMPLES] ON-OFF bias:\n"<<inertialState.abias_onoff<<"\n";
		#endif

		#ifdef DEBUG_PRINTS
                euler[2] = attitude.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
                euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
	        euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
                std::cout<< "******** [PROCESSING INERTIAL-SAMPLES]\n";
		std::cout<< "******** Initial Attitude *******"<<"\n";
		std::cout<< "Init Roll: "<<euler[0]*R2D<<" Init Pitch: "<<euler[1]*R2D<<" Init Yaw: "<<euler[2]*R2D<<"\n";
		#endif
	    }
	
	}
    }
    else
    {
	base::samples::IMUSensors imusample;
	
	/** A new sample arrived to the port**/
	
	#ifdef DEBUG_PRINTS
        std::cout<<"** [PROCESSING INERTIAL-SAMPLES] counter.imuSamples("<<counter.imuSamples<<") at ("<<inertial_samples_sample.time.toMicroseconds()<< ")**\n";
	std::cout<<"acc(imu_frame):\n"<<inertial_samples_sample.acc<<"\n";
	std::cout<<"acc(quat body_frame ):\n"<<qtf.inverse() * inertial_samples_sample.acc<<"\n";
	std::cout<<"acc(Rot body_frame):\n"<< tf.rotation().inverse() * inertial_samples_sample.acc<<"\n";
	std::cout<<"gyro(imu_frame):\n"<<inertial_samples_sample.gyro<<"\n";
	std::cout<<"gyro(quat body_frame):\n"<<qtf.inverse() * inertial_samples_sample.gyro<<"\n";
	std::cout<<"mag(imu_frame):\n"<<inertial_samples_sample.mag<<"\n";
        std::cout<<"mag(quat body_frame):\n"<<qtf.inverse() * inertial_samples_sample.mag<<"\n";
	#endif

        /** If a new sample arrived to the input port of the Back-End Feedback **/
        if (_state_estimation_samples.read(stateEstimationSamples) == RTT::NewData)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"** [PROCESSING BACK-END-INFO] at "<< stateEstimationSamples.time.toMicroseconds()<<"\n";
            std::cout<<"** [PROCESSING BACK-END-INFO] gbias:\n"<<stateEstimationSamples.gbias<< "\n";
            std::cout<<"** [PROCESSING BACK-END-INFO] abias:\n"<<stateEstimationSamples.abias<< "\n";
            Eigen::Vector3d euler;
            euler[2] = stateEstimationSamples.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
            euler[1] = stateEstimationSamples.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
            euler[0] = stateEstimationSamples.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
            std::cout<<"** [PROCESSING BACK-END-INFO] Roll: "<<euler[0]*R2D<<" Init Pitch: "<<euler[1]*R2D<<" Init Yaw: "<<euler[2]*R2D<<"\n";
            #endif
        }

	/** Convert the IMU values in the body frame **/
	imusample.time = inertial_samples_sample.time;
	imusample.acc = qtf.inverse() * inertial_samples_sample.acc;
	imusample.gyro = qtf.inverse() * inertial_samples_sample.gyro;
	imusample.mag = qtf.inverse() * inertial_samples_sample.mag;

        /** Subtract Earth rotation from gyros **/
        localization::Util::SubtractEarthRotation(&(imusample.gyro), &(world2navigationRbs.orientation), location.latitude);

        /** Eliminate noise from sensor body frame **/
        imusample.gyro -= stateEstimationSamples.gbias; /** Eliminate estimated bias */

        /** Eliminate gravity perturbation and bias from accelerometers values in body frame **/
        Eigen::Matrix <double,3,1>  gtilde_body, gtilde;
        gtilde << 0.00, 0.00, inertialState.theoretical_g;
        gtilde_body = stateEstimationSamples.orientation.inverse() * gtilde;//!gravity in body frame
        imusample.acc = imusample.acc - stateEstimationSamples.abias - gtilde_body;

	/** Push the corrected inertial values into the buffer **/
	cbImuSamples.push_front(imusample);

        #ifdef DEBUG_PRINTS
        std::cout<<"** [PROCESSING INERTIAL-SAMPLES] Corrected inertial ("<<counter.imuSamples<<") at ("<<inertial_samples_sample.time.toMicroseconds()<< ")**\n";
	std::cout<<"acc(body_frame):\n"<<imusample.acc<<"\n";
	std::cout<<"gyro(body_frame):\n"<<imusample.gyro<<"\n";
	std::cout<<"mag(body_frame):\n"<<imusample.mag<<"\n";
	#endif

	/** Set the flag of IMU values valid to true **/
	if (!flag.imuSamples && (cbImuSamples.size() == cbImuSamples.capacity()))
	    flag.imuSamples = true;
	
	counter.imuSamples++;
    }

}

void Processing::torque_samplesTransformerCallback(const base::Time &ts, const ::torque_estimator::WheelTorques &torque_samples_sample)
{
    throw std::runtime_error("Transformer callback for torque_samples not implemented");
}

void Processing::ground_forces_samplesTransformerCallback(const base::Time &ts, const ::torque_estimator::GroundForces &ground_forces_samples_sample)
{
    throw std::runtime_error("Transformer callback for ground_forces_samples not implemented");
}

void Processing::systemstate_samplesTransformerCallback(const base::Time &ts, const ::sysmon::SystemStatus &systemstate_samples_sample)
{
    /** A new sample arrived to the port **/
    cbAsguardStatusSamples.push_front(systemstate_samples_sample);

    /** Set the flag of Asguard Status values valid to true **/
    if (!flag.asguardStatusSamples && (cbAsguardStatusSamples.size() == cbAsguardStatusSamples.capacity()))
    	flag.asguardStatusSamples = true;

    counter.asguardStatusSamples ++;

    #ifdef DEBUG_PRINTS
    std::cout<<"** [PROCESSING PASSIVE-JOINT] counter.asguardStatusSamples("<<counter.asguardStatusSamples<<") at ("<<systemstate_samples_sample.time.toMicroseconds()<< ")**\n";
    std::cout<<"** [PROCESSING PASSIVE-JOINT] passive joint value: "<< systemstate_samples_sample.asguardJointEncoder<<"\n";
    #endif

}

void Processing::encoder_samplesTransformerCallback(const base::Time &ts, const ::base::actuators::Status &encoder_samples_sample)
{
    /** A new sample arrived to the Input port **/
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
    std::cout<<"** [PROCESSING ENCODERS-SAMPLES] counter.encoderSamples("<<counter.encoderSamples<<") at ("<<encoder_samples_sample.time.toMicroseconds()
	<<") received FR ("<<encoder_samples_sample.states[0].positionExtern<<")**\n";
    #endif

    #ifdef DEBUG_PRINTS
    std::cout<<"** [PROCESSING ENCODERS-SAMPLES] [COUNTERS] encoderCounter ("<<counter.encoderSamples<<") asguardCounter("<<counter.asguardStatusSamples<<") imuCounter("<<counter.imuSamples<<") **\n";
    std::cout<<"** [PROCESSING ENCODERS-SAMPLES] [FLAGS] initAttitude ("<<initAttitude<<") initPosition("<<initPosition<<") **\n";
    std::cout<<"** [PROCESSING ENCODERS-SAMPLES] [FLAGS] flagEncoders ("<<flag.encoderSamples<<") flagAsguard("<<flag.asguardStatusSamples<<") flagIMU("<<flag.imuSamples<<") **\n";
    #endif

    if (initAttitude && initPosition)
    {
        if (flag.imuSamples && flag.asguardStatusSamples && flag.encoderSamples)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"[ON] ** [PROCESSING ENCODERS-SAMPLES] ** [ON] ("<<encoderSamples[0].time.toMicroseconds()<<")\n";
       	    #endif

            /** Get the correct values from the input ports buffers  **/
            this->inputPortSamples();

            /** Calculate velocities from the input ports **/
            this->calculateVelocities();

            /** Out port the information of the  **/
            this->outputPortSamples ();

            /** Reset back the counters and the flags **/
            counter.reset();
            flag.reset();

        }

        /** Reset counter in case of inconsistency **/
        if (counter.encoderSamples > cbEncoderSamples.size())
            counter.encoderSamples = 0;

    }
}

void Processing::left_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample)
{
    /** Get the transformation from the transformer (iMoby transformer) **/
    Eigen::Affine3d tf;

    if(!_lcamera2body.get( ts, tf ))
    {
        RTT::log(RTT::Warning)<<"[LEFT-CAMERA FATAL ERROR] No transformation provided for the transformer."<<RTT::endlog();
        return;
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[PROCESSING LEFT-CAMERA] Frame at: "<<left_frame_sample->time.toMicroseconds()<<"\n";
    #endif

    /** Undistorted image depending on meta data information **/
    ::base::samples::frame::Frame *frame_ptr = leftFrame.write_access();
    frame_ptr->time = left_frame_sample->time;
    frame_ptr->init(left_frame_sample->size.width, left_frame_sample->size.height, left_frame_sample->getDataDepth(), left_frame_sample->getFrameMode());
    frameHelperLeft.undistort(*left_frame_sample, *frame_ptr);
    leftFrame.reset(frame_ptr);

     /** If synchronize camera with the servo **/
    if (cameraSynch.synchOn)
    {
        /** Check the time difference between inertial sensors and joint samples **/
        base::Time diffTime = leftFrame->time - rightFrame->time;

        /** If the difference in time is less than half of a period run the synchronization **/
        if (diffTime.toSeconds() < (_left_frame_period/2.0))
        {
            this->cameraWithDynamixelSynchro(ts, tf, leftFrame, rightFrame);
        }
    }
    else
    {
        /** Write the camera frame into the port **/
        _left_frame_out.write(leftFrame);
    }

    return;
}

void Processing::right_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample)
{
    /** Get the transformation from the transformer (iMoby transformer) **/
    Eigen::Affine3d tf;
    if(!_lcamera2body.get( ts, tf ))
    {
        RTT::log(RTT::Warning)<<"[RIGHT-CAMERA FATAL ERROR] No transformation provided for the transformer."<<RTT::endlog();
        return;
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[PROCESSING RIGHT-CAMERA] Frame at: "<<right_frame_sample->time.toMicroseconds()<<"\n";
    #endif

    /** Undistorted image depending on meta data information **/
    ::base::samples::frame::Frame *frame_ptr = rightFrame.write_access();
    frame_ptr->time = right_frame_sample->time;
    frame_ptr->init(right_frame_sample->size.width, right_frame_sample->size.height, right_frame_sample->getDataDepth(), right_frame_sample->getFrameMode());
    frameHelperRight.undistort(*right_frame_sample, *frame_ptr);
    rightFrame.reset(frame_ptr);

    /** If synchronize camera with the servo **/
    if (cameraSynch.synchOn)
    {
        /** Check the time difference between inertial sensors and joint samples **/
        base::Time diffTime = rightFrame->time - leftFrame->time;

        /** If the difference in time is less than half of a period run the synchronization **/
        if (diffTime.toSeconds() < (_right_frame_period/2.0))
        {
            this->cameraWithDynamixelSynchro(ts, tf, leftFrame, rightFrame);
        }
    }
    else
    {
        /** Write the camera frame into the port **/
        _right_frame_out.write(rightFrame);
    }

    return;
}

void Processing::scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample)
{

    /** Get the transformation from the transformer (iMoby transformer) **/
    Eigen::Affine3d tf;
    if(!_laser2body.get( ts, tf ))
        return;

    /** Convert laser scans to point-cloud in body frame **/
    base::samples::Pointcloud pointcloud;
    scan_samples_sample.convertScanToPointCloud<base::Point>(pointcloud.points, tf);

    #ifdef DEBUG_PRINTS
    std::cout<<"[PROCESSING LASER-SCANS] Scan time: "<<scan_samples_sample.time.toMicroseconds()<<"\n";
    std::cout<<"[PROCESSING LASER-SCANS] Scan size: "<<scan_samples_sample.ranges.size()<<"\n";
    std::cout<<"[PROCESSING LASER-SCANS] Start angle: "<<scan_samples_sample.start_angle<<"\n";
    std::cout<<"[PROCESSING LASER-SCANS] Angular resolution: "<<scan_samples_sample.angular_resolution<<"\n";
    std::cout<<"[PROCESSING LASER-SCANS] Point cloud size: "<<pointcloud.points.size()<<"\n";
    #endif

    /** Delete all the points which have a Z component higher than the laser frame **/
    base::samples::Pointcloud pointcloudFiltered;
    pointcloudFiltered.time = pointcloud.time;
    for (std::vector<base::Point>::iterator it = pointcloud.points.begin() ; it != pointcloud.points.end(); ++it)
    {
        if ((*it)[2] < tf.translation()[2])
            pointcloudFiltered.points.push_back(*it);
    }

    /** Write the point cloud into the port **/
    _point_cloud_samples_out.write(pointcloudFiltered);

    /** Debug transformation port (laser frame expressed in body frame) **/
    if (_output_debug.value())
    {
        base::samples::RigidBodyState body2laserRbs;
        body2laserRbs.invalidate();
        body2laserRbs.time = ts;
        body2laserRbs.position = tf.translation();
        body2laserRbs.orientation = Eigen::Quaternion <double> (tf.rotation());
        _body_to_laser.write(body2laserRbs);
    }

    return;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Processing.hpp for more detailed
// documentation about them.

bool Processing::configureHook()
{
    if (! ProcessingBase::configureHook())
        return false;

    /************************/
    /** Read configuration **/
    /************************/
    location = _location.value();
    config = _configuration.value();
    cameraSynch = _camera_synch.value();


    /*******************************************/
    /** Initial world to navigation transform **/
    /*******************************************/

    /** Set the initial world to navigation frame transform **/
    world2navigationRbs.invalidate();
    world2navigationRbs.sourceFrame = "world";
    world2navigationRbs.targetFrame = "navigation";

    /** If there is not an external reference system **/
    if (!_reference_pose_samples.connected())
    {
	/** set zero position **/
	world2navigationRbs.position.setZero();
	world2navigationRbs.velocity.setZero();
	world2navigationRbs.angular_velocity.setZero();
	
	/** Assume very well know initial attitude **/
	world2navigationRbs.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
	world2navigationRbs.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
	
	/** Assume well known starting position **/
	world2navigationRbs.cov_position = Eigen::Matrix <double, 3 , 3>::Zero();
	world2navigationRbs.cov_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
	
	initPosition = true;
    }

    /******************************************/
    /** Use properties to Configure the Task **/
    /******************************************/

    /** Set the initial number of samples for the attitude **/
    if (config.init_leveling_time != 0.00)
        init_leveling_size = static_cast<int>(config.init_leveling_time/_inertial_samples_period.value());
    else
        init_leveling_size = 1;

    /** Set the index to Zero **/
    init_leveling_accidx = 0;

    /** Initial size for initial acceleration leveling (using accelerometers) **/
    init_leveling_acc.resize (3, init_leveling_size);

    /** Initial size for initial acceleration leveling (using inclinometers) **/
    init_leveling_incl.resize (3, init_leveling_size);

    /** Set the Input ports counter to Zero **/
    counter.reset();

    /** Set the number of samples between each sensor input (if there are not coming at the same sampling rate) */
    if (config.output_frequency != 0.00)
    {
	number.imuSamples = (1.0/_inertial_samples_period.value())/config.output_frequency;
	number.encoderSamples = (1.0/_encoder_samples_period.value())/config.output_frequency;
	number.asguardStatusSamples = (1.0/_systemstate_samples_period.value())/config.output_frequency;
	number.forceSamples = (1.0/_ground_forces_samples_period.value())/config.output_frequency;
	number.torqueSamples = (1.0/_torque_samples_period.value())/config.output_frequency;
	number.referencePoseSamples = (1.0/_reference_pose_samples_period.value())/config.output_frequency;
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[PROCESSING CONFIGURE] cbEncoderSamples has init capacity "<<cbEncoderSamples.capacity()<<" and size "<<cbEncoderSamples.size()<<"\n";
    std::cout<<"[PROCESSING CONFIGURE] cbAsguardStatusSamples has init capacity "<<cbAsguardStatusSamples.capacity()<<" and size "<<cbAsguardStatusSamples.size()<<"\n";
    std::cout<<"[PROCESSING CONFIGURE] cbImuSamples has init capacity "<<cbImuSamples.capacity()<<" and size "<<cbImuSamples.size()<<"\n";
    #endif

    /** Set the capacity of the circular_buffer according to the sampling rate **/
    cbEncoderSamples.set_capacity(number.encoderSamples);
    cbAsguardStatusSamples.set_capacity(number.asguardStatusSamples);
    cbImuSamples.set_capacity(number.imuSamples);

    for(register unsigned int i=0; i<cbEncoderSamples.size(); ++i)
    {
	cbEncoderSamples[i].resize(config.jointNames.size()-1);
    }


    #ifdef DEBUG_PRINTS
    std::cout<<"[PROCESSING CONFIGURE] cbEncoderSamples has capacity "<<cbEncoderSamples.capacity()<<" and size "<<cbEncoderSamples.size()<<"\n";
    std::cout<<"[PROCESSING CONFIGURE] cbAsguardStatusSamples has capacity "<<cbAsguardStatusSamples.capacity()<<" and size "<<cbAsguardStatusSamples.size()<<"\n";
    std::cout<<"[PROCESSING CONFIGURE] cbImuSamples has capacity "<<cbImuSamples.capacity()<<" and size "<<cbImuSamples.size()<<"\n";
    #endif

    /** Initialize the samples for the filtered buffer hbridge values **/
    for(register unsigned int i=0;i<encoderSamples.size();i++)
    {
	/** Sizing hbridgeStatus **/
	encoderSamples[i].resize(config.jointNames.size()-1);

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

    /** Initialize values **/
    stateEstimationSamples.time.fromMicroseconds (base::NaN<uint64_t>());
    stateEstimationSamples.abias.setZero();
    stateEstimationSamples.gbias.setZero();

    #ifdef DEBUG_PRINTS
    std::cout<<"[PROCESSING CONFIGURE] encoderSamples has capacity "<<encoderSamples.capacity()<<" and size "<<encoderSamples.size()<<"\n";
    std::cout<<"[PROCESSING CONFIGURE] asguardStatusSamples has capacity "<<asguardStatusSamples.capacity()<<" and size "<<asguardStatusSamples.size()<<"\n";
    std::cout<<"[PROCESSING CONFIGURE] imuSamples has capacity "<<imuSamples.capacity()<<" and size "<<imuSamples.size()<<"\n";
    std::cout<<"[PROCESSING CONFIGURE] poseSamples has capacity "<<poseSamples.capacity()<<" and size "<<poseSamples.size()<<"\n";
    #endif

    /** Gravitational value according to the location. Ideal theoretical value **/
    inertialState.theoretical_g = localization::Util::GravityModel (location.latitude, location.altitude);

    /** Output Joints state vector **/
    jointSamples.resize(config.jointNames.size());
    jointSamples.names = config.jointNames;

    /** Output images **/
    ::base::samples::frame::Frame *lFrame = new ::base::samples::frame::Frame();
    ::base::samples::frame::Frame *rFrame = new ::base::samples::frame::Frame();

    leftFrame.reset(lFrame);
    rightFrame.reset(rFrame);

    lFrame = NULL; rFrame = NULL;

    /** Frame Helper **/
    frameHelperLeft.setCalibrationParameter(_left_camera_parameters.value());
    frameHelperRight.setCalibrationParameter(_right_camera_parameters.value());

    /**************************************/
    /** Info and Warnings about the Task **/
    /**************************************/

    if (_inertial_samples.connected())
    {
	RTT::log(RTT::Warning) << "[Info] IMU Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "[Info] IMU samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "[Info] Malfunction on the task!!" << RTT::endlog();
    }

    if (_encoder_samples.connected())
    {
	RTT::log(RTT::Warning) << "[Info] Encoders Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "[Info] Encoders samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Info] Malfunction on the task!!" << RTT::endlog();
    }

    if (_systemstate_samples.connected())
    {
	RTT::log(RTT::Warning) << "[Info] System State Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "[Info] System State samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "[Info] Malfunction on the task!!" << RTT::endlog();
    }

    if (_torque_samples.connected())
    {
	RTT::log(RTT::Warning) << "[Info] Wheel Torque samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "[Info] Wheel Torque samples NO connected." << RTT::endlog();
    }

    if (_ground_forces_samples.connected())
    {
	RTT::log(RTT::Warning) << "[Info] Wheel ground force estimation samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "[Info] Wheel ground force estimation samples NO connected." << RTT::endlog();
    }

    if (_reference_pose_samples.connected())
    {
	RTT::log(RTT::Warning) << "[Info] Initial pose/Ground Truth connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "[Info] Initial orientation/Ground Truth NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "[Info] Zero Yaw angle pointing to North is then assumed." << RTT::endlog();
	RTT::log(RTT::Warning) << "[Info] Pitch and Roll are taken from inertial sensors assuming static body at Initial phase of this task." << RTT::endlog();
    }

    RTT::log(RTT::Warning)<<"[Info] Frequency of IMU samples[Hertz]: "<<(1.0/_inertial_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Frequency of Encoders Samples[Hertz]: "<<(1.0/_encoder_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Frequency of Asguard Status Samples[Hertz]: "<<(1.0/_systemstate_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Frequency of Torque Samples[Hertz]: "<<(1.0/_torque_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Frequency of Ground Force Samples[Hertz]: "<<(1.0/_ground_forces_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Output Frequency[Hertz]: "<<config.output_frequency<<RTT::endlog();

    if (config.use_inclinometers_leveling)
        RTT::log(RTT::Warning)<<"[Info] Initial leveling using Inclinometers ";
    else
        RTT::log(RTT::Warning)<<"[Info] Initial leveling using Accelerometers ";

    RTT::log(RTT::Warning)<<"Time[seconds]: "<<config.init_leveling_time<<" which at "<<(1.0)/_inertial_samples_period.value()
        <<" Hertz are "<<init_leveling_size<<" #Samples"<<RTT::endlog();

    RTT::log(RTT::Warning)<<"[Info] number.imuSamples: "<<number.imuSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] number.encoderSamples: "<<number.encoderSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] number.asguardStatusSamples: "<<number.asguardStatusSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] number.forceSamples: "<<number.forceSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] number.torqueSamples: "<<number.torqueSamples<<RTT::endlog();

    if ((number.encoderSamples == 0)||(number.asguardStatusSamples == 0)||(number.imuSamples == 0))
    {
        RTT::log(RTT::Warning)<<"[FATAL ERROR] Output frequency cannot be higher than sensors frequency."<<RTT::endlog();
        return false;
    }

    return true;
}

bool Processing::startHook()
{
    if (! ProcessingBase::startHook())
        return false;
    return true;
}
void Processing::updateHook()
{
    ProcessingBase::updateHook();
}
void Processing::errorHook()
{
    ProcessingBase::errorHook();
}
void Processing::stopHook()
{
    ProcessingBase::stopHook();
}
void Processing::cleanupHook()
{
    ProcessingBase::cleanupHook();
}

void Processing::inputPortSamples()
{
    unsigned int cbEncoderSize = cbEncoderSamples.size();
    unsigned int cbAsguardStatusSize = cbAsguardStatusSamples.size();
    unsigned int cbImuSize = cbImuSamples.size();

    /** Local variable of the ports **/
    base::actuators::Status encoder;
    sysmon::SystemStatus asguardStatus;
    base::samples::IMUSensors imu;

    /** sizing the encoders **/
    encoder.resize(config.jointNames.size()-1);

    #ifdef DEBUG_PRINTS
    std::cout<<"[GetInportValue] cbEncoderSamples has capacity "<<cbEncoderSamples.capacity()<<" and size "<<cbEncoderSamples.size()<<"\n";
    std::cout<<"[GetInportValue] cbAsguardStatusSamples has capacity "<<cbAsguardStatusSamples.capacity()<<" and size "<<cbAsguardStatusSamples.size()<<"\n";
    std::cout<<"[GetInportValue] cbImuSamples has capacity "<<cbImuSamples.capacity()<<" and size "<<cbImuSamples.size()<<"\n";
    #endif

    /** ********* **/
    /** Encoders  **/
    /** ********* **/
    for (unsigned int i = 0; i<(config.jointNames.size()-1); ++i)
    {
	encoder.states[i].current = 0.0;
	encoder.states[i].position = 0.0;
	encoder.states[i].positionExtern = 0.0;
	encoder.states[i].pwm = 0.0;
    }
	
    /** Process the buffer **/
    for (register unsigned int i = 0; i<cbEncoderSize; ++i)
    {
	for (register unsigned int j = 0; j<(config.jointNames.size()-1); ++j)
	{
	    encoder.states[j].current += cbEncoderSamples[i].states[j].current;
	    encoder.states[j].pwm += cbEncoderSamples[i].states[j].pwm;
	}
    }

    /** Set the time **/
    encoder.time = (cbEncoderSamples[cbEncoderSize-1].time + cbEncoderSamples[0].time)/2.0;

    encoder.index = cbEncoderSamples[0].index;

    for (register unsigned int i = 0; i<(config.jointNames.size()-1); ++i)
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

    /*****************************/
    /** Store the Joint values  **/
    /*****************************/
    jointSamples[0].position = asguardStatusSamples[0].asguardJointEncoder;
    for (register int i=0; i<static_cast<int> ((config.jointNames.size()-1)); ++i)
    {
        jointSamples[i+1].position = encoderSamples[0].states[i].positionExtern;
    }

    /** Set all counters to zero **/
    counter.reset();

    return;
}

void Processing::calculateVelocities()
{
    double delta_t = (1.0/config.output_frequency);
    #ifdef DEBUG_PRINTS
    base::Time encoderDelta_t = encoderSamples[0].time - encoderSamples[1].time;
    base::Time asguardStatusDelta_t = asguardStatusSamples[0].time - asguardStatusSamples[1].time;
    base::Time imuDelta_t = imuSamples[0].time - imuSamples[1].time;
    #endif
    Eigen::Matrix<double, Eigen::Dynamic, 1> derivationEncoderSamples; //!Local variable for the derivation of the encoders
    Eigen::Matrix<double, Eigen::Dynamic, 1> derivationAsguardStatusSamples; //!Local variable for the derivation of Asguard Status (pasive Joint)

    /** At least two values to perform the derivative **/
    if (static_cast<int>(encoderSamples.size()) < 2)
    {
        for (register size_t i=0; i<config.jointNames.size(); ++i)
        {
            jointSamples[i].speed = 0.00;
        }
    }
    else
    {
        /** Set the correct size for the derivative data vectors **/
	derivationEncoderSamples.resize(static_cast<int>(encoderSamples.size()),1);
	derivationAsguardStatusSamples.resize(static_cast<int>(encoderSamples.size()),1);

        /** Set the initial derivation vector **/
        derivationAsguardStatusSamples.setZero();
	
	#ifdef DEBUG_PRINTS
        std::cout<<"[PROCESSING CALCULATING_VELO] ********************************************* \n";
	std::cout<<"[PROCESSING CALCULATING_VELO] Timestamp New(asguardStatus): "<< encoderSamples[0].time.toMicroseconds() <<" Timestamp Prev: "<<encoderSamples[1].time.toMicroseconds()<<"\n";
	std::cout<<"[PROCESSING CALCULATING_VELO] Delta time(asguardStatus): "<< asguardStatusDelta_t.toSeconds()<<"\n";
	std::cout<<"[PROCESSING CALCULATING_VELO] Timestamp New(IMU): "<< imuSamples[0].time.toMicroseconds() <<" Timestamp Prev: "<<imuSamples[1].time.toMicroseconds()<<"\n";
	std::cout<<"[PROCESSING CALCULATING_VELO] Delta time(IMU): "<< imuDelta_t.toSeconds()<<"\n";
	std::cout<<"[PROCESSING CALCULATING_VELO] asguardStatus(passive Joint): "<< asguardStatusSamples[0].asguardJointEncoder <<" Prev: "<<asguardStatusSamples[1].asguardJointEncoder<<"\n";
	#endif

	/** Fill the derivative vector **/
	for (register int i=0; i<static_cast<int>(encoderSamples.size()); ++i)
	{
	    derivationAsguardStatusSamples[i] = asguardStatusSamples[i].asguardJointEncoder;
	}
	
	#ifdef DEBUG_PRINTS
	std::cout<<"[PROCESSING CALCULATING_VELO] passiveJoint old velocity: "<<(asguardStatusSamples[0].asguardJointEncoder - asguardStatusSamples[1].asguardJointEncoder)/delta_t<<"\n";
	#endif
		
	/** Passive joint velocity **/
	jointSamples[0].speed = localization::Util::finiteDifference (derivationAsguardStatusSamples, delta_t); //passive joints speed
	
	#ifdef DEBUG_PRINTS
	std::cout<<"[PROCESSING CALCULATING_VELO] passiveJoint new velocity: "<<jointSamples[0].speed<<"\n";
	#endif

	/** Velocities for the vector **/
	for (register size_t i = 0; i<static_cast<size_t>((config.jointNames.size()-1)); ++i)
	{
	    #ifdef DEBUG_PRINTS
	    std::cout<<"[PROCESSING CALCULATING_VELO] Timestamp New(encoders): "<< encoderSamples[0].time.toMicroseconds() <<" Timestamp Prev: "<<encoderSamples[1].time.toMicroseconds()<<"\n";
	    std::cout<<"[PROCESSING CALCULATING_VELO] Delta time(encoders): "<< encoderDelta_t.toSeconds()<<"\n";
	    std::cout<<"[PROCESSING CALCULATING_VELO] ["<<i<<"] New: "<< encoderSamples[0].states[i].positionExtern <<" Prev: "<<encoderSamples[1].states[i].positionExtern<<"\n";
	    #endif
	
	    derivationEncoderSamples.setZero();//!Set to zero
	
	    /** Fill the derivative vector **/
	    for (register size_t j=0; j<static_cast<size_t>(encoderSamples.size()); ++j)
	    {
		derivationEncoderSamples[j] = encoderSamples[j].states[i].positionExtern;
	    }
	
	    #ifdef DEBUG_PRINTS
	    std::cout<<"[PROCESSING CALCULATING_VELO] ["<<i<<"] encoderSamples old velocity: "<<(encoderSamples[0].states[i].positionExtern - encoderSamples[1].states[i].positionExtern)/delta_t<<"\n";
	    #endif
	
	    /** Motor joint velocity **/
	    jointSamples[i+1].speed = localization::Util::finiteDifference(derivationEncoderSamples, delta_t); //!wheel rotation speed
	
	    #ifdef DEBUG_PRINTS
	    std::cout<<"[PROCESSING CALCULATING_VELO] ["<<i<<"] encoderSamples new velocity: "<<jointSamples[i+1].speed<<"\n";
	    #endif

	}

        /** TO-DO: remove this debug info ports **/
        if (_output_debug.value())
        {
            _angular_position.write(encoderSamples[0].states[3].positionExtern);
            _angular_rate.write(jointSamples[4].speed); //!Front Left
            _angular_rate_old.write((encoderSamples[0].states[3].positionExtern - encoderSamples[1].states[3].positionExtern)/delta_t);
        }
    }

    return;
}

void Processing::cameraWithDynamixelSynchro(const base::Time &ts, const Eigen::Affine3d &tf,
                        const RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> &leftFrame,
                        const RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> &rightFrame)
{

    /** Transformation left camera expressed in body frame **/
    body2lcameraRbs.invalidate();
    body2lcameraRbs.time = ts;
    body2lcameraRbs.position = tf.translation();
    body2lcameraRbs.orientation = Eigen::Quaternion <double> (tf.rotation());

    Eigen::Quaterniond errorQuaternion = body2lcameraRbs.orientation.inverse() * cameraSynch.body2lcamera;

    /** Set to true the zero Mark when passing the horizon line (in body frame) **/
    if (!cameraSynch.zeroMark && (errorQuaternion.toRotationMatrix().eulerAngles(2,1,0)[2] > 0.0))
    {
        cameraSynch.zeroMark = true;
    }
    else if (cameraSynch.zeroMark &&
    (fabs(errorQuaternion.toRotationMatrix().eulerAngles(2,1,0)[2]) < cameraSynch.quatError.toRotationMatrix().eulerAngles(2,1,0)[2]))
    {
        /** Write the camera frame into the port **/
        cameraSynch.zeroMark = false;
        _left_frame_out.write(leftFrame);
        _right_frame_out.write(rightFrame);
    }

    if (_output_debug.value())
    {
        _body_to_lcamera.write(body2lcameraRbs);

        Eigen::Matrix <double, 3, 1> euler; /** In Euler angles **/
        euler[2] = body2lcameraRbs.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
        euler[1] = body2lcameraRbs.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
        euler[0] = body2lcameraRbs.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
        _body_to_lcamera_euler.write(euler*R2D);

        euler[2] = errorQuaternion.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
        euler[1] = errorQuaternion.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
        euler[0] = errorQuaternion.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
        _error_body_to_lcamera_euler.write(euler*R2D);

        /** Get extrinsic parameters to have rcamera with respect to lcamera **/
        Eigen::Vector3d tlcamera2rcamera = Eigen::Vector3d(_extrinsic_camera_parameters.value().tx,
                                                        _extrinsic_camera_parameters.value().ty,
                                                        _extrinsic_camera_parameters.value().tz);
        Eigen::Quaterniond qlcamera2rcamera = Eigen::Quaternion <double> (Eigen::AngleAxisd(_extrinsic_camera_parameters.value().rz, Eigen::Vector3d::UnitZ())*
                Eigen::AngleAxisd(_extrinsic_camera_parameters.value().ry, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(_extrinsic_camera_parameters.value().rx, Eigen::Vector3d::UnitX()));


        /** Port out body to rcamera **/
        base::samples::RigidBodyState body2rcameraRbs = body2lcameraRbs;
        body2rcameraRbs.position += body2lcameraRbs.orientation * tlcamera2rcamera;
        body2rcameraRbs.orientation *= qlcamera2rcamera;

        _body_to_rcamera.write(body2rcameraRbs);
    }

    return;
}

void Processing::outputPortSamples()
{
    std::vector<Eigen::Affine3d> fkRobot;

    /*******************************************/
    /** Port out the Output Ports information **/
    /*******************************************/

    /** Joint samples out **/
    jointSamples.time = encoderSamples[0].time;
    _joints_samples_out.write(jointSamples);

    /** Calibrated and compensate inertial values **/
    inertialSamples = imuSamples[0];
    _inertial_samples_out.write(inertialSamples);

    /** Inertial State  **/
    inertialState.time = imuSamples[0].time;
    _inertial_state_out.write(inertialState);

    /** The estimated world 2 navigation transform **/
    world2navigationRbs.time = encoderSamples[0].time;//timestamp;
    _world_to_navigation_out.write(world2navigationRbs);

    /** Ground Truth if available **/
    if (_reference_pose_samples.connected())
    {
        /** Port Out the info coming from the ground truth **/
        referenceOut = poseSamples[0];
        referenceOut.velocity = world2navigationRbs.orientation.inverse() * poseSamples[0].velocity; //velocity in body frame
        referenceOut.time = encoderSamples[0].time;
        _reference_pose_samples_out.write(referenceOut);

        /** Delta increments of the ground truth at delta_t given by the output_frequency **/
        referenceOut.position = poseSamples[0].position - poseSamples[1].position;
        referenceOut.cov_position = poseSamples[0].cov_position + poseSamples[1].cov_position;
        referenceOut.velocity = world2navigationRbs.orientation.inverse() * (poseSamples[0].velocity - poseSamples[1].velocity);//in body frame
        referenceOut.cov_velocity = world2navigationRbs.orientation.inverse() * (poseSamples[0].cov_velocity + poseSamples[1].cov_velocity);
        _reference_delta_pose_samples_out.write(referenceOut);
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[PROCESSING OUTPUT_PORTS]: world2navigationRbs.position\n"<<world2navigationRbs.position<<"\n";
    std::cout<<"[PROCESSING OUTPUT_PORTS]: world2navigationRbs.velocity\n"<<world2navigationRbs.velocity<<"\n";
    std::cout<<"[PROCESSING OUTPUT_PORTS]: referenceOut.position\n"<<referenceOut.velocity<<"\n";
    std::cout<<"[PROCESSING OUTPUT_PORTS] ******************** END ******************** \n";
    #endif
    /** Store the Debug OutPorts information **/

    return;
}
