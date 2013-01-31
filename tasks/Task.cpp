/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#define DEBUG_PRINTS 1

using namespace rover_localization;
using namespace localization;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    
    /** Booleans values to false **/
    imuValues = false;
    hbridgeValues = false;
    asguardValues = false;
    poseInitValues = false;
    initPosition = false;
    initAttitude = false;
    
    /** Default size for the circular_buffer **/
    cbHbridges = boost::circular_buffer<base::actuators::Status>(DEFAULT_CIRCULAR_BUFFER_SIZE);
    cbAsguard = boost::circular_buffer<sysmon::SystemStatus> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    cbIMU = boost::circular_buffer<base::samples::IMUSensors> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    
    /** Sizing hbridgeStatus **/
    hbridgeStatus.resize(NUMBER_WHEELS);
    
    /** Set to a NaN index **/
    hbridgeStatus.index = base::NaN<unsigned int>();
    
    /** Set to NaN passiveJoint **/
    asguardStatus.asguardJointEncoder = base::NaN<double>();
    
    wheelFL = asguard::KinematicModel(3,2);
    wheelFR = asguard::KinematicModel(2,2);
    wheelRR = asguard::KinematicModel(1,2);
    wheelRL = asguard::KinematicModel(0,2);
    
    /** Imu Samples **/
    imuSamples.acc[0] = base::NaN<double>();
    imuSamples.acc[1] = base::NaN<double>();
    imuSamples.acc[2] = base::NaN<double>();
    imuSamples.gyro = imuSamples.acc;
    imuSamples.mag = imuSamples.acc;
    
    /** Set eccentricity to NaN **/
    eccx[0] = base::NaN<double>();
    eccx[1] = base::NaN<double>();
    eccx[2] = base::NaN<double>();
    eccy = eccx;
    eccz = eccx;
    
    /** Contact angle to zero **/
    contactAngle.resize(NUMBER_WHEELS);
    std::fill(contactAngle.begin(), contactAngle.end(), 0.00);
    
    /** Pose Init **/
    poseInit.invalidate();
    
    /** Set also here the default Foot in Contact **/
    contactPoints.resize(4);
    contactPoints[0] = 2; contactPoints[1] = 2;
    contactPoints[2] = 2; contactPoints[3] = 2;
    
    /** Set output times **/
    outTimeHbridge.fromMilliseconds(0.00);
    outTimeStatus.fromMilliseconds(0.00);
    outTimeForce.fromMilliseconds(0.00);
    outTimeIMU.fromMilliseconds(0.00);
    outTimeTorque.fromMilliseconds(0.00);
    
    /** Point the measurement to the one in the filter object **/
    mymeasure = &(mysckf.filtermeasurement);
    
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
    /** Measurement pointer to null **/
    mymeasure = NULL;
}

void Task::calibrated_sensorsTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &calibrated_sensors_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation in quaternion form **/
    
    /** Get the transformation **/
    if (!_imu2body.get(ts, tf, false))
	return;
    
    qtf = Eigen::Quaternion <double> (tf.rotation());
    
    #ifdef DEBUG_PRINTS
    std::cout<<"Transformer:\n"<<tf.matrix()<<"\n";
    #endif
	
    /** Check if the eccentricities are not defined **/
    if (base::isNaN(eccx[0]) && base::isNaN(eccy[0]) && base::isNaN(eccz[0]))
    {
	#ifdef DEBUG_PRINTS
	std::cout<<"Eccentricities are NaN\n";
	std::cout<<"Translation:\n"<<tf.translation()<<"\n";
	std::cout<<"Eccx :\n"<<tf.rotation() * _eccx.value()<<"\n";
	std::cout<<"Eccy :\n"<<tf.rotation() * _eccy.value()<<"\n";
	std::cout<<"Eccz :\n"<<tf.rotation() * _eccz.value()<<"\n";
	#endif
	
	/** Store the excentricity **/
	eccx = tf.translation() + tf.rotation() *_eccx.value();
	eccy = tf.translation() + tf.rotation() *_eccy.value();
	eccz = tf.translation() + tf.rotation() *_eccz.value();
	
	/** Set the eccentricity to the filter **/
	mysckf.filtermeasurement.setEccentricity(eccx, eccy, eccz);
    }
    
    /** If the initial attitude is not defined and the orientation port is not connected **/
    if (!initAttitude)
    {
	#ifdef DEBUG_PRINTS
	std::cout<< "Calculating initial level position since Init orientation is not provided." <<"\n";
	#endif
	
	/** Add one acc sample to the buffer **/
	init_acc.col(accidx) = calibrated_sensors_sample.acc;
	accidx++;

	if (accidx == NUMBER_INIT_ACC)
	{
	    Eigen::Matrix <double,NUMAXIS,1> meanacc; /** In euler angles **/
	    Eigen::Matrix <double,NUMAXIS,1> euler; /** In euler angles **/
	    Eigen::Quaternion <double> attitude; /** Initial attitude in case no port in orientation is connected **/
	    
	    meanacc[0] = init_acc.row(0).mean();
	    meanacc[1] = init_acc.row(1).mean();
	    meanacc[2] = init_acc.row(2).mean();
	    
	    #ifdef DEBUG_PRINTS
	    std::cout<<"*** Computed mean acc values: "<<meanacc[0]<<" "<<meanacc[1]<<" "<<meanacc[2]<<"\n";
	    std::cout<<"*** Computed gravity: "<<meanacc.norm()<<"\n";
	    #endif
	    
	    euler[0] = (double) asin((double)meanacc[1]/ (double)meanacc.norm()); // Roll
	    euler[1] = (double) -atan(meanacc[0]/meanacc[2]); //Pitch
	    euler[2] = M_PI;
	    
	    /** Set the initial attitude when no initial IMU orientation is provided **/
	    attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
	    Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
	    
	    #ifdef DEBUG_PRINTS
	    std::cout<< "******** Initial Attitude (STIM300 frame)  *******"<<"\n";
	    std::cout<< "Init Roll: "<<euler[0]*R2D<<"Init Pitch: "<<euler[1]*R2D<<"Init Yaw: "<<euler[2]*R2D<<"\n";
	    #endif
	    
	    /** This attitude is in the IMU frame. It needs to be expressed in body with the help of the transformer **/
	    attitude = attitude * qtf;
	    
	    /** Check if there is initial pose connected **/
	    if (_pose_init.connected() && initPosition)
	    {
		/** Get the initial Yaw from the initialPose **/
		euler[2] = poseInit.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
		euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
		euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
		
		/** Set the initial attitude with the Yaw provided from the initial pose **/
		attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
		Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
		
		/** Set the initial attitude quaternion of the filter **/
		mysckf.setAttitude (attitude);
		
		initAttitude = true;
		
	    }
	    else if (!_pose_init.connected() || !initPosition)
	    {
		/** Set the initial attitude quaternion of the filter **/
		mysckf.setAttitude (attitude);
		initAttitude = true;
	    }
    	    
	   
	    if (initAttitude)
	    {
		/**Store the value as the initial one for the rbsBC **/
		rbsBC.orientation = attitude;
		rbsBC.angular_velocity.setZero();
		
		/** Assume very well know initial attitude **/
		rbsBC.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
		rbsBC.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
		
		/** Set the initial pose in the dead reckoning process **/
		drPose.setInitPose(rbsBC);
// 		this->actualPose = rbsBC;
		
		/** Set the gravity to the filter to the mean computed **/
// 		mysckf.setGravity(meanacc.norm());
		
		#ifdef DEBUG_PRINTS
		euler = mysckf.getEuler();
		std::cout<< "******** Initial Attitude *******"<<"\n";
		std::cout<< "Init Roll: "<<euler[0]*R2D<<"Init Pitch: "<<euler[1]*R2D<<"Init Yaw: "<<euler[2]*R2D<<"\n";
		#endif
	    }
	    
	}
    }
    else
    {
	/** A new sample arrived to the port**/
	
	#ifdef DEBUG_PRINTS
	std::cout<<"** Received IMU Samples **\n";
	std::cout<<"acc:\n"<<calibrated_sensors_sample.acc<<"\n";
	std::cout<<"acc(quat):\n"<<qtf * calibrated_sensors_sample.acc<<"\n";
	std::cout<<"acc(Rot):\n"<< tf.rotation() * calibrated_sensors_sample.acc<<"\n";
	std::cout<<"gyro:\n"<<calibrated_sensors_sample.gyro<<"\n";
	std::cout<<"mag:\n"<<calibrated_sensors_sample.mag<<"\n";
	#endif
	
	/** Convert the IMU values in the body orientation **/
	imuSamples.time = calibrated_sensors_sample.time;
	imuSamples.acc = qtf * calibrated_sensors_sample.acc;
	imuSamples.gyro = qtf * calibrated_sensors_sample.gyro;
	imuSamples.mag = qtf * calibrated_sensors_sample.mag;
	
	/** Push the IMU sensor into the buffer **/
	cbIMU.push_back(imuSamples);
	
	counterIMUSamples++;
    }
}

void Task::hbridge_samplesTransformerCallback(const base::Time &ts, const ::base::actuators::Status &hbridge_samples_sample)
{
    /** A new sample arrived to the inport **/
    cbHbridges.push_back(hbridge_samples_sample);
    
    counterHbridgeSamples++;
    
    #ifdef DEBUG_PRINTS
    std::cout<<"** counterHbridgeSamples("<<counterHbridgeSamples<<") at ("<<hbridge_samples_sample.time.toMicroseconds()
	<<") received FR ("<<hbridge_samples_sample.states[0].positionExtern<<")**\n";
    #endif
       
    /** Implementation of the filter **/
    #ifdef DEBUG_PRINTS
    std::cout<<"In Filter Implementation\n";
    std::cout<<"hbridgeCounter ("<<counterHbridgeSamples<<") asguardCounter("<<counterAsguardStatusSamples<<") imuCounter("<<counterIMUSamples<<")\n";
    #endif
    
    /** Start the filter if everything is alrigth **/
    if (initAttitude && initPosition && (counterHbridgeSamples == numberHbridgeSamples))
    {
	
	#ifdef DEBUG_PRINTS
	std::cout<<"****************** ("<<hbridgeStatus.time.toMicroseconds()<<") ******************************\n";
	#endif
	
	/** Get the correct values from the input ports buffers **/
	this->getInputPortValues();
    
	/** Calculate the position and orientation of all asguard feets with the new encoders information **/
	this->calculateFootPoints();
	
	/** Select the Contact Point among the Foot Points **/
	this->selectContactPoints(contactPoints);
	
	/** Set the Contact Point in the KinematicModel **/
	wheelFL.setContactFootID(contactPoints[3]);
	wheelFR.setContactFootID(contactPoints[2]);
	wheelRR.setContactFootID(contactPoints[1]);
	wheelRL.setContactFootID(contactPoints[0]);
	
	wheelFL.getContactFootID();
	wheelFR.getContactFootID();
	wheelRR.getContactFootID();
	wheelRL.getContactFootID();
	
	#ifdef DEBUG_PRINTS
	std::cout<<"*********** Compute Trans Body to Contact Point **\n";
	#endif
	/** Compute the wheel Contact Point **/
	wheelRL.Body2ContactPoint(hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, contactAngle[0]);    
	wheelRR.Body2ContactPoint(hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, contactAngle[1]);
	wheelFR.Body2ContactPoint(hbridgeStatus.states[2].positionExtern, asguardStatus.asguardJointEncoder, contactAngle[2]);
	wheelFL.Body2ContactPoint(hbridgeStatus.states[3].positionExtern, asguardStatus.asguardJointEncoder, contactAngle[3]);
	
	/** Compute the wheel Jacobians for the Selected Foot Point **/
	Eigen::Matrix< double, NUMAXIS , 1> slipvector = Eigen::Matrix< double, 3 , 1>::Zero();
	jacobRL = wheelRL.getWheelJacobian (slipvector);
	jacobRR = wheelRR.getWheelJacobian (slipvector);
	jacobFR = wheelFR.getWheelJacobian (slipvector);
	jacobFL = wheelFL.getWheelJacobian (slipvector);
	
	/** Calculate encoders joints velocities **/
	this->calculateEncodersVelocities();
	
	/** Composite Slip-Kinematics and Observation matrix for the SCKF **/
	this->compositeMotionJacobians();
	
	/** Inertial sensor information **/
	Eigen::Matrix<double, NUMAXIS, 1> angular_velocity = imuSamples.gyro;
	Eigen::Matrix<double, NUMAXIS, 1> acc = imuSamples.acc;
	Eigen::Matrix<double, NUMAXIS, 1> mag = imuSamples.mag;
	
	/** Substract Earth rotation from gyros **/
	Eigen::Quaternion <double> currentq = mysckf.getAttitude();
	Measurement::SubstractEarthRotation(&angular_velocity, &currentq, _latitude.value());
	
	#ifdef DEBUG_PRINTS
	mysckf.getEuler();
	std::cout<<"********** PREDICT *****************************\n";
	#endif
	
	/** Predict the state of the filter **/
	mysckf.predict(angular_velocity, acc, delta_t);
	
	/** Using the incremental velocity as measurement **/
	Eigen::Matrix<double,NUMAXIS,NUMAXIS> Hme;
 	Eigen::Matrix<double,NUMAXIS,NUMAXIS> Rme;
	Eigen::Matrix<double,NUMAXIS,1> vel_error;
	
	/** Measurement Generation **/
	mysckf.measurementGeneration (Anav, Bnav, vjoints, vel_error, Rme, delta_t);
	
	Hme.setIdentity();
	
	#ifdef DEBUG_PRINTS
	std::cout<< "[Measurement] Hme is of size "<<Hme.rows()<<"x"<<Hme.cols()<<"\n";
	std::cout<< "[Measurement] Hme:\n"<<Hme<<"\n";
	if (mysckf.filtermeasurement.getCurrentVeloModel()[0] > 0.5)
	    std::cout<< "[Measurement] AQUI ES MAYOR DE 0.5 at "<<hbridgeStatus.time.toMicroseconds()<<"\n";
	#endif
	
	#ifdef DEBUG_PRINTS
	std::cout<<"********** UPDATE *****************************\n";
	#endif
	
	/** Update the state of the filter **/
 	mysckf.update(Hme, Rme, vel_error, acc, mag, delta_t, false);

	/** Set the variables to perform the dead-reckoning **/
	Eigen::Matrix<double, NUMAXIS, 1> linvelo = mysckf.filtermeasurement.getCurrentVeloModel();
	Eigen::Quaternion <double> delta_q = mysckf.deltaQuaternion();
	Eigen::Matrix<double, NUMAXIS, NUMAXIS> covlinvelo = Eigen::Matrix<double, NUMAXIS, NUMAXIS>::Zero();
	Eigen::Matrix<double, NUMAXIS, NUMAXIS> covdelta_q = Eigen::Matrix<double, NUMAXIS, NUMAXIS>::Zero();
	Eigen::Matrix<double, NUMAXIS, 1> linvelo_error = mysckf.getStatex().block<NUMAXIS,1> (3,0);
	
	std::cout<< "[Measurement] linvelo_error:\n"<<linvelo_error<<"\n";
	
	/** Dead-reckoning and save into rbsBC **/
	rbsBC = drPose.updatePose (linvelo, delta_q, covlinvelo, covdelta_q, linvelo_error, delta_q, covlinvelo, covdelta_q, hbridgeStatus.time, delta_t);
	
	/** Write new pose to the output port **/
	_pose_samples_out.write(rbsBC);
	
	#ifdef DEBUG_PRINTS
	std::cout<<"********** To Asguard Body **********\n";
	#endif
	
	/** Body pose to asguard for the vizkit visualization **/
	this->toAsguardBodyState();
	
	#ifdef DEBUG_PRINTS
	std::cout<<"********** To Deburg Ports **********\n";
	#endif
	
	/** To Debug ports **/
	this->toDebugPorts();
	
	/** Reset the state vector **/
	mysckf.resetStateVector();
	
	/** Envire outport **/
	this->sendEnvireEnvironment();
	
	#ifdef DEBUG_PRINTS
	std::cout<<"********** END **********\n";
	#endif
	
    }

}
void Task::systemstate_samplesTransformerCallback(const base::Time &ts, const ::sysmon::SystemStatus &systemstate_samples_sample)
{
    /** A new sample arrived to the port **/
    cbAsguard.push_back(systemstate_samples_sample);
    
    counterAsguardStatusSamples ++;
    
    #ifdef DEBUG_PRINTS
    std::cout<<"** counterAsguardStatusSamples("<<counterAsguardStatusSamples<<") at ("<<systemstate_samples_sample.time.toMicroseconds()<< ")**\n";
    std::cout<<"** passive joint value: "<< systemstate_samples_sample.asguardJointEncoder<<"\n";
    #endif
}

void Task::torque_estimatedTransformerCallback(const base::Time &ts, const ::torque_estimator::WheelTorques &torque_estimated_sample)
{
//     throw std::runtime_error("Transformer callback for torque_estimated not implemented");
}

void Task::ground_forces_estimatedTransformerCallback(const base::Time &ts, const ::torque_estimator::GroundForces &ground_forces_estimated_sample)
{
//     throw std::runtime_error("Transformer callback for ground_forces_estimated not implemented");
}

void Task::pose_initTransformerCallback(const base::Time& ts, const base::samples::RigidBodyState& pose_init_sample)
{
    
    poseInit = pose_init_sample;
    
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation in quaternion form **/
    
    /** Get the transformation **/
    if (!_vicon2body.get(ts, tf, false))
	return;
    
    qtf = Eigen::Quaternion <double> (tf.rotation());
    
    #ifdef DEBUG_PRINTS
    std::cout<<"** received poseInit sample at("<<pose_init_sample.time.toMicroseconds()<<") **\n";
    #endif
    
    /** Apply the transformer pose offset **/
    poseInit.position += qtf * tf.translation();
    poseInit.orientation = poseInit.orientation * qtf;
//     poseInit.velocity = qtf * poseInit.velocity;
//     poseInit.cov_velocity = tf.rotation() * poseInit.cov_velocity;
//     poseInit.cov_angular_velocity = tf.rotation() * poseInit.cov_angular_velocity;
    
    if (!initPosition)
    {
	rbsBC.position = poseInit.position;
	rbsBC.velocity.setZero();
	
	/** Assume well known starting position **/
	rbsBC.cov_position = Eigen::Matrix <double, NUMAXIS , NUMAXIS>::Zero();
	rbsBC.cov_velocity = Eigen::Matrix <double, NUMAXIS , NUMAXIS>::Zero();
	
	#ifdef DEBUG_PRINTS
	Eigen::Matrix <double,NUMAXIS,1> euler; /** In euler angles **/
	euler[2] = poseInit.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
	euler[1] = poseInit.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
	euler[0] = poseInit.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
 	std::cout<<"** poseInit at ("<<poseInit.time.toMicroseconds()<< ")**\n";
	std::cout<<"** position offset\n"<<tf.translation()<<"\n";
	std::cout<<"** rotation offset\n"<<tf.rotation()<<"\n";
	std::cout<<"** position\n"<< poseInit.position<<"\n";
	std::cout<<"** Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
	#endif

	/** Initial attitude from IMU acceleration has been already calculated **/
	if (initAttitude)
	{
	    Eigen::Matrix <double,NUMAXIS,1> euler; /** In euler angles **/
	    Eigen::Quaternion <double> attitude = mysckf.getAttitude(); /** Initial attitude in case no port in orientation is connected **/
	    
	    /** Get the initial Yaw from the initialPose **/
	    euler[2] = poseInit.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
	    euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
	    euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
	    
	    /** Set the initial attitude with the Yaw provided from the initial pose **/
	    attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
	    Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
	    
	    /** Set the initial attitude quaternion of the filter **/
	    mysckf.setAttitude (attitude);
	    
	    /**Store the value as the initial one for the rbsBC **/
	    rbsBC.orientation = attitude;
	    
	    #ifdef DEBUG_PRINTS
	    euler = mysckf.getEuler();
	    std::cout<< "******** Initial Attitude in poseInit *******"<<"\n";
	    std::cout<< "Init Roll: "<<euler[0]*R2D<<"Init Pitch: "<<euler[1]*R2D<<"Init Yaw: "<<euler[2]*R2D<<"\n";
	    #endif
	}
	
	/** Initial angular velocity **/
	rbsBC.angular_velocity.setZero();
	
	/** Assume very well know initial attitude **/
	rbsBC.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
	rbsBC.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
		
	/** Set the initial pose in the uncertainty variable **/
	drPose.setInitPose(rbsBC);
// 	this->actualPose = rbsBC;
	
	initPosition = true;
    }
    
    poseInitValues = true;
     
    if (!prevPoseInit.hasValidOrientation())
	prevPoseInit = poseInit;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    double delta_bandwidth = 0.00;/** Delta (bandwidth) time of inertial sensors **/
    double theoretical_g; /** Ideal gravity value **/
    Eigen::Matrix< double, Eigen::Dynamic,1> x_0; /** Initial vector state **/
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Ra; /** Measurement noise convariance matrix for acc */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rat; /** Measurement noise convariance matrix for attitude correction (gravity vector noise) */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rv; /** Measurement noise convariance matrix for velocity (accelerometers integration) */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rg; /** Measurement noise convariance matrix for gyros */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rm; /** Measurement noise convariance matrix for mag */
    Eigen::Matrix <double,localization::ENCODERS_VECTOR_SIZE,localization::ENCODERS_VECTOR_SIZE> Ren; /** Measurement noise of encoders **/
    Eigen::Matrix <double,Eigen::Dynamic,Eigen::Dynamic> P_0; /** Initial covariance matrix **/
    Eigen::Matrix <double,localization::NUMBER_OF_WHEELS,localization::NUMBER_OF_WHEELS> Rcontact; /** Measurement noise for contact angle */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Qbg; /** Process noise matrix of gyros bias (bias instability) **/
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Qba; /** Process noise matric of acc bias (bias instability) **/
    
    if (! TaskBase::configureHook())
        return false;
    
    if (_sensors_bandwidth.value() != 0.00)
	delta_bandwidth = 1.0/_sensors_bandwidth.value();
    
    /** Set the initial BC to the Geographic frame **/
    rbsBC.invalidate();
    rbsBC.sourceFrame = "Body Frame";
    rbsBC.targetFrame = "Geographic_Frame (North-West-Up)";
        
    if (!_pose_init.connected())
    {
	/** set zero position **/
	rbsBC.position.setZero();
	rbsBC.velocity.setZero();
	rbsBC.angular_velocity.setZero();
	
	/** Assume very well know initial attitude **/
	rbsBC.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
	rbsBC.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
	
	/** Assume well known starting position **/
	rbsBC.cov_position = Eigen::Matrix <double, 3 , 3>::Zero();
	rbsBC.cov_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
	
	initPosition = true;
    }
    
    
    /** Set the counters to Zero **/
    counterHbridgeSamples = 0;
    counterAsguardStatusSamples = 0;
    counterTorqueSamples = 0;
    counterForceSamples = 0;
    counterIMUSamples = 0;
    counterPose = 0;
    
    /** Set the number of samples between each sensor input (if there are not comming at the same sampling rate) */
    if (_filter_frequency.value() != 0.00)
    {
	numberIMUSamples = (1.0/_calibrated_sensors_period.value())/_filter_frequency.value();
	numberHbridgeSamples = (1.0/_hbridge_samples_period.value())/_filter_frequency.value();
	numberAsguardStatusSamples = (1.0/_systemstate_samples_period.value())/_filter_frequency.value();
	numberForceSamples = (1.0/_ground_forces_estimated_period.value())/_filter_frequency.value();
	numberTorqueSamples = (1.0/_torque_estimated_period.value())/_filter_frequency.value();
	numberPose = (1.0/_pose_init_period.value())/_filter_frequency.value();
    }
    
    
    /** Set the capacity of the circular_buffer according to the sampling rate **/
    cbHbridges.set_capacity(numberHbridgeSamples);
    cbAsguard.set_capacity(numberAsguardStatusSamples);
    cbIMU.set_capacity(numberIMUSamples);
    
    for(register unsigned int i=0;i<cbHbridges.size();i++)
    {
	cbHbridges[i].resize(NUMBER_WHEELS);
    }
    
    
    #ifdef DEBUG_PRINTS
    std::cout<<"[Task configure] cbHbridges has capacity "<<cbHbridges.capacity()<<" and size "<<cbHbridges.size()<<"\n";
    std::cout<<"[Task configure] cbAsguard has capacity "<<cbAsguard.capacity()<<" and size "<<cbAsguard.size()<<"\n";
    std::cout<<"[Task configure] cbIMU has capacity "<<cbIMU.capacity()<<" and size "<<cbIMU.size()<<"\n";
    #endif
    
    /** Filter delta time step in seconds **/
    if (_filter_frequency.value() != 0.00)
	delta_t = 1.0/_filter_frequency.value();    
    
    /** Fill the filter initial matrices **/
    /** Accelerometers covariance matrix **/
    Ra = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Ra(0,0) = pow(_accrw.get()[0]/sqrt(delta_bandwidth),2);
    Ra(1,1) = pow(_accrw.get()[1]/sqrt(delta_bandwidth),2);
    Ra(2,2) = pow(_accrw.get()[2]/sqrt(delta_bandwidth),2);
    
    #ifdef DEBUG_PRINTS
    std::cout<< "[Task configure] Ra is of size "<<Ra.rows()<<"x"<<Ra.cols()<<"\n";
    std::cout<< "[Task configure] Ra:\n"<<Ra<<"\n";
    #endif
    
    
    /** Gravity vector covariance matrix **/
    Rat = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Rat(0,0) = 0.0054785914701378034;
    Rat(1,1) = 0.0061094546837916494;
    Rat(2,2) = 0.0063186020143245212;
    
    /** Gyroscopes covariance matrix **/
    Rg = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Rg(0,0) = pow(_gyrorw.get()[0]/sqrt(delta_bandwidth),2);
    Rg(1,1) = pow(_gyrorw.get()[1]/sqrt(delta_bandwidth),2);
    Rg(2,2) = pow(_gyrorw.get()[2]/sqrt(delta_bandwidth),2);

    /** Magnetometers covariance matrix **/
    Rm = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Rm(0,0) = pow(_magrw.get()[0]/sqrt(delta_bandwidth),2);
    Rm(1,1) = pow(_magrw.get()[1]/sqrt(delta_bandwidth),2);
    Rm(2,2) = pow(_magrw.get()[2]/sqrt(delta_bandwidth),2);
    
    /** Encoders velocity errors **/
    Ren = 0.001 * Eigen::Matrix <double,localization::ENCODERS_VECTOR_SIZE, localization::ENCODERS_VECTOR_SIZE>::Identity();
    
    /** Contact angle error **/
    Rcontact = 0.01 * Eigen::Matrix <double,localization::NUMBER_OF_WHEELS,localization::NUMBER_OF_WHEELS>::Identity();
    
    /** Initial error covariance **/
    P_0.resize(Sckf::X_STATE_VECTOR_SIZE, Sckf::X_STATE_VECTOR_SIZE);
    P_0 = Eigen::Matrix <double,Sckf::X_STATE_VECTOR_SIZE,Sckf::X_STATE_VECTOR_SIZE>::Zero();
    P_0.block <2*NUMAXIS, 2*NUMAXIS> (0, 0) = 1e-20 * Eigen::Matrix <double,2*NUMAXIS,2*NUMAXIS>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> (2*NUMAXIS,2*NUMAXIS) = 1e-06 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> (3*NUMAXIS,3*NUMAXIS) = 1e-10 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> (4*NUMAXIS,4*NUMAXIS) = 1e-10 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    
    /** Process noise matrices **/    
    Qbg = 0.00000000001 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    Qba = 0.00000000001 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    
    /** Gravitational value according to the location **/
    theoretical_g = Measurement::GravityModel (_latitude.value(), _altitude.value());
    
    /** Initialization of the filter **/
    mysckf.Init(P_0, Rg, Qbg, Qba, Ra, Rat, Rm, theoretical_g, (double)_dip_angle.value());
    
    /** Initialization of the measurement generation **/
    mysckf.filtermeasurement.Init(Ren, Rcontact, _q_weight_distribution.value());
     
    /** Initialization set the vector state and bias offset to zero but they can be changed here **/
    x_0.resize(Sckf::X_STATE_VECTOR_SIZE,1);
    x_0 = Eigen::Matrix<double,Sckf::X_STATE_VECTOR_SIZE,1>::Zero();
//     x_0.block<NUMAXIS, 1> (3*NUMAXIS,0) = _gbiasof.value();
//     x_0.block<NUMAXIS, 1> (4*NUMAXIS,0) = _abiasof.value();
    mysckf.setStatex(x_0);
    mysckf.setBiasOffset(_gbiasof.value(), _abiasof.value());
    
    /** Info and Warnings about the Task **/
    if (_calibrated_sensors.connected())
    {
	RTT::log(RTT::Warning) << "IMU Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "IMU samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Malfunction on the task!!" << RTT::endlog();
    }
    
    if (_hbridge_samples.connected())
    {
	RTT::log(RTT::Warning) << "Hbridge Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "Hbridge samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Malfunction on the task!!" << RTT::endlog();
    }
    
    if (_systemstate_samples.connected())
    {
	RTT::log(RTT::Warning) << "System State Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "System State samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Malfunction on the task!!" << RTT::endlog();
    }
    
    if (_torque_estimated.connected())
    {
	RTT::log(RTT::Warning) << "Wheel Torque estimation samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "Wheel Torque estimation samples NO connected." << RTT::endlog();
    }
    
    if (_ground_forces_estimated.connected())
    {
	RTT::log(RTT::Warning) << "Wheel ground force estimation samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "Wheel ground force estimation samples NO connected." << RTT::endlog();
    }
    
    if (_pose_init.connected())
    {
	RTT::log(RTT::Warning) << "Initial pose connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "Initial orientation NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Initial orientation is not provided."<< RTT::endlog();
	RTT::log(RTT::Warning) << "Zero Yaw angle pointing to North is then assumed." << RTT::endlog();
	RTT::log(RTT::Warning) << "Pitch and Roll are taken from accelerometers assuming static body at Initial phase of this task." << RTT::endlog();
    }
    
    RTT::log(RTT::Warning)<<"[Info] Frequency of IMU samples[Hertz]: "<<(1.0/_calibrated_sensors_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Frequency of Hbridge Status[Hertz]: "<<(1.0/_hbridge_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Frequency of Asguard Status[Hertz]: "<<(1.0/_systemstate_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Frequency of Torque Estimator[Hertz]: "<<(1.0/_torque_estimated_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Frequency of Ground Force Estimator[Hertz]: "<<(1.0/_ground_forces_estimated_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Filter running at Frequency[Hertz]: "<<_filter_frequency.value()<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] numberIMUSamples: "<<numberIMUSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] numberHbridgeSamples: "<<numberHbridgeSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] numberAsguardStatusSamples: "<<numberAsguardStatusSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] numberForceSamples: "<<numberForceSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] numberTorqueSamples: "<<numberTorqueSamples<<RTT::endlog();
    
    
    /** envire **/
    mpSlip = new envire::MLSGrid(_grid_resolution.get(), _grid_resolution.get(), _patch_size.get(), _patch_size.get(),
            _grid_center_x.get(), _grid_center_y.get());
   
    mEnv.attachItem(mpSlip);
   
    /** Create envire components. **/
    envire::FrameNode* p_fn = NULL;
   
    p_fn = new envire::FrameNode();
    mEnv.getRootNode()->addChild(p_fn);
    mpSlip->setFrameNode(p_fn);
    
    /** Emitter **/
    mEmitter = new envire::OrocosEmitter(_envire_environment_out);
    mEmitter->useContextUpdates(&mEnv); // Without this line the parents are not set(?)
    mEmitter->useEventQueue( true );
    mEmitter->attach(&mEnv);
    
    /** Set a flat surface **/
    for(register int i=0; i<100; i++ )
    {
	for(register int j=0; j<100; j++ )
	{
	    double h = 0.0;
	    mpSlip->insertTail( i, j, envire::MLSGrid::SurfacePatch( h, 0.05 ) );
	}
    }
    mpSlip->itemModified();

    /** Envire outport **/
    this->sendEnvireEnvironment();


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
}

void Task::getInputPortValues()
{
    unsigned int cbHbridgesize = cbHbridges.size();
    unsigned int cbAsguardsize = cbAsguard.size();
    unsigned int cbIMUsize = cbIMU.size();
    
    /** ********* **/
    /** Hbridges **/
    for (unsigned int i = 0; i<asguard::NUMBER_OF_WHEELS; i++)
    {
	hbridgeStatus.states[i].current = 0.0;
	hbridgeStatus.states[i].position = 0.0;
	hbridgeStatus.states[i].positionExtern = 0.0;
	hbridgeStatus.states[i].pwm = 0.0;
    }
	
    /** Process the buffer **/
    for (register unsigned int i = 0; i<cbHbridgesize; i++)
    {
	for (register unsigned int j = 0; j<asguard::NUMBER_OF_WHEELS; j++)
	{
	    hbridgeStatus.states[j].current += cbHbridges[i].states[j].current;
	    hbridgeStatus.states[j].pwm += cbHbridges[i].states[j].pwm;
	}
    }
    
    /** Set the time **/
    hbridgeStatus.time = (cbHbridges[0].time + cbHbridges[cbHbridgesize-1].time)/2.0;
    
    hbridgeStatus.index = cbHbridges[cbHbridgesize-1].index;
    
    for (register unsigned int i = 0; i<asguard::NUMBER_OF_WHEELS; i++)
    {
	hbridgeStatus.states[i].current /= cbHbridgesize; //the mean current in mA
	hbridgeStatus.states[i].position = cbHbridges[cbHbridgesize-1].states[i].position;
	hbridgeStatus.states[i].positionExtern = cbHbridges[cbHbridgesize-1].states[i].positionExtern;
	hbridgeStatus.states[i].pwm /= cbHbridgesize; //the mean PWM signal duty cicle [-1,1]
    }
	
    /** ************** **/
    /** Asguard status **/
    asguardStatus.asguardVoltage = 0.00;
    
    /** Process the buffer **/
    for (register unsigned int i=0; i<cbAsguardsize; i++)
    {
	asguardStatus.asguardVoltage += cbAsguard[i].asguardVoltage;
    }
    
    asguardStatus.asguardJointEncoder = cbAsguard[cbAsguardsize-1].asguardJointEncoder;
    asguardStatus.systemState = cbAsguard[cbAsguardsize-1].systemState;
    asguardStatus.packetsPerSec = cbAsguard[cbAsguardsize-1].packetsPerSec;
    asguardStatus.controlPacketsPerSec = cbAsguard[cbAsguardsize-1].controlPacketsPerSec;
    
    /** Set the time **/
    asguardStatus.time = (cbAsguard[0].time + cbAsguard[cbAsguardsize-1].time)/2.0;
    
    asguardStatus.asguardVoltage = asguardStatus.asguardVoltage/cbAsguardsize; //the mean voltage
    
    /** ************ **/
    /** IMU samples **/
    imuSamples.acc.setZero();
    imuSamples.gyro.setZero();
    imuSamples.mag.setZero();
    
    /** Process the buffer **/
    for (register unsigned int i=0; i<cbIMUsize; i++)
    {
	imuSamples.acc += cbIMU[i].acc;
	imuSamples.gyro += cbIMU[i].gyro;
	imuSamples.mag += cbIMU[i].mag;
    }
    
    /** Set the time **/
    imuSamples.time = (cbIMU[0].time + cbIMU[cbIMUsize-1].time)/2.0;
    
    /** Set the mean of this time interval **/
    imuSamples.acc /= cbIMUsize;
    imuSamples.gyro /= cbIMUsize;
    imuSamples.mag /= cbIMUsize;
    
    /** Set all counters to zero **/
    counterHbridgeSamples = 0;
    counterAsguardStatusSamples = 0;
    counterIMUSamples = 0;
    counterTorqueSamples = 0;
    counterForceSamples = 0;
    
    /** Set the previous values to the current in case of NaN values **/
    if (prevImuSamples.acc[0] == base::NaN<double>() && prevImuSamples.acc[1] == base::NaN<double>() && prevImuSamples.acc[2] == base::NaN<double>())
	prevImuSamples = imuSamples;
    
    if (prevHbridgeStatus.index == base::NaN<unsigned int>())
	prevHbridgeStatus = hbridgeStatus;
    
    if (base::isNaN(prevAsguardStatus.asguardJointEncoder))
	prevAsguardStatus = asguardStatus;
    
    return;
}

void Task::calculateFootPoints()
{
    /** For the Wheel 0 Rear Left **/
    wheelRL.Body2FootPoint (0, hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC0RL2body = wheelRL.getBody2FootPoint();
    wheelRL.Body2FootPoint (1, hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC1RL2body = wheelRL.getBody2FootPoint();
    wheelRL.Body2FootPoint (2, hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC2RL2body = wheelRL.getBody2FootPoint();
    wheelRL.Body2FootPoint (3, hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC3RL2body = wheelRL.getBody2FootPoint();
    wheelRL.Body2FootPoint (4, hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC4RL2body = wheelRL.getBody2FootPoint();
    
//     wheelRL.Body2ContactPoint (hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
//     rbsC2RL2body = wheelRL.getBody2ContactPoint();
    
    /** For the Wheel 1 Rear Right **/
    wheelRR.Body2FootPoint (0, hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC0RR2body = wheelRR.getBody2FootPoint();
    wheelRR.Body2FootPoint (1, hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC1RR2body = wheelRR.getBody2FootPoint();
    wheelRR.Body2FootPoint (2, hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC2RR2body = wheelRR.getBody2FootPoint();
    wheelRR.Body2FootPoint (3, hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC3RR2body = wheelRR.getBody2FootPoint();
    wheelRR.Body2FootPoint (4, hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC4RR2body = wheelRR.getBody2FootPoint();
    
//     wheelRR.Body2ContactPoint (hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
//     rbsC2RR2body = wheelRR.getBody2ContactPoint();
    
    /** For the Wheel 2 Forward Right **/
    wheelFR.Body2FootPoint (0, hbridgeStatus.states[2].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC0FR2body = wheelFR.getBody2FootPoint();
    wheelFR.Body2FootPoint (1, hbridgeStatus.states[2].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC1FR2body = wheelFR.getBody2FootPoint();
    wheelFR.Body2FootPoint (2, hbridgeStatus.states[2].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC2FR2body = wheelFR.getBody2FootPoint();
    wheelFR.Body2FootPoint (3, hbridgeStatus.states[2].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC3FR2body = wheelFR.getBody2FootPoint();
    wheelFR.Body2FootPoint (4, hbridgeStatus.states[2].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC4FR2body = wheelFR.getBody2FootPoint();
    
    	
    /** For the Wheel 3 Forward Left **/
    wheelFL.Body2FootPoint (0, hbridgeStatus.states[3].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC0FL2body = wheelFL.getBody2FootPoint();
    wheelFL.Body2FootPoint (1, hbridgeStatus.states[3].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC1FL2body = wheelFL.getBody2FootPoint();
    wheelFL.Body2FootPoint (2, hbridgeStatus.states[3].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC2FL2body = wheelFL.getBody2FootPoint();
    wheelFL.Body2FootPoint (3, hbridgeStatus.states[3].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC3FL2body = wheelFL.getBody2FootPoint();
    wheelFL.Body2FootPoint (4, hbridgeStatus.states[3].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC4FL2body = wheelFL.getBody2FootPoint();
    
//     wheelFL.Body2ContactPoint (hbridgeStatus.states[3].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
//     rbsC2FL2body = wheelFL.getBody2ContactPoint();
    
    
    return;

}

void Task::selectContactPoints(std::vector<int> &contactPoints)
{
    std::vector<double> footFR;
    std::vector<double> footFL;
    std::vector<double> footRR;
    std::vector<double> footRL;

    /** For the FL wheel **/
    footFL.resize(5);
    
    /** Z-coordinate of the foot **/
    footFL[0] = rbsC0FL2body.position(2);
    footFL[1] = rbsC1FL2body.position(2);
    footFL[2] = rbsC2FL2body.position(2);
    footFL[3] = rbsC3FL2body.position(2);
    footFL[4] = rbsC4FL2body.position(2);
    
    contactPoints[3] = std::distance(footFL.begin(), std::min_element(footFL.begin(), footFL.end()));
    
    #ifdef DEBUG_PRINTS
    std::cout<<"FL position: "<< rbsC0FL2body.position(2)<<"\n";
    std::cout<<"footFL: "<<footFL[0]<<" "<<footFL[1]<<" "<<footFL[2]<<" "<<footFL[3]<<" "<<footFL[4]<<"\n";
    std::cout<<"contactPoints[3]: "<< contactPoints[3]<<"\n";
    #endif
    
    /** For the FR wheel **/
    footFR.resize(5);
    
    footFR[0] = rbsC0FR2body.position(2);
    footFR[1] = rbsC1FR2body.position(2);
    footFR[2] = rbsC2FR2body.position(2);
    footFR[3] = rbsC3FR2body.position(2);
    footFR[4] = rbsC4FR2body.position(2);
    
    contactPoints[2] = std::distance(footFR.begin(), std::min_element(footFR.begin(), footFR.end()));
    
    /** For the RR wheel **/
    footRR.resize(5);
    
    footRR[0] = rbsC0RR2body.position(2);
    footRR[1] = rbsC1RR2body.position(2);
    footRR[2] = rbsC2RR2body.position(2);
    footRR[3] = rbsC3RR2body.position(2);
    footRR[4] = rbsC4RR2body.position(2);
    
    contactPoints[1] = std::distance(footRR.begin(), std::min_element(footRR.begin(), footRR.end()));
    
    /** For the RL wheel **/
    footRL.resize(5);
    
    footRL[0] = rbsC0RL2body.position(2);
    footRL[1] = rbsC1RL2body.position(2);
    footRL[2] = rbsC2RL2body.position(2);
    footRL[3] = rbsC3RL2body.position(2);
    footRL[4] = rbsC4RL2body.position(2);
    
    contactPoints[0] = std::distance(footRL.begin(), std::min_element(footRL.begin(), footRL.end()));
    
    return;
    
}

void Task::calculateEncodersVelocities()
{
    base::Time hbridgeDelta_t = hbridgeStatus.time - prevHbridgeStatus.time;
    base::Time passiveJointDelta_t = asguardStatus.time - prevAsguardStatus.time;
    base::Time imuDelta_t = imuSamples.time - prevImuSamples.time;
    
    /** Set the correct size for the joint velocities vector **/
    vjoints.resize (1 + NUMBER_WHEELS, 1);
   
    #ifdef DEBUG_PRINTS
    std::cout<<"Timestamp New: "<< asguardStatus.time.toMicroseconds() <<" Timestamp Prev: "<<prevAsguardStatus.time.toMicroseconds()<<"\n";
    std::cout<<"Delta time(passiveJoint): "<< passiveJointDelta_t.toSeconds()<<"\n";
    std::cout<<"Timestamp New: "<< imuSamples.time.toMicroseconds() <<" Timestamp Prev: "<<prevImuSamples.time.toMicroseconds()<<"\n";
    std::cout<<"Delta time(IMU): "<< imuDelta_t.toSeconds()<<"\n";
    std::cout<<"New: "<< asguardStatus.asguardJointEncoder <<" Prev: "<<prevAsguardStatus.asguardJointEncoder<<"\n";
    #endif
    
    /** Passive joint velocity **/
    vjoints(0) = (asguardStatus.asguardJointEncoder - prevAsguardStatus.asguardJointEncoder)/delta_t; //passive joints
    
    /** Velocities for the vector **/
    for (int i = 1; i< (NUMBER_WHEELS + 1); i++)
    {
	#ifdef DEBUG_PRINTS
	std::cout<<"Timestamp New: "<< hbridgeStatus.time.toMicroseconds() <<" Timestamp Prev: "<<prevHbridgeStatus.time.toMicroseconds()<<"\n";
	std::cout<<"Delta time: "<< hbridgeDelta_t.toSeconds()<<"\n";
	std::cout<<"New: "<< hbridgeStatus.states[i-1].positionExtern <<" Prev: "<<prevHbridgeStatus.states[i-1].positionExtern<<"\n";
	#endif
	vjoints(i) = (hbridgeStatus.states[i-1].positionExtern - prevHbridgeStatus.states[i-1].positionExtern)/delta_t; //wheel rotation
    }
    
    #ifdef DEBUG_PRINTS
    std::cout<<"Encoders velocities:\n"<<vjoints<<"\n";
    #endif
    
    /** Set the previous values for the velocities **/
    prevHbridgeStatus = hbridgeStatus;
    prevAsguardStatus = asguardStatus;
    prevImuSamples = imuSamples;
    
    return;
}

void Task::compositeMotionJacobians()
{
    /** Set the proper size of the matrices **/
    E.resize(localization::NUMBER_OF_WHEELS*(2*NUMAXIS), 2*NUMAXIS);
    J.resize(localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::ENCODERS_VECTOR_SIZE + localization::SLIP_VECTOR_SIZE + localization::NUMBER_OF_WHEELS);
    
    Anav.resize(localization::NUMBER_OF_WHEELS*(2*NUMAXIS), NUMAXIS+2*localization::NUMBER_OF_WHEELS);
    Bnav.resize(localization::NUMBER_OF_WHEELS*(2*NUMAXIS), NUMAXIS+localization::ENCODERS_VECTOR_SIZE);
    Aslip.resize(localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::SLIP_VECTOR_SIZE);
    Bslip.resize(localization::NUMBER_OF_WHEELS*(2*NUMAXIS), 2*NUMAXIS + ENCODERS_VECTOR_SIZE + NUMBER_OF_WHEELS);
    J.setZero(); Anav.setZero(); Bnav.setZero(); Aslip.setZero(); Bslip.setZero();
    
    /** Compute the composite rover equation matrix E **/
    E.block<2*NUMAXIS, 2*NUMAXIS>(0,0) = Eigen::Matrix <double, 2*NUMAXIS, 2*NUMAXIS>::Identity();
    E.block<2*NUMAXIS, 2*NUMAXIS>(2*NUMAXIS, 0) = Eigen::Matrix <double, 2*NUMAXIS, 2*NUMAXIS>::Identity();
    E.block<2*NUMAXIS, 2*NUMAXIS>(2*(2*NUMAXIS), 0) = Eigen::Matrix <double, 2*NUMAXIS, 2*NUMAXIS>::Identity();
    E.block<2*NUMAXIS, 2*NUMAXIS>(3*(2*NUMAXIS), 0) = Eigen::Matrix <double, 2*NUMAXIS, 2*NUMAXIS>::Identity();
    
    #ifdef DEBUG_PRINTS
    std::cout<< "Composite matrices\n";
    std::cout<< "E is of size "<<E.rows()<<"x"<<E.cols()<<"\n";
    std::cout << "The E matrix \n" << E << std::endl;
    #endif
    
    /** Compute the rover Jacobian matrix **/
    
    /** Twist passive joint **/
    J.col(0).block<2*NUMAXIS,1>(0, 0) = jacobRL.col(1);
    J.col(0).block<2*NUMAXIS,1>(2*NUMAXIS, 0) = jacobRR.col(1);
    
    /** Motor encoders **/
    J.col(1).block<2*NUMAXIS,1>(0, 0) = jacobRL.col(0);
    
    J.col(2).block<2*NUMAXIS,1>(2*NUMAXIS, 0) = jacobRR.col(0);
        
    J.col(3).block<2*NUMAXIS,1>(2*(2*NUMAXIS), 0) = jacobFR.col(0);
    
    J.col(4).block<2*NUMAXIS,1>(3*(2*NUMAXIS), 0) = jacobFL.col(0);
    
    /** Slip vector x for RL wheel **/
    J.col(5).block<2*NUMAXIS,1>(0, 0) = jacobRL.col(2);
    
    /** Slip vector y for RL wheel **/
    J.col(6).block<2*NUMAXIS,1>(0, 0) = jacobRL.col(3);
    
    /** Slip vector z for RL wheel **/
    J.col(7).block<2*NUMAXIS,1>(0, 0) = jacobRL.col(4);
    
    /** Slip vector x for RR wheel **/
    J.col(8).block<2*NUMAXIS,1>(2*NUMAXIS, 0) = jacobRR.col(2);
    
    /** Slip vector y for RR wheel **/
    J.col(9).block<2*NUMAXIS,1>(2*NUMAXIS, 0) = jacobRR.col(3);
    
    /** Slip vector z for RR wheel **/
    J.col(10).block<2*NUMAXIS,1>(2*NUMAXIS, 0) = jacobRR.col(4);
    
    /** Slip vector x for FR wheel **/
    J.col(11).block<2*NUMAXIS,1>(2*(2*NUMAXIS), 0) = jacobFR.col(1);
    
    /** Slip vector y for FR wheel **/
    J.col(12).block<2*NUMAXIS,1>(2*(2*NUMAXIS), 0) = jacobFR.col(2);
    
    /** Slip vector z for FR wheel **/
    J.col(13).block<2*NUMAXIS,1>(2*(2*NUMAXIS), 0) = jacobFR.col(3);
    
    /** Slip vector x for FL wheel **/
    J.col(14).block<2*NUMAXIS,1>(3*(2*NUMAXIS), 0) = jacobFL.col(1);
    
    /** Slip vector y for FL wheel **/
    J.col(15).block<2*NUMAXIS,1>(3*(2*NUMAXIS), 0) = jacobFL.col(2);
    
    /** Slip vector z for FL wheel **/
    J.col(16).block<2*NUMAXIS,1>(3*(2*NUMAXIS), 0) = jacobFL.col(3);
    
    /** Contact angle for RL wheel **/
    J.col(17).block<2*NUMAXIS,1>(0, 0) = jacobRL.col(5);
    
    /** Contact angle for RR wheel **/
    J.col(18).block<2*NUMAXIS,1>(2*NUMAXIS, 0) = jacobRR.col(5);
    
    /** Contact angle for FR wheel **/
    J.col(19).block<2*NUMAXIS,1>(2*(2*NUMAXIS), 0) = jacobFR.col(4);
    
    /** Contact angle for FL wheel **/
    J.col(20).block<2*NUMAXIS,1>(3*(2*NUMAXIS), 0) = jacobFL.col(4);
    
    #ifdef DEBUG_PRINTS
    std::cout<< "J is of size "<<J.rows()<<"x"<<J.cols()<<"\n";
    std::cout << "The J matrix \n" << J << std::endl;
    #endif
    
    
    /** Form the matrices for the Navigation Kinematics **/
    Anav.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), NUMAXIS>(0,0) = E.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), NUMAXIS> (0,0);
    Anav.col(3) = -J.col(7); Anav.col(4) = -J.col(10); Anav.col(5) = -J.col(13); Anav.col(6) = -J.col(16);
    Anav.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::NUMBER_OF_WHEELS>(0,NUMAXIS+localization::NUMBER_OF_WHEELS) =
	-J.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::NUMBER_OF_WHEELS>(0,localization::ENCODERS_VECTOR_SIZE+localization::SLIP_VECTOR_SIZE);
    Bnav.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), NUMAXIS> (0,0) = -E.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), NUMAXIS> (0,NUMAXIS);
    Bnav.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::ENCODERS_VECTOR_SIZE> (0,NUMAXIS) = J.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::ENCODERS_VECTOR_SIZE> (0,0);
    
    /** Form the matrix for the slip kinematics **/
    Aslip.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::SLIP_VECTOR_SIZE>(0,0) =
	-J.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::SLIP_VECTOR_SIZE>(0,localization::ENCODERS_VECTOR_SIZE);
    Bslip.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), 2*NUMAXIS> (0,0) = -E;
    Bslip.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::ENCODERS_VECTOR_SIZE> (0,2*NUMAXIS) =
	J.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::ENCODERS_VECTOR_SIZE> (0,0);
    Bslip.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::NUMBER_OF_WHEELS> (0,(2*NUMAXIS)+localization::ENCODERS_VECTOR_SIZE) =
	J.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::NUMBER_OF_WHEELS> (0,localization::ENCODERS_VECTOR_SIZE+localization::SLIP_VECTOR_SIZE);


    #ifdef DEBUG_PRINTS
    std::cout<< "Anav is of size "<<Anav.rows()<<"x"<<Anav.cols()<<"\n";
    std::cout<< "The Anav matrix \n" << Anav << std::endl;
    std::cout<< "Bnav is of size "<<Bnav.rows()<<"x"<<Bnav.cols()<<"\n";
    std::cout<< "The Bnav matrix \n" << Bnav << std::endl;
    std::cout<< "Aslip is of size "<<Aslip.rows()<<"x"<<Aslip.cols()<<"\n";
    std::cout<< "The Aslip matrix \n" << Aslip << std::endl;
    std::cout<< "Bslip is of size "<<Bslip.rows()<<"x"<<Bslip.cols()<<"\n";
    std::cout<< "The Bslip matrix \n" << Bslip << std::endl;
    #endif
    
    return;
}


void Task::toAsguardBodyState()
{
    register int i;
    
    /** Asguard BodyState (for visualization) **/
    asguard::BodyState asguardBodyState;
    
    asguardBodyState.time = hbridgeStatus.time;
    asguardBodyState.twistAngle = asguardStatus.asguardJointEncoder;
    
    /** For wheel FL (index 3) **/
    for (i = 0; i< FEET_PER_WHEEL; i++)
    {
	asguard::WheelContact mContact;
	mContact.angle = 0.00;
	
	if (i == this->contactPoints[3])
	    mContact.contact = 1.0;
	else
	    mContact.contact = 0.0;
	
	asguardBodyState.setWheelContact(wheelFL.getWheelIdx(), i, mContact);
	asguardBodyState.setWheelPos(wheelFL.getWheelIdx(), hbridgeStatus.states[3].positionExtern);
    }
    
    
    /** For wheel FR (index 2) **/
    for (i = 0; i< FEET_PER_WHEEL; i++)
    {
	asguard::WheelContact mContact;
	mContact.angle = 0.00;
	
	if (i == this->contactPoints[2])
	    mContact.contact = 1.0;
	else
	    mContact.contact = 0.0;
	
	asguardBodyState.setWheelContact(wheelFR.getWheelIdx(), i, mContact);
	asguardBodyState.setWheelPos(wheelFR.getWheelIdx(), hbridgeStatus.states[2].positionExtern);
    }
    
    /** For wheel RR (index 1) **/
    for (i = 0; i< FEET_PER_WHEEL; i++)
    {
	asguard::WheelContact mContact;
	mContact.angle = 0.00;
	
	if (i == this->contactPoints[1])
	    mContact.contact = 1.0;
	else
	    mContact.contact = 0.0;
	
	asguardBodyState.setWheelContact(wheelRR.getWheelIdx(), i, mContact);
	asguardBodyState.setWheelPos(wheelRR.getWheelIdx(), hbridgeStatus.states[1].positionExtern);
    }
    
    /** For wheel RL (index 0) **/
    for (i = 0; i< FEET_PER_WHEEL; i++)
    {
	asguard::WheelContact mContact;
	mContact.angle = 0.00;
	
	if (i == this->contactPoints[0])
	    mContact.contact = 1.0;
	else
	    mContact.contact = 0.0;
	
	asguardBodyState.setWheelContact(wheelRL.getWheelIdx(), i, mContact);
	asguardBodyState.setWheelPos(wheelRL.getWheelIdx(), hbridgeStatus.states[0].positionExtern);
    }
    
    _bodystate_samples.write(asguardBodyState);
    
    
    /** For the movement of the points with the BC **/
    rbsC0FL2body.setTransform(rbsBC.getTransform()*rbsC0FL2body.getTransform());
    rbsC1FL2body.setTransform(rbsBC.getTransform()*rbsC1FL2body.getTransform());
    rbsC2FL2body.setTransform(rbsBC.getTransform()*rbsC2FL2body.getTransform());
    rbsC3FL2body.setTransform(rbsBC.getTransform()*rbsC3FL2body.getTransform());
    rbsC4FL2body.setTransform(rbsBC.getTransform()*rbsC4FL2body.getTransform());
    
    rbsC0FR2body.setTransform(rbsBC.getTransform()*rbsC0FR2body.getTransform());
    rbsC1FR2body.setTransform(rbsBC.getTransform()*rbsC1FR2body.getTransform());
    rbsC2FR2body.setTransform(rbsBC.getTransform()*rbsC2FR2body.getTransform());
    rbsC3FR2body.setTransform(rbsBC.getTransform()*rbsC3FR2body.getTransform());
    rbsC4FR2body.setTransform(rbsBC.getTransform()*rbsC4FR2body.getTransform());
    
    rbsC0RL2body.setTransform(rbsBC.getTransform()*rbsC0RL2body.getTransform());
    rbsC1RL2body.setTransform(rbsBC.getTransform()*rbsC1RL2body.getTransform());
    rbsC2RL2body.setTransform(rbsBC.getTransform()*rbsC2RL2body.getTransform());
    rbsC3RL2body.setTransform(rbsBC.getTransform()*rbsC3RL2body.getTransform());
    rbsC4RL2body.setTransform(rbsBC.getTransform()*rbsC4RL2body.getTransform());
    
    rbsC0RR2body.setTransform(rbsBC.getTransform()*rbsC0RR2body.getTransform());
    rbsC1RR2body.setTransform(rbsBC.getTransform()*rbsC1RR2body.getTransform());
    rbsC2RR2body.setTransform(rbsBC.getTransform()*rbsC2RR2body.getTransform());
    rbsC3RR2body.setTransform(rbsBC.getTransform()*rbsC3RR2body.getTransform());
    rbsC4RR2body.setTransform(rbsBC.getTransform()*rbsC4RR2body.getTransform());
    
    
    
    /** Write values in the output ports **/
    _C0FL2body_out.write(rbsC0FL2body);
    _C1FL2body_out.write(rbsC1FL2body);
    _C2FL2body_out.write(rbsC2FL2body);
    _C3FL2body_out.write(rbsC3FL2body);
    _C4FL2body_out.write(rbsC4FL2body);
    
    _C0FR2body_out.write(rbsC0FR2body);
    _C1FR2body_out.write(rbsC1FR2body);
    _C2FR2body_out.write(rbsC2FR2body);
    _C3FR2body_out.write(rbsC3FR2body);
    _C4FR2body_out.write(rbsC4FR2body);
    
    _C0RR2body_out.write(rbsC0RR2body);
    _C1RR2body_out.write(rbsC1RR2body);
    _C2RR2body_out.write(rbsC2RR2body);
    _C3RR2body_out.write(rbsC3RR2body);
    _C4RR2body_out.write(rbsC4RR2body);
    
    _C0RL2body_out.write(rbsC0RL2body);
    _C1RL2body_out.write(rbsC1RL2body);
    _C2RL2body_out.write(rbsC2RL2body);
    _C3RL2body_out.write(rbsC3RL2body);
    _C4RL2body_out.write(rbsC4RL2body);

}


void Task::toDebugPorts()
{
    base::samples::RigidBodyState rbsIMU;
    base::samples::RigidBodyState rbsModel;
    base::samples::RigidBodyState rbsCorrected;
    base::samples::RigidBodyState rbsVicon;
    localization::FilterInfo finfo;
    localization::SlipInfo sinfo;

    /** Port out the slip vectors **/
    mymeasure->toSlipInfo(sinfo);
    sinfo.time = rbsBC.time;
    _slip_vector.write(sinfo);
    
    /** Port out the contact angle **/
//     _contact_angle.write(mysckf.filtermeasurement.getContactAnglesVelocity()*this->delta_t);
    
    /** Port out the imu velocities in body frame **/
    rbsIMU.invalidate();
    rbsIMU.time = rbsBC.time;
    rbsIMU.position = mysckf.filtermeasurement.getLinearAcceleration();//imuSamples.acc;//store in position the acceleration on body frame (for debugging)
    rbsIMU.velocity = mysckf.filtermeasurement.getLinearVelocities();
    rbsIMU.angular_velocity = mysckf.filtermeasurement.getAngularVelocities();
    _incre_velocities_imu.write(rbsIMU);
    
    /** Port out the incremental velocity computed in the model (body frame) **/
    rbsModel.invalidate();
    rbsModel.time = rbsBC.time;
    rbsModel.velocity = mysckf.filtermeasurement.getIncrementalVeloModel();
    _incre_velocities_model.write(rbsModel);
    
    /** Port out the velocity computed in the model (body frame) **/
    rbsModel.velocity = mysckf.filtermeasurement.getCurrentVeloModel();
    _velocities_model.write(rbsModel);
    
    rbsCorrected.invalidate();
    rbsCorrected.time = rbsBC.time;
    rbsCorrected.velocity =  mysckf.getVeloTruth().data;//truth = estimated + error
    _velocities_corrected.write(rbsCorrected);
    
    /** Port out the incremental velocity error **/
    base::samples::RigidBodyState rbsVelError;
    rbsVelError.time = rbsBC.time;
    rbsVelError.velocity = mysckf.getVeloError().data; //error = truth - estimated
    _velocities_error.write(rbsVelError);
    
    rbsVelError.velocity = mysckf.getIncreVeloError().data;
    _incre_velocities_error.write(rbsVelError);
    
    /** Port out the info comming from vicon **/
    rbsVicon = poseInit;
//     rbsVicon.velocity = mysckf.getAttitude() * poseInit.velocity; // velocity in body frame
    rbsVicon.time = rbsBC.time;
    _rbsVicon.write(rbsVicon);
    
    /** Port out the delta info comming from vicon **/
    rbsVicon.position = poseInit.position - prevPoseInit.position;
    rbsVicon.cov_position = poseInit.cov_position - prevPoseInit.cov_position;
    rbsVicon.velocity = poseInit.velocity - prevPoseInit.velocity;
    rbsVicon.cov_velocity = poseInit.cov_velocity - prevPoseInit.cov_velocity;
    _incre_rbsVicon.write(rbsVicon);
    
    /** IMU outport **/
    _imu_sensors_out.write(imuSamples);
    
    /** For the next export of the incremental value from Vicon data **/
    prevPoseInit = poseInit;
    
    /** Port out filter info **/
    mysckf.toFilterInfo(finfo);
    finfo.time = rbsBC.time;
    _filter_debug.write(finfo);
     
    _least_squares_error.write(leastSquaresError);
    
}

bool Task::sendEnvireEnvironment()
{
    mEmitter->setTime(rbsBC.time);
    mEmitter->flush();

    return true;
}


